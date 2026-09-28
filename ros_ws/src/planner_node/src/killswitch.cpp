// Remote kill-switch receiver for the RoboRacer.
//
// The competition rules put the kill-switch in the VEHICLE SPECIFICATION, not in
// the software: the Operator must hold a physical remote in a raised hand and be
// able to stop the car "immediately and remotely". So the authority lives in a
// controller paired to the laptop, and this node is the car's end of that link.
//
//     laptop:  gamepad -> tools/killswitch_remote.py -> UDP @ 50 Hz
//     car:     this node -> /estop + /commands/motor/brake @ 100 Hz
//
// It speaks EXACTLY the contract estop.cpp already speaks, so the planner needs
// no changes: /estop is a 100 Hz heartbeat (not edge-triggered) and the brake is
// slammed straight at the VESC while stopped.
//
// THE CAR IS STOPPED BY DEFAULT and stays stopped unless all three hold:
//
//   1. A valid packet arrived within timeout_ms.
//   2. That packet said ARMED.
//   3. Its sequence number advanced, so a wedged or replayed sender cannot hold
//      the car armed by repeating one packet forever.
//
// Everything that can go wrong lands on (1): WiFi drops, the laptop sleeps, the
// script crashes, the gamepad's battery dies, someone closes the terminal. All
// of them look identical from here - packets stop - and all of them stop the car
// within timeout_ms. That is the whole point of the design.
//
// WHY UDP AND NOT A ROS TOPIC ACROSS THE WIFI: a kill-switch wants "newest
// datagram wins, silence means stop". DDS wants to retry, reorder and rediscover,
// and reliable+transient_local over a lossy link is precisely the wrong bias. It
// also means the laptop needs no ROS install at all.
//
// DO NOT RUN THIS AND estop.cpp AT THE SAME TIME. Both publish /estop at 100 Hz
// and the planner would see the two of them alternating.

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{
// Wire format. Fixed size, no parsing library, no ambiguity about what a short
// read means. Both multi-byte fields are network byte order so the sender can be
// anything - the reference sender is Python struct '!' on Windows.
constexpr char kMagic[4] = {'R', 'R', 'K', 'S'};
constexpr uint8_t kVersion = 1;

struct __attribute__((packed)) KillPacket
{
  char magic[4];    // "RRKS"
  uint8_t version;  // kVersion
  uint32_t token;   // shared secret, network byte order
  uint32_t seq;     // monotonic per sender run, network byte order
  uint8_t armed;    // 0 = stop, nonzero = allowed to drive
};
static_assert(sizeof(KillPacket) == 14, "wire format must stay 14 bytes");

// Owns the socket fd so no early return or exception can leak it.
class UdpSocket
{
public:
  explicit UdpSocket(uint16_t port)
  {
    // Non-blocking: this is drained from the same 100 Hz timer that publishes,
    // so a blocking read would stall the heartbeat - the one thing that must
    // never stop.
    fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0)
      return;

    // Deliberately NO SO_REUSEADDR. On Linux it would let a second instance bind
    // this same UDP port, and the kernel then hands each datagram to only one of
    // them - so a forgotten node silently swallows the operator's packets while
    // the real one sits there seeing a dead link. Without it the second instance
    // fails to bind and says so loudly, which is the failure you can debug.
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

  ~UdpSocket()
  {
    if (fd_ >= 0)
      ::close(fd_);
  }

  UdpSocket(const UdpSocket &) = delete;
  UdpSocket & operator=(const UdpSocket &) = delete;

  int fd() const { return fd_; }
  bool valid() const { return fd_ >= 0; }

private:
  int fd_ = -1;
};
} // namespace

class KillSwitchNode : public rclcpp::Node
{
public:
  KillSwitchNode() : Node("killswitch_node")
  {
    const int port = declare_parameter<int>("port", 5005);

    // Shared secret. Its job is narrow but real: at a competition every team is
    // running a hotspot in the same hall, and a stray packet that could ARM the
    // car is the one failure mode worth spending four bytes on. A mismatch is
    // fail-safe and obvious - the car simply refuses to arm.
    token_ = static_cast<uint32_t>(declare_parameter<int64_t>("token", 0x5252534BLL));

    // 400 ms, not the planner's 300 ms. Long enough to ride out an ordinary WiFi
    // hiccup without ending a run, short enough that the worst case is ~1.3 m of
    // travel at cruise (15000 eRPM ~ 3.25 m/s). Tune from the measured drop time.
    const int timeout_ms = declare_parameter<int>("timeout_ms", 400);
    timeout_ = std::chrono::milliseconds(timeout_ms);

    brake_current_ = declare_parameter<double>("brake_current", 4.0);

    // Latched, matching estop.cpp, so the planner picks up the current state
    // whenever it starts rather than only on the next change.
    rclcpp::QoS latched(1);
    latched.transient_local().reliable();
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("/estop", latched);
    brake_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/motor/brake", 10);

    socket_ = std::make_unique<UdpSocket>(static_cast<uint16_t>(port));
    if (!socket_->valid())
    {
      // Nothing to recover to: with no socket, no packet can ever arrive, so the
      // node holds the car stopped forever. That is the right failure, but it is
      // worth screaming about rather than looking like a dead gamepad.
      RCLCPP_FATAL(
        get_logger(),
        "Could not bind UDP port %d (%s). The car will stay STOPPED. "
        "Is another killswitch_node already running?",
        port, std::strerror(errno));
    }

    last_packet_ = std::chrono::steady_clock::now();

    // 100 Hz: fast enough that the brake drowns out the planner's 15 Hz speed
    // commands even if the planner is misbehaving, and it doubles as the /estop
    // heartbeat rate.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&KillSwitchNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Kill-switch listening on UDP :%d (timeout %d ms, brake %.1f A). "
      "STOPPED BY DEFAULT - car drives only while the remote says ARMED.",
      port, timeout_ms, brake_current_);
  }

  // Ctrl+C path: say STOP explicitly rather than relying on the planner noticing
  // the heartbeat stop one timeout later.
  void engageOnExit()
  {
    for (int i = 0; i < 5; i++)
      publishEstop(true);
  }

private:
  void publishEstop(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    estop_pub_->publish(msg);
  }

  // Take everything queued and keep the newest packet that survives validation.
  // Draining rather than reading one means a burst cannot leave us acting on a
  // stale datagram while a fresher one waits in the buffer.
  void drainSocket()
  {
    if (!socket_->valid())
      return;

    KillPacket pkt;
    for (;;)
    {
      const ssize_t n = ::recvfrom(socket_->fd(), &pkt, sizeof(pkt), 0, nullptr, nullptr);
      if (n < 0)
        break; // EAGAIN: nothing left this tick

      // Reject rather than break, so a flood of junk cannot starve a good packet
      // sitting behind it in the queue.
      if (static_cast<size_t>(n) != sizeof(pkt))
        continue;
      if (std::memcmp(pkt.magic, kMagic, sizeof(kMagic)) != 0)
        continue;
      if (pkt.version != kVersion)
        continue;
      if (ntohl(pkt.token) != token_)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring kill-switch packets with a bad token - sender and car disagree. "
          "The car will NOT arm until they match.");
        continue;
      }

      const uint32_t seq = ntohl(pkt.seq);
      const auto now = std::chrono::steady_clock::now();

      // Normal case: the counter moved forward. Wraps correctly at 2^32 because
      // the difference is taken as a signed 32-bit value.
      const bool advanced = static_cast<int32_t>(seq - last_seq_) > 0;

      // The sender restarted (its counter went back to 0) while we were already
      // stopped on a stale link. Accept and resync, otherwise restarting the
      // laptop script could never re-arm the car.
      //
      // Requiring a DIFFERENT sequence number here is what makes the guard hold.
      // Resyncing on any packet once stale looks equivalent but is not: a wedged
      // sender repeating one ARMED packet forever would go stale, resync on that
      // same stale packet, arm, go stale 400 ms later, and flap - leaving the car
      // armed ~97% of the time. Distinguishing "restarted" from "stuck" is the
      // entire reason the packet carries a sequence number.
      const bool restarted = (now - last_packet_) > timeout_ && seq != last_seq_;

      if (ever_seen_ && !advanced && !restarted)
        continue;

      last_seq_ = seq;
      last_packet_ = now;
      ever_seen_ = true;
      armed_ = (pkt.armed != 0);
    }
  }

  void tick()
  {
    drainSocket();

    const auto now = std::chrono::steady_clock::now();
    const bool link_stale = !ever_seen_ || (now - last_packet_) > timeout_;
    const bool stopped = link_stale || !armed_;

    if (stopped != last_reported_stopped_)
    {
      if (stopped)
        RCLCPP_WARN(
          get_logger(), "*** STOPPED *** (%s)", link_stale ? "link lost" : "remote kill");
      else
        RCLCPP_INFO(get_logger(), "ARMED - car is allowed to drive");
      last_reported_stopped_ = stopped;
    }

    if (link_stale)
    {
      const double age_sec = std::chrono::duration<double>(now - last_packet_).count();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        ever_seen_
          ? "Kill-switch link lost (%.2fs since last packet) - braking."
          : "Waiting for the kill-switch remote - braking. Start killswitch_remote.py.",
        age_sec);
    }

    // Heartbeat: published every tick, not just on change, so the planner can
    // tell "remote says drive" apart from "this node is gone" and brake in the
    // second case.
    publishEstop(stopped);

    // Hold the brake down for as long as the stop is engaged. Re-sending every
    // tick also keeps the VESC's own command timeout from expiring.
    if (stopped)
    {
      std_msgs::msg::Float64 brake;
      brake.data = brake_current_;
      brake_pub_->publish(brake);
    }
  }

  uint32_t token_ = 0;
  std::chrono::milliseconds timeout_{400};
  double brake_current_ = 4.0;

  // Stopped by default: nothing moves until a valid ARMED packet arrives.
  bool armed_ = false;
  bool ever_seen_ = false;
  bool last_reported_stopped_ = true;
  uint32_t last_seq_ = 0;
  std::chrono::steady_clock::time_point last_packet_;

  std::unique_ptr<UdpSocket> socket_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KillSwitchNode>();

  rclcpp::on_shutdown([node]() { node->engageOnExit(); });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
