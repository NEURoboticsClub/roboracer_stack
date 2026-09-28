// Standalone emergency-stop console for the RoboRacer.
//
// Run this in its OWN terminal, alongside the launch file:
//
//     ros2 run planner_node estop
//
// It cannot live inside neo_telemetry_irl_launch.py: nodes spawned by
// ros2 launch do not get a usable stdin, so keypresses would never arrive.
//
// The car is STOPPED BY DEFAULT. It only moves once this console is running and
// you press 'r'. SPACE stops it again, 'q' quits (leaving it stopped).
//
// Three independent mechanisms keep the car from running when it shouldn't:
//
//   1. This process publishes braking current straight to /commands/motor/brake
//      at 100 Hz while stopped. The VESC obeys whichever command reached it
//      last, so a 100 Hz brake drowns out the planner's 15 Hz speed commands
//      even if the planner is misbehaving and ignoring /estop entirely.
//   2. /estop is published continuously as a HEARTBEAT, not just on change. The
//      planner brakes if the heartbeat goes stale, so closing, crashing, or
//      kill -9'ing this console stops the car rather than leaving it latched
//      into "running".
//   3. The planner's own default is stopped, so the car stays put until this
//      console has explicitly said otherwise.

#include <atomic>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{
// Restores the terminal on the way out, including on an exception or Ctrl+C
// path, so a stopped e-stop console never leaves the shell without echo.
class RawTerminal
{
public:
  RawTerminal()
  {
    // O_NONBLOCK regardless of what stdin is. The VMIN/VTIME settings below only
    // apply to terminals, so without this a piped or redirected stdin would make
    // read() block and stall the brake timer.
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags != -1)
      fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    if (tcgetattr(STDIN_FILENO, &original_) != 0)
      return; // not a terminal (piped/redirected) - nothing to restore

    saved_ = true;
    termios raw = original_;
    // Turn off line buffering and echo so a single keypress arrives with no
    // Enter, and make read() return immediately when nothing is pending.
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ~RawTerminal()
  {
    if (saved_)
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
  }

  RawTerminal(const RawTerminal &) = delete;
  RawTerminal & operator=(const RawTerminal &) = delete;

private:
  termios original_{};
  bool saved_ = false;
};
} // namespace

class EstopNode : public rclcpp::Node
{
public:
  EstopNode() : Node("estop_node")
  {
    brake_current_ = declare_parameter<double>("brake_current", 4.0);

    // Latched so the planner picks up the current state whenever it starts,
    // rather than only on the next change.
    rclcpp::QoS latched(1);
    latched.transient_local().reliable();
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("/estop", latched);
    brake_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/motor/brake", 10);

    // 100 Hz: fast enough to win against the planner's 15 Hz speed commands,
    // and it doubles as the heartbeat rate.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&EstopNode::tick, this));

    printBanner();
  }

  bool shouldQuit() const { return quit_; }

  // Leave the car STOPPED on the way out. The planner's heartbeat timeout would
  // catch this anyway, but saying it explicitly stops the car immediately rather
  // than one timeout later.
  void engageOnExit()
  {
    engaged_ = true;
    for (int i = 0; i < 5; i++)
      publishEstop(true);
  }

private:
  void printBanner() const
  {
    std::printf("\r\n");
    std::printf("  ============================================\r\n");
    std::printf("   ROBORACER E-STOP  (keep this window focused)\r\n");
    std::printf("  ============================================\r\n");
    std::printf("     r      RELEASE - let the car drive\r\n");
    std::printf("     SPACE  STOP\r\n");
    std::printf("     q      quit (leaves the car stopped)\r\n");
    std::printf("\r\n  The car is stopped by default. Press r to drive.\r\n");
    std::printf("\r  state: *** STOPPED ***\r\n");
    std::fflush(stdout);
  }

  void publishEstop(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    estop_pub_->publish(msg);
  }

  void tick()
  {
    // Drain everything buffered so a burst of keypresses cannot lag the state.
    char c;
    while (::read(STDIN_FILENO, &c, 1) == 1)
    {
      if (c == ' ')
      {
        if (!engaged_)
        {
          engaged_ = true;
          publishEstop(true);
          std::printf("\r  state: *** STOPPED ***   (press r to resume)   \r\n");
          std::fflush(stdout);
        }
      }
      else if (c == 'r' || c == 'R')
      {
        if (engaged_)
        {
          engaged_ = false;
          publishEstop(false);
          std::printf("\r  state: RUNNING - car is driving               \r\n");
          std::fflush(stdout);
        }
      }
      else if (c == 'q' || c == 'Q')
      {
        quit_ = true;
      }
    }

    // Heartbeat. Published every tick, not just on change, so the planner can
    // tell "console says drive" apart from "console is gone" and brake in the
    // second case. This is what makes closing or killing this window stop the
    // car instead of leaving it running on a stale latched value.
    publishEstop(engaged_);

    // Hold the brake down for as long as the stop is engaged. Re-sending every
    // tick also keeps the VESC's own command timeout from expiring into a
    // coast, which would let the car keep rolling.
    if (engaged_)
    {
      std_msgs::msg::Float64 brake;
      brake.data = brake_current_;
      brake_pub_->publish(brake);
    }
  }

  double brake_current_ = 4.0;
  // Stopped by default: the car must be explicitly released before it moves.
  bool engaged_ = true;
  std::atomic<bool> quit_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RawTerminal raw_terminal;
  auto node = std::make_shared<EstopNode>();

  // Spin manually so 'q' can break out promptly.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->shouldQuit())
    executor.spin_once(std::chrono::milliseconds(20));

  node->engageOnExit();
  // Give the stop a moment to leave the process before the context dies.
  executor.spin_once(std::chrono::milliseconds(50));

  std::printf("\r\n  e-stop console closed - car is STOPPED.\r\n");
  std::fflush(stdout);

  rclcpp::shutdown();
  return 0;
}
