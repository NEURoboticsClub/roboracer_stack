#include "planner_gap_follow.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

// ---------------------------------------------------------------------------
// BLANK TEMPLATE. The node's plumbing (lidar in, drive commands out, e-stop) is
// done for you. The only thing to write is computeDrive() below.
//
// Output target. Selected at RUNTIME by the `output_mode` parameter:
//
//   "vesc" (default)  the real car - /commands/motor/{speed,brake} and
//                     /commands/servo/position, e-stop enforced.
//   "sim"             the AutoDRIVE simulator - /autodrive/roboracer_1/
//                     {throttle,steering}_command, no e-stop console.
//
// The same DriveCommand drives both, so an algorithm tested in the sim runs on
// the car unchanged.
// ---------------------------------------------------------------------------

float angleIncrement = 0.0f; // radians between consecutive beams
float angleMin = -2.35619f;  // angle of beam 0; updated from the scan each callback

// What your algorithm hands back each scan.
struct DriveCommand
{
  // Radians, positive = LEFT, 0 = straight. Reaches full lock at
  // max_steer_angle (0.80 rad by default); anything beyond is clamped.
  float steering = 0.0f;

  // Normalized:  +1.0  full cruise speed
  //               0.0  coast (motor freewheels)
  //              <0.0  brake
  // Values between 0 and 1 are a fraction of cruise speed.
  float throttle = 0.0f;
};

// ===========================================================================
// YOUR ALGORITHM GOES HERE.
//
// `ranges` is one lidar scan: ranges[i] is the distance in meters seen by beam
// i, at angle ind2angle(i) radians (0 = straight ahead, positive = LEFT). A
// beam with no return can read 0, inf or nan - treat those as open space.
//
// Called once per scan, so whatever you return is sent to the car right away.
// ===========================================================================
DriveCommand computeDrive(const std::vector<float> &ranges)
{
  (void)ranges; // remove once you use it

  DriveCommand cmd;
  cmd.steering = 0.0f; // straight
  cmd.throttle = 0.0f; // stopped - raise this to move
  return cmd;
}

class GapFollowerNode : public rclcpp::Node
{
public:
  GapFollowerNode() : Node("gap_follower_node")
  {
    // Which stack this run drives. See the block at the top of the file.
    // Defaults to "vesc" so an un-parameterized launch is still the real car,
    // which is what every existing launch file expects.
    output_mode_ = declare_parameter<std::string>("output_mode", "vesc");
    if (output_mode_ != "vesc" && output_mode_ != "sim")
    {
      RCLCPP_ERROR(
        get_logger(), "output_mode '%s' is not 'vesc' or 'sim' - falling back to 'vesc'.",
        output_mode_.c_str());
      output_mode_ = "vesc";
    }
    sim_mode_ = (output_mode_ == "sim");

    // Speeds are in ELECTRICAL RPM, which is what the VESC speed command takes.
    // Below roughly 2000-5000 eRPM the VESC will not start the motor at all.
    cruise_erpm_ = declare_parameter<double>("cruise_erpm", 6000.0);
    // Braking asks for a low speed rather than dumping braking current, so the
    // car eases off instead of locking up. brake_current_ is only the hard stop
    // (e-stop console and Ctrl+C shutdown).
    brake_erpm_ = declare_parameter<double>("brake_erpm", 4000.0);
    brake_current_ = declare_parameter<double>("brake_current", 4.0);

    // Steering angle (radians) at which the servo reaches full lock.
    max_steer_angle_ = declare_parameter<double>("max_steer_angle", 0.80);

    invert_steering_ = declare_parameter<bool>("invert_steering", false);
    // Servo range in the VESC's normalized 0..1 units. Defaults match
    // servo_min/servo_max in vesc_driver's vesc_config.yaml so the driver never
    // has to clip what we publish.
    servo_center_ = declare_parameter<double>("servo_center", 0.5);
    servo_min_ = declare_parameter<double>("servo_min", 0.117);
    servo_max_ = declare_parameter<double>("servo_max", 0.895);

    // Simulator scaling. These convert the normalized throttle into what
    // AutoDRIVE's open-loop throttle expects. Only read in output_mode:=sim.
    sim_cruise_throttle_ = declare_parameter<double>("sim_cruise_throttle", 0.1);
    // Below this the open-loop ESC/sim simply ignores the request, so anything
    // smaller has to be pulsed rather than sent directly. A sim launch can set
    // this to 0.0.
    sim_throttle_floor_ = declare_parameter<double>("sim_throttle_floor", 0.1);
    sim_brake_throttle_ = declare_parameter<double>("sim_brake_throttle", -0.05);
    // AutoDRIVE's steering_command is NORMALIZED [-1, 1], positive LEFT. Set
    // this to -1.0 if the car in the sim mirrors every turn.
    sim_steer_sign_ = declare_parameter<double>("sim_steer_sign", 1.0);

    // INPUT: every lidar scan lands here.
    auto topic_callback = [this](sensor_msgs::msg::LaserScan::UniquePtr scan) -> void
    {
      angleIncrement = scan->angle_increment;
      angleMin = scan->angle_min;

      const DriveCommand cmd = computeDrive(scan->ranges);

      // Held stopped (real car only) => the stop has already been published.
      if (holdStopped())
        return;

      // OUTPUT. Publishing straight from the scan callback means the scan rate
      // is also the command rate, so a stalled lidar lets the VESC's own
      // command timeout stop the motor.
      outputSteeringCommand(cmd.steering);
      outputThrottleCommand(cmd.throttle);
    };

    // The scan topic is /scan on the car. The sim launch file remaps it onto
    // the AutoDRIVE bridge's /autodrive/roboracer_1/lidar.
    subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, topic_callback);

    // Only the stack actually being driven gets publishers.
    if (sim_mode_)
    {
      steering_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/roboracer_1/steering_command", 1);
      throttle_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/roboracer_1/throttle_command", 1);
    }
    else
    {
      speed_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 1);
      brake_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/brake", 1);
      servo_pub = this->create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 1);
    }

    // How long the estop heartbeat may go missing before we brake. The console
    // sends it at 100 Hz, so this is ~30 missed beats.
    estop_timeout_sec_ = declare_parameter<double>("estop_timeout_sec", 0.3);

    // No e-stop console is part of the sim stack. See holdStopped().
    if (!sim_mode_)
    {
      // Latched to match the estop console's publisher, so starting the planner
      // second still picks up an already-engaged stop instead of driving off.
      rclcpp::QoS estop_qos(1);
      estop_qos.transient_local().reliable();
      estop_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/estop", estop_qos,
        [this](std_msgs::msg::Bool::UniquePtr msg)
        {
          if (msg->data != estop_engaged_)
            RCLCPP_WARN(
              get_logger(), "E-STOP %s", msg->data ? "ENGAGED - braking" : "released - driving");
          estop_engaged_ = msg->data;
          last_estop_msg_ = std::chrono::steady_clock::now();
          estop_seen_ = true;
        });
    }

    if (sim_mode_)
      RCLCPP_INFO(
        get_logger(),
        "SIM output: throttle %.2f cruise / %.2f brake on "
        "/autodrive/roboracer_1/throttle_command, steering on "
        "/autodrive/roboracer_1/steering_command (normalized, full lock at "
        "%.2f rad, sign %+.0f). NO E-STOP - the car drives as soon as scans arrive.",
        sim_cruise_throttle_, sim_brake_throttle_, max_steer_angle_, sim_steer_sign_);
    else
      RCLCPP_INFO(
        get_logger(),
        "VESC output: cruise %.0f eRPM, brake %.0f eRPM (hard stop %.1f A). "
        "STOPPED BY DEFAULT - run 'ros2 run planner_node estop' and press r to drive.",
        cruise_erpm_, brake_erpm_, brake_current_);
  }

private:
  // Map a REP-103 steering angle in radians (positive = LEFT) onto the VESC's
  // 0..1 servo output. Each side is scaled against its own limit so an
  // off-center trim still reaches full lock in both directions.
  double steeringToServo(float angle_rad) const
  {
    if (!std::isfinite(angle_rad))
      return servo_center_;

    // This car's servo runs OPPOSITE to REP-103: 0.117 is full LEFT and 0.895
    // full RIGHT, so a positive (left) angle has to land BELOW servo_center_.
    // invert_steering_ is the trackside override for a car wired the other way.
    constexpr double servo_polarity = -1.0;
    const double angle = servo_polarity *
      (invert_steering_ ? -static_cast<double>(angle_rad)
                        : static_cast<double>(angle_rad));
    const double normalized = std::clamp(angle / max_steer_angle_, -1.0, 1.0);
    const double servo = normalized >= 0.0
      ? servo_center_ + normalized * (servo_max_ - servo_center_)
      : servo_center_ + normalized * (servo_center_ - servo_min_);
    return std::clamp(servo, servo_min_, servo_max_);
  }

  // The car is stopped unless the estop console is alive AND has released it.
  // Both halves matter: estop_engaged_ starts true so nothing moves before the
  // console appears, and the heartbeat check means a closed or killed console
  // stops the car instead of leaving it running on a stale value.
  bool stopRequested()
  {
    if (estop_engaged_)
      return true;

    const double age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_estop_msg_).count();

    if (!estop_seen_ || age_sec > estop_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        estop_seen_
          ? "E-STOP heartbeat lost (%.2fs) - braking. Restart: ros2 run planner_node estop"
          : "Waiting for e-stop console - braking. Run 'ros2 run planner_node estop', press r.",
        age_sec);
      return true;
    }

    return false;
  }

  // Hold the car stopped if the e-stop says so, publishing the stop itself.
  // Returns true when the caller should emit nothing further this frame.
  // Only the real car has an e-stop console, so output_mode:=sim never holds.
  bool holdStopped()
  {
    if (sim_mode_)
      return false;

    if (!stopRequested())
      return false;

    std_msgs::msg::Float64 msg;
    msg.data = servo_center_;
    servo_pub->publish(msg);
    msg.data = brake_current_;
    brake_pub->publish(msg);
    return true;
  }

  // The single point where a throttle becomes an actuator command.
  // The `output_mode` parameter (see the top of this file) picks the stack.
  void outputThrottleCommand(float throttle)
  {
    if (!std::isfinite(throttle))
      throttle = 0.0f;
    throttle = std::clamp(throttle, -1.0f, 1.0f);

    if (sim_mode_)
    {
      std_msgs::msg::Float32 msg;

      if (throttle < 0.0f)
      {
        msg.data = static_cast<float>(sim_brake_throttle_);
      }
      else if (throttle == 0.0f)
      {
        msg.data = 0.0f;
      }
      else
      {
        const float scaled = throttle * static_cast<float>(sim_cruise_throttle_);
        const float floor_v = static_cast<float>(sim_throttle_floor_);

        if (scaled >= floor_v || floor_v <= 0.0f)
        {
          msg.data = scaled;
        }
        else
        {
          // An open-loop ESC ignores anything below the floor outright, so a
          // low request has to be pulsed AT the floor with the duty cycle that
          // averages out to what was asked for.
          const long long period_ms = 500;
          const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
          const float duty = scaled / floor_v;
          const long long on_ms = static_cast<long long>(duty * period_ms);
          msg.data = (elapsed_ms % period_ms) < on_ms ? floor_v : 0.0f;
        }
      }

      throttle_pub->publish(msg);
      return;
    }

    // The VESC acts on whichever command arrived last, so exactly one of the
    // speed/brake topics gets a message per frame.
    std_msgs::msg::Float64 msg;

    if (throttle > 0.0f)
    {
      msg.data = static_cast<double>(throttle) * cruise_erpm_;
      speed_pub->publish(msg);
    }
    else if (throttle < 0.0f)
    {
      // Braking is a speed request, not a current request: the PID winds the
      // motor down to brake_erpm_ instead of slamming the brakes on.
      msg.data = brake_erpm_;
      speed_pub->publish(msg);
    }
    else
    {
      // Zero braking current lets the motor freewheel.
      msg.data = 0.0;
      brake_pub->publish(msg);
    }
  }

  // Steering's counterpart to the above, switched by the same flag so one run
  // never drives half of each stack.
  void outputSteeringCommand(float steering_angle)
  {
    if (sim_mode_)
    {
      // AutoDRIVE wants a NORMALIZED command in [-1, 1], not radians. Same
      // normalization the servo path uses.
      std_msgs::msg::Float32 msg;
      msg.data = std::isfinite(steering_angle) && max_steer_angle_ > 0.0
        ? static_cast<float>(std::clamp(
            sim_steer_sign_ * static_cast<double>(steering_angle) / max_steer_angle_,
            -1.0, 1.0))
        : 0.0f;
      steering_pub->publish(msg);
      return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = steeringToServo(steering_angle);
    servo_pub->publish(msg);
  }

public:
  // Brake burst used on shutdown (Ctrl+C) so the car stops immediately instead
  // of coasting until the VESC's own command timeout expires.
  void brakeOnShutdown()
  {
    if (!brake_pub)
      return;

    std_msgs::msg::Float64 brake_msg;
    brake_msg.data = brake_current_;
    for (int i = 0; i < 5; i++)
      brake_pub->publish(brake_msg);
  }

private:

  std::string output_mode_ = "vesc";
  bool sim_mode_ = false;
  double cruise_erpm_ = 6000.0;
  double brake_erpm_ = 4000.0;
  double brake_current_ = 4.0;
  double max_steer_angle_ = 0.80;
  bool invert_steering_ = false;
  double servo_center_ = 0.5;
  double servo_min_ = 0.117;
  double servo_max_ = 0.895;
  double sim_cruise_throttle_ = 0.1;
  double sim_throttle_floor_ = 0.1;
  double sim_brake_throttle_ = -0.05;
  double sim_steer_sign_ = 1.0;
  // Stopped by default: nothing moves until the estop console releases it.
  // Only consulted in output_mode:=vesc - the sim has no e-stop console.
  bool estop_engaged_ = true;
  bool estop_seen_ = false;
  double estop_timeout_sec_ = 0.3;
  std::chrono::steady_clock::time_point last_estop_msg_{};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub;
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GapFollowerNode>();

  // Ctrl+C path: brake before the context tears down, so the car does not coast
  // for the duration of the VESC's command timeout.
  rclcpp::on_shutdown([node]() { node->brakeOnShutdown(); });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// Angle (radians) of beam `index`: 0 = straight ahead, positive = LEFT.
float ind2angle(int index)
{
  return (index * angleIncrement) + angleMin;
}
