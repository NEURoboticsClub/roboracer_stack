#include "algorithm.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "vesc_msgs/msg/vesc_imu_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

// ---------------------------------------------------------------------------
// The node's plumbing: lidar in, drive commands out, e-stop. The algorithm
// itself lives in algorithm.cpp and there should be no need to edit this file.
//
// Output target. Selected at RUNTIME by the `output_mode` parameter:
//
//   "vesc" (default)  the real car - /commands/motor/{speed,brake} and
//                     /commands/servo/position, e-stop enforced.
//   "sim"             the AutoDRIVE simulator - /autodrive/roboracer_1/
//                     {throttle,steering}_command, no e-stop.
//
// The same DriveCommand drives both, so an algorithm tested in the sim runs on
// the car unchanged.
// ---------------------------------------------------------------------------

namespace
{
float angleIncrement = 0.0f; // radians between consecutive beams
float angleMin = -2.35619f;  // angle of beam 0; updated from the scan each callback
}

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("planner")
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

    // What the algorithm's throttle maps to. Each target has exactly two values:
    // +1.0 is its max throttle, -1.0 its max brake, and everything between scales
    // linearly toward 0. Both are magnitudes, so the sign is ignored.

    // Real car, throttle +1.0: speed in ELECTRICAL RPM, which is what the VESC
    // speed command takes: mechanical RPM x the motor's pole pairs.
    //
    // Measured on this car: 2000 eRPM is silently ignored (the motor never
    // starts - reported speed/current/duty all stay at 0), while 5000 eRPM
    // holds cleanly at ~5020 eRPM drawing ~4.4 A. The VESC speed PID has a
    // minimum ERPM below which it will not start the motor; the threshold sits
    // somewhere between 2000 and 5000 here.
    irl_max_throttle_erpm_ = std::abs(declare_parameter<double>("irl_max_throttle_erpm", 6000.0));
    // Real car, throttle -1.0: braking current in amps. Also the planner's hard
    // stop (e-stop and Ctrl+C).
    irl_max_brake_amps_ = std::abs(declare_parameter<double>("irl_max_brake_amps", 4.0));

    invert_steering_ = declare_parameter<bool>("invert_steering", false);
    // Servo range in the VESC's normalized 0..1 units. Defaults match
    // servo_min/servo_max in vesc_driver's vesc_config.yaml so the driver never
    // has to clip what we publish.
    servo_center_ = declare_parameter<double>("servo_center", 0.5);
    servo_min_ = declare_parameter<double>("servo_min", 0.117);
    servo_max_ = declare_parameter<double>("servo_max", 0.895);

    // Sim, throttle +1.0: AutoDRIVE's open-loop throttle command.
    sim_max_throttle_ = std::abs(declare_parameter<double>("sim_max_throttle", 0.1));
    // Sim, throttle -1.0: AutoDRIVE has no brake, so this is sent as reverse
    // throttle, and holding it backs the car up.
    sim_max_brake_ = std::abs(declare_parameter<double>("sim_max_brake", 0.1));
    // AutoDRIVE's steering_command is NORMALIZED [-1, 1], positive LEFT. Set
    // this to -1.0 if the car in the sim mirrors every turn.
    sim_steer_sign_ = declare_parameter<double>("sim_steer_sign", 1.0);

    // INPUT: every lidar scan lands here.
    auto topic_callback = [this](sensor_msgs::msg::LaserScan::UniquePtr scan) -> void
    {
      angleIncrement = scan->angle_increment;
      angleMin = scan->angle_min;

      // Runs even while stopped, so an algorithm that keeps state between scans
      // does not lose track of the world during an e-stop.
      const DriveCommand cmd = computeDrive(scan->ranges, latest_imu_.get());

      // Held stopped (real car only) => the stop has already been published.
      if (holdStopped())
        return;

      // OUTPUT. Publishing straight from the scan callback means the scan rate
      // is also the command rate, so a stalled lidar lets the VESC's own
      // command timeout stop the motor.
      outputSteeringCommand(cmd.steering);
      outputThrottleCommand(cmd.throttle);
    };

    // The scan topic is /scan on the car. The AutoDRIVE bridge publishes its
    // lidar on /autodrive/roboracer_1/lidar instead, so the sim launch file
    // remaps this rather than the node knowing about either name. Default QoS
    // at depth 10 is reliable/volatile, which the bridge's reliable depth-1
    // publisher matches.
    subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, topic_callback);
    imu_sub = this->create_subscription<vesc_msgs::msg::VescImuStamped>("/sensors/imu", rclcpp::SensorDataQoS(), [this](vesc_msgs::msg::VescImuStamped::SharedPtr msg) { latest_imu_ = msg; });


    // Only the stack actually being driven gets publishers, so `ros2 topic
    // list` shows what this run commands.
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

    // How long the /estop heartbeat may go missing before we brake. The
    // kill-switch sends it at 100 Hz, so this is ~30 missed beats.
    estop_timeout_sec_ = declare_parameter<double>("estop_timeout_sec", 0.3);

    // No kill-switch is part of the sim stack. See holdStopped().
    if (!sim_mode_)
    {
      // Latched to match the kill-switch's publisher, so starting the planner
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
        "SIM output: throttle +1.0 = %.2f, -1.0 = -%.2f on /autodrive/roboracer_1/throttle_command, "
        "steering on /autodrive/roboracer_1/steering_command (sign %+.0f). "
        "NO E-STOP - the car drives as soon as scans arrive.",
        sim_max_throttle_, sim_max_brake_, sim_steer_sign_);
    else
      RCLCPP_INFO(
        get_logger(),
        "VESC output: throttle +1.0 = %.0f eRPM, -1.0 = %.1f A brake. "
        "STOPPED BY DEFAULT - start killswitch_node and arm the remote to drive.",
        irl_max_throttle_erpm_, irl_max_brake_amps_);
  }

private:
  // Map a normalized steering command (positive = LEFT) onto the VESC's 0..1
  // servo output. Each side is scaled against its own limit so an off-center
  // trim still reaches full lock in both directions.
  double steeringToServo(float steering) const
  {
    if (!std::isfinite(steering))
      return servo_center_;

    // This car's servo runs OPPOSITE to REP-103: 0.117 is full LEFT and 0.895
    // full RIGHT, so a positive (left) command has to land BELOW servo_center_.
    // That fixed wiring polarity lives here; invert_steering_ stays as the
    // trackside override on top of it, for a car wired the other way round.
    constexpr double servo_polarity = -1.0;
    const double normalized = std::clamp(
      servo_polarity * (invert_steering_ ? -static_cast<double>(steering)
                                         : static_cast<double>(steering)),
      -1.0, 1.0);
    const double servo = normalized >= 0.0
      ? servo_center_ + normalized * (servo_max_ - servo_center_)
      : servo_center_ + normalized * (servo_center_ - servo_min_);
    return std::clamp(servo, servo_min_, servo_max_);
  }

  // The car is stopped unless the kill-switch is alive AND has released it.
  // Both halves matter: estop_engaged_ starts true so nothing moves before the
  // kill-switch appears, and the heartbeat check means a dead or killed
  // kill-switch stops the car instead of leaving it running on a stale value.
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
          ? "E-STOP heartbeat lost (%.2fs) - braking. Is killswitch_node still running?"
          : "Waiting for killswitch_node - braking. Start it and arm the remote.",
        age_sec);
      return true;
    }

    return false;
  }

  // Hold the car stopped if the e-stop says so, publishing the stop itself.
  // Returns true when the caller should emit nothing further this frame.
  // Only the real car has a kill-switch, so output_mode:=sim never holds.
  bool holdStopped()
  {
    if (sim_mode_)
      return false;

    // The kill-switch also brakes the VESC directly at 100 Hz, so this is the
    // cooperative half of the stop: hold the brake and straighten the wheels
    // instead of steering blind.
    if (!stopRequested())
      return false;

    std_msgs::msg::Float64 msg;
    msg.data = servo_center_;
    servo_pub->publish(msg);
    msg.data = irl_max_brake_amps_;
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
      msg.data = throttle * static_cast<float>(throttle >= 0.0f ? sim_max_throttle_ : sim_max_brake_);
      throttle_pub->publish(msg);
      return;
    }

    // The VESC acts on whichever command arrived last, so exactly one of the
    // speed/brake topics gets a message per frame.
    std_msgs::msg::Float64 msg;

    if (throttle > 0.0f)
    {
      msg.data = static_cast<double>(throttle) * irl_max_throttle_erpm_;
      speed_pub->publish(msg);
    }
    else
    {
      // Zero braking current lets the motor freewheel. Slowing down has to be
      // current rather than a low speed request: a speed request would drive a
      // car that had already stopped back up to that speed.
      msg.data = static_cast<double>(-throttle) * irl_max_brake_amps_;
      brake_pub->publish(msg);
    }
  }

  // Steering's counterpart to the above, switched by the same flag so one run
  // never drives half of each stack.
  void outputSteeringCommand(float steering)
  {
    if (sim_mode_)
    {
      std_msgs::msg::Float32 msg;
      msg.data = std::isfinite(steering)
        ? static_cast<float>(std::clamp(
            sim_steer_sign_ * static_cast<double>(steering), -1.0, 1.0))
        : 0.0f;
      steering_pub->publish(msg);
      return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = steeringToServo(steering);
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
    brake_msg.data = irl_max_brake_amps_;
    for (int i = 0; i < 5; i++)
      brake_pub->publish(brake_msg);
  }

private:

  std::string output_mode_ = "vesc";
  bool sim_mode_ = false;
  double irl_max_throttle_erpm_ = 6000.0;
  double irl_max_brake_amps_ = 4.0;
  bool invert_steering_ = false;
  double servo_center_ = 0.5;
  double servo_min_ = 0.117;
  double servo_max_ = 0.895;
  double sim_max_throttle_ = 0.1;
  double sim_max_brake_ = 0.1;
  double sim_steer_sign_ = 1.0;
  // Stopped by default: nothing moves until the kill-switch releases it.
  // Only consulted in output_mode:=vesc - the sim has no kill-switch.
  bool estop_engaged_ = true;
  bool estop_seen_ = false;
  double estop_timeout_sec_ = 0.3;
  std::chrono::steady_clock::time_point last_estop_msg_{};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
  rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr imu_sub;
  vesc_msgs::msg::VescImuStamped::SharedPtr latest_imu_;
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
  auto node = std::make_shared<PlannerNode>();

  // Ctrl+C path: brake before the context tears down, so the car does not coast
  // for the duration of the VESC's command timeout.
  rclcpp::on_shutdown([node]() { node->brakeOnShutdown(); });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

float ind2angle(int index)
{
  return (index * angleIncrement) + angleMin;
}
