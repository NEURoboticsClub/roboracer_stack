// Constant-radius circle test: measures the car's REAL minimum turning radius.
//
// Holds one fixed steering position and one fixed speed so the car traces a
// steady circle. What that circle measures tells you which limit you are up
// against:
//
//   measured radius ~= wheelbase / tan(steer angle)  -> geometry limited, the
//                                                       steering is maxed out
//   measured radius noticeably larger                -> understeer: front grip
//                                                       or too much speed
//
// Run it exactly like the planner - the e-stop console owns the car:
//
//     ros2 run planner_node estop        (terminal 1, press r to release)
//     ros2 run planner_node circle_test  (terminal 2)
//
// SPACE in the e-stop console stops the car, same as always.
//
// Do NOT run this alongside planner_gap_follow: both publish to
// /commands/servo/position and /commands/motor/speed, and they will fight over
// the car. This node warns if it sees another publisher on the servo topic.
//
// Defaults command full LEFT lock. Run it again with
//   --ros-args -p servo_position:=0.895
// for the other side; the two are rarely identical.

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

class CircleTestNode : public rclcpp::Node
{
public:
  CircleTestNode() : Node("circle_test_node")
  {
    // Full lock on one side. Defaults to servo_min from the measured steering
    // range; pass servo_position:=0.895 to circle the other way.
    servo_position_ = declare_parameter<double>("servo_position", 0.117);
    servo_center_ = declare_parameter<double>("servo_center", 0.5);
    // Lowest speed the VESC's closed-loop PID will actually hold on this car.
    // Below ~5000 eRPM the motor silently refuses to start.
    speed_erpm_ = declare_parameter<double>("speed_erpm", 5000.0);
    brake_current_ = declare_parameter<double>("brake_current", 4.0);
    estop_timeout_sec_ = declare_parameter<double>("estop_timeout_sec", 0.3);
    // Hold full lock, stopped, before spinning up. Without this the car pulls
    // away while the servo is still swinging and the first quarter turn is a
    // spiral rather than part of the circle you want to measure.
    settle_sec_ = declare_parameter<double>("settle_sec", 1.0);
    // 0 => run until the e-stop console stops it.
    duration_sec_ = declare_parameter<double>("duration_sec", 0.0);

    servo_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 1);
    speed_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 1);
    brake_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/motor/brake", 1);

    // Latched, matching the e-stop console's publisher, so starting this node
    // second still picks up an already-engaged stop instead of driving off.
    rclcpp::QoS estop_qos(1);
    estop_qos.transient_local().reliable();
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/estop", estop_qos,
      [this](std_msgs::msg::Bool::UniquePtr msg)
      {
        estop_engaged_ = msg->data;
        last_estop_msg_ = std::chrono::steady_clock::now();
        estop_seen_ = true;
      });

    // 50 Hz: well inside the VESC's command timeout, and fast enough that the
    // console's 100 Hz brake still wins whenever the stop is engaged.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&CircleTestNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "CIRCLE TEST: servo %.3f (%.3f from center %.3f), %.0f eRPM. "
      "STOPPED BY DEFAULT - run 'ros2 run planner_node estop' and press r.",
      servo_position_, servo_position_ - servo_center_, servo_center_, speed_erpm_);
  }

  void brakeOnShutdown()
  {
    if (!brake_pub_)
      return;

    std_msgs::msg::Float64 brake_msg;
    brake_msg.data = brake_current_;
    for (int i = 0; i < 5; i++)
      brake_pub_->publish(brake_msg);
  }

private:
  // Same two-part rule the planner uses: stopped unless the console is alive
  // AND has released it, so a closed or killed console stops the car instead of
  // leaving it running on a stale value.
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
          ? "E-STOP heartbeat lost (%.2fs) - braking."
          : "Waiting for e-stop console - braking. Run 'ros2 run planner_node estop', press r.",
        age_sec);
      return true;
    }

    return false;
  }

  void tick()
  {
    // The planner publishing at the same time would make the measured circle
    // meaningless, so say so loudly rather than producing a bad number.
    if (count_publishers("/commands/servo/position") > 1)
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 3000,
        "Another node is publishing servo commands - is planner_gap_follow running? "
        "Stop it: the circle test needs the car to itself.");

    std_msgs::msg::Float64 msg;

    if (stopRequested())
    {
      running_ = false;
      msg.data = servo_center_;
      servo_pub_->publish(msg);
      msg.data = brake_current_;
      brake_pub_->publish(msg);
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!running_)
    {
      running_ = true;
      release_time_ = now;
      RCLCPP_INFO(get_logger(), "Released - holding lock for %.1fs to settle.", settle_sec_);
    }

    // Steering is held for the whole run, settle phase included.
    msg.data = servo_position_;
    servo_pub_->publish(msg);

    const double since_release = std::chrono::duration<double>(now - release_time_).count();
    if (since_release < settle_sec_)
    {
      // Brake during the settle so the car cannot creep while the servo swings.
      msg.data = brake_current_;
      brake_pub_->publish(msg);
      return;
    }

    const double driving_sec = since_release - settle_sec_;
    if (duration_sec_ > 0.0 && driving_sec >= duration_sec_)
    {
      msg.data = brake_current_;
      brake_pub_->publish(msg);
      RCLCPP_INFO_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "duration_sec %.1f reached - braking. Press SPACE in the e-stop console.",
        duration_sec_);
      return;
    }

    msg.data = speed_erpm_;
    speed_pub_->publish(msg);

    // Read the revolution time straight off this counter: note the elapsed
    // value as the car passes the same point on two successive laps.
    RCLCPP_INFO_THROTTLE(
      get_logger(), steady_clock_, 500, "circling  t = %6.2f s", driving_sec);
  }

  double servo_position_ = 0.117;
  double servo_center_ = 0.5;
  double speed_erpm_ = 5000.0;
  double brake_current_ = 4.0;
  double estop_timeout_sec_ = 0.3;
  double settle_sec_ = 1.0;
  double duration_sec_ = 0.0;

  bool running_ = false;
  std::chrono::steady_clock::time_point release_time_{};

  // Stopped by default: nothing moves until the e-stop console releases it.
  bool estop_engaged_ = true;
  bool estop_seen_ = false;
  std::chrono::steady_clock::time_point last_estop_msg_{};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CircleTestNode>();

  rclcpp::on_shutdown([node]() { node->brakeOnShutdown(); });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
