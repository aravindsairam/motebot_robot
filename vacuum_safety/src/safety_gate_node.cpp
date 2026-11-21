// The safety gate: the only node that commands the wheels.
//
//   in:   /cmd_vel            geometry_msgs/Twist          manual driving (keyboard, app joystick)
//         /cmd_vel_nav        geometry_msgs/Twist          Nav2
//         /robot_state        vacuum_interfaces/RobotState the robot's mode, from the mode manager
//         /cliff/<name>       sensor_msgs/Range            front_left, front_right, side_left, side_right
//         /bumper             std_msgs/UInt8               bit mask of pressed zones
//   out:  /cmd_vel_safe       geometry_msgs/Twist          the wheel command, at a steady rate
//         /safety_status      vacuum_interfaces/SafetyStatus  what the gate is doing and why
//
// This file only moves data between ROS and GateLogic, where every decision is made.
//
// All timing here uses the computer's steady clock, never simulation time: if the simulator
// freezes, its clock stops, and a timeout measured on that clock would never expire.

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "vacuum_interfaces/msg/robot_state.hpp"
#include "vacuum_interfaces/msg/safety_status.hpp"
#include "vacuum_safety/gate_logic.hpp"

namespace vacuum_safety
{

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::Range;
using std_msgs::msg::UInt8;
using vacuum_interfaces::msg::RobotState;
using vacuum_interfaces::msg::SafetyStatus;

class SafetyGate : public rclcpp::Node
{
public:
  SafetyGate()
  : Node("safety_gate"), started_(std::chrono::steady_clock::now())
  {
    Config config;
    const auto positive = [this](const std::string & name, double fallback) {
        const double value = declare_parameter(name, fallback);
        if (!(value > 0.0)) {
          throw std::invalid_argument("parameter " + name + " must be greater than zero");
        }
        return value;
      };
    config.command_timeout = positive("command_timeout", config.command_timeout);
    config.mode_timeout = positive("mode_timeout", config.mode_timeout);
    config.sensor_timeout = positive("sensor_timeout", config.sensor_timeout);
    config.cliff_threshold = positive("cliff_threshold", config.cliff_threshold);
    config.backoff_speed = positive("backoff_speed", config.backoff_speed);
    config.backoff_after_clear = positive("backoff_after_clear", config.backoff_after_clear);
    config.backoff_max = positive("backoff_max", config.backoff_max);
    config.forward_pause = positive("forward_pause", config.forward_pause);
    config.motion_window = positive("motion_window", config.motion_window);
    config.max_linear = positive("max_linear", config.max_linear);
    config.max_angular = positive("max_angular", config.max_angular);
    const double publish_rate = positive("publish_rate", 50.0);
    status_period_ = 1.0 / positive("status_rate", 10.0);
    logic_ = std::make_unique<GateLogic>(config);

    wheel_pub_ = create_publisher<Twist>("cmd_vel_safe", rclcpp::QoS(1));
    status_pub_ = create_publisher<SafetyStatus>("safety_status", rclcpp::QoS(10));

    // Commands: only the newest one matters.
    manual_sub_ = create_subscription<Twist>(
      "cmd_vel", rclcpp::QoS(1), [this](Twist::ConstSharedPtr msg) {
        in_.manual = {msg->linear.x, msg->angular.z};
        in_.manual_time = seconds();
      });
    nav_sub_ = create_subscription<Twist>(
      "cmd_vel_nav", rclcpp::QoS(1), [this](Twist::ConstSharedPtr msg) {
        in_.nav = {msg->linear.x, msg->angular.z};
        in_.nav_time = seconds();
      });

    // The mode is latched by the mode manager, so a restarted gate learns it at once.
    state_sub_ = create_subscription<RobotState>(
      "robot_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](RobotState::ConstSharedPtr msg) {
        const bool changed = msg->state != in_.mode || msg->transitioning != in_.transitioning;
        in_.mode = msg->state;
        in_.transitioning = msg->transitioning;
        in_.mode_time = seconds();
        if (changed) {
          tick();  // a mode change takes effect now, not at the next timer tick
        }
      });

    const std::array<const char *, kCliffSensors> cliff_names{
      {"front_left", "front_right", "side_left", "side_right"}};  // the order of CliffSensor
    for (std::size_t i = 0; i < kCliffSensors; ++i) {
      cliff_subs_[i] = create_subscription<Range>(
        std::string("cliff/") + cliff_names[i], rclcpp::SensorDataQoS().keep_last(1),
        [this, i](Range::ConstSharedPtr msg) {
          const bool first = in_.cliff_time[i] < 0.0;
          const bool was_cliff = isCliff(in_.cliff_range[i]);
          in_.cliff_range[i] = msg->range;
          in_.cliff_time[i] = seconds();
          if (first || was_cliff != isCliff(msg->range)) {
            tick();  // react to a hazard as soon as it is seen
          }
        });
    }
    bumper_sub_ = create_subscription<UInt8>(
      "bumper", rclcpp::QoS(10), [this](UInt8::ConstSharedPtr msg) {
        const bool changed = in_.bumper_time < 0.0 || msg->data != in_.bumper;
        in_.bumper = msg->data;
        in_.bumper_time = seconds();
        if (changed) {
          tick();
        }
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate), std::bind(&SafetyGate::tick, this));
    RCLCPP_INFO(
      get_logger(), "safety gate started: wheel commands on %s at %.0f Hz",
      wheel_pub_->get_topic_name(), publish_rate);
  }

  // One last stop command on the way out. The base also stops by itself when commands end.
  void stop()
  {
    wheel_pub_->publish(Twist());
  }

private:
  double seconds() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
  }

  bool isCliff(double range) const
  {
    return !std::isfinite(range) || range > logic_->config().cliff_threshold;
  }

  void tick()
  {
    in_.now = seconds();
    const Output out = logic_->update(in_);

    Twist command;
    command.linear.x = out.command.linear;
    command.angular.z = out.command.angular;
    wheel_pub_->publish(command);

    const bool changed = !have_last_ || out.source != last_.source ||
      out.blocked_reason != last_.blocked_reason || out.cliff != last_.cliff ||
      out.bumper != last_.bumper || out.off_floor != last_.off_floor ||
      out.reflex_active != last_.reflex_active || out.stale != last_.stale;
    if (changed || in_.now - last_status_ >= status_period_) {
      publishStatus(out);
      last_status_ = in_.now;
    }
    if (changed) {
      log(out);
    }
    last_ = out;
    have_last_ = true;
  }

  void publishStatus(const Output & out)
  {
    SafetyStatus msg;
    msg.stamp = now();
    msg.source = toString(out.source);
    msg.blocked_reason = out.blocked_reason;
    msg.cliff = out.cliff;
    msg.bumper = out.bumper;
    msg.off_floor = out.off_floor;
    msg.reflex_active = out.reflex_active;
    msg.stale = out.stale;
    status_pub_->publish(msg);
  }

  void log(const Output & out)
  {
    const bool hazard = out.reflex_active || out.off_floor || !out.stale.empty();
    const bool hazard_before = have_last_ &&
      (last_.reflex_active || last_.off_floor || !last_.stale.empty());
    if (hazard && (!hazard_before || out.blocked_reason != last_.blocked_reason)) {
      RCLCPP_WARN(get_logger(), "%s", describe(out).c_str());
    } else if (!have_last_ || out.source != last_.source ||
      out.blocked_reason != last_.blocked_reason)
    {
      RCLCPP_INFO(get_logger(), "%s", describe(out).c_str());
    }
  }

  static std::string describe(const Output & out)
  {
    std::string text = std::string("source: ") + toString(out.source);
    if (!out.blocked_reason.empty()) {
      text += "; " + out.blocked_reason;
    }
    for (const std::string & name : out.stale) {
      text += "; no " + name + " data";
    }
    return text;
  }

  std::chrono::steady_clock::time_point started_;
  std::unique_ptr<GateLogic> logic_;
  Inputs in_;
  Output last_;
  bool have_last_{false};
  double last_status_{0.0};
  double status_period_{0.1};

  rclcpp::Publisher<Twist>::SharedPtr wheel_pub_;
  rclcpp::Publisher<SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<RobotState>::SharedPtr state_sub_;
  std::array<rclcpp::Subscription<Range>::SharedPtr, kCliffSensors> cliff_subs_;
  rclcpp::Subscription<UInt8>::SharedPtr bumper_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vacuum_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vacuum_safety::SafetyGate>();
  std::weak_ptr<vacuum_safety::SafetyGate> weak = node;
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [weak]() {
      if (auto gate = weak.lock()) {
        gate->stop();
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
