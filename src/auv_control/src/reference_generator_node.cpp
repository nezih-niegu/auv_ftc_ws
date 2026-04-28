// ============================================================================
//  auv_control/reference_generator_node.cpp
//
//  Publishes the desired state x_ref(t) = [u v w q r] on /auv/reference_state.
//  Several preset patterns are selectable by parameter `pattern`:
//
//      "constant"   : track a fixed cruise speed u0 (default)
//      "yaw_sine"   : constant u0, sinusoidal yaw-rate reference
//      "heave_step" : step in w (depth rate) every step_period_s seconds
//      "mission"    : scripted sequence used to exercise all 4 actuators
//                     (mirrors the 300 s simulations in the paper: cruise
//                      then yaw manoeuvre, so a thruster fault injected at
//                      150 s is clearly visible in the response).
// ============================================================================
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ReferenceGenerator : public rclcpp::Node {
 public:
  ReferenceGenerator() : Node("reference_generator") {
    declare_parameter<std::string>("pattern",        "mission");
    declare_parameter<double>("rate_hz",             20.0);
    declare_parameter<double>("u0",                  0.8);   // cruise surge ref
    declare_parameter<double>("yaw_amplitude",       0.08);  // rad/s
    declare_parameter<double>("yaw_period_s",        60.0);
    declare_parameter<double>("heave_amp",           0.15);  // m/s
    declare_parameter<double>("step_period_s",       40.0);

    pattern_      = get_parameter("pattern").as_string();
    rate_hz_      = get_parameter("rate_hz").as_double();
    u0_           = get_parameter("u0").as_double();
    yaw_amp_      = get_parameter("yaw_amplitude").as_double();
    yaw_period_   = get_parameter("yaw_period_s").as_double();
    heave_amp_    = get_parameter("heave_amp").as_double();
    step_period_  = get_parameter("step_period_s").as_double();

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("reference_state", 10);
    t0_  = now().seconds();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_),
      std::bind(&ReferenceGenerator::tick, this));

    RCLCPP_INFO(get_logger(),
      "reference_generator: pattern=%s u0=%.2f", pattern_.c_str(), u0_);
  }

 private:
  void tick() {
    const double t = now().seconds() - t0_;

    double u_ref = u0_, v_ref = 0.0, w_ref = 0.0, q_ref = 0.0, r_ref = 0.0;

    if (pattern_ == "constant") {
      // nothing to do — all other refs are 0
    } else if (pattern_ == "yaw_sine") {
      r_ref = yaw_amp_ * std::sin(2.0 * M_PI * t / yaw_period_);
    } else if (pattern_ == "heave_step") {
      const int phase = static_cast<int>(std::floor(t / step_period_)) % 3;
      w_ref = (phase == 1) ? heave_amp_ : (phase == 2) ? -heave_amp_ : 0.0;
    } else if (pattern_ == "mission") {
      // 0..50 s   : accelerate/cruise at u0, no turning
      // 50..150 s : gentle yaw manoeuvre at +yaw_amp
      // 150..250 s: (fault window) reversed yaw manoeuvre
      // 250..300 s: return to straight cruise
      if (t < 50.0)      { r_ref = 0.0; }
      else if (t < 150.0){ r_ref =  yaw_amp_; }
      else if (t < 250.0){ r_ref = -yaw_amp_; }
      else               { r_ref = 0.0; }
    } else {
      RCLCPP_WARN_ONCE(get_logger(),
        "unknown pattern '%s' — falling back to constant", pattern_.c_str());
    }

    std_msgs::msg::Float64MultiArray m;
    m.data = {u_ref, v_ref, w_ref, q_ref, r_ref};
    pub_->publish(m);
  }

  std::string pattern_;
  double rate_hz_, u0_, yaw_amp_, yaw_period_, heave_amp_, step_period_;
  double t0_ = 0.0;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReferenceGenerator>());
  rclcpp::shutdown();
  return 0;
}
