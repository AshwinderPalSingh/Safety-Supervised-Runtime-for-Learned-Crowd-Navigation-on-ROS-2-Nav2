#ifndef CROWD_NAV_CONTROLLER__CROWD_NAV_CONTROLLER_HPP_
#define CROWD_NAV_CONTROLLER__CROWD_NAV_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "onnxruntime_cxx_api.h"
#include "pluginlib/class_loader.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"

#include "crowd_nav_controller/controller_decision_core.hpp"
#include "crowd_nav_observation/world_state.hpp"
#include "crowd_nav_policy_adapters/candidate_action_space.hpp"
#include "crowd_nav_policy_adapters/dummy_adapter.hpp"

namespace crowd_nav_controller
{

// IMPLEMENTATION_PLAN.md S3 Phase 7 / S4.6. Registered as the `FollowPath` plugin
// (`crowd_nav_controller::CrowdNavController`). Owns exactly: (1) building a WorldState and
// running it through a PolicyAdapter each decision tick, via ControllerDecisionCore's
// watchdog/hold-last-action logic, and (2) converting the adapter's holonomic Velocity2D output
// into a diff-drive Twist. Everything nav2_velocity_smoother already does (acceleration-limited
// smoothing) is deliberately NOT reimplemented here (S4.6). Failover is delegation to a
// genuinely embedded second nav2_core::Controller instance (nav2_mppi_controller::MPPIController
// by config), loaded via pluginlib the same way nav2_rotation_shim_controller loads its wrapped
// controller - verified against that package's actual source before writing this class.
class CrowdNavController : public nav2_core::Controller
{
public:
  CrowdNavController();
  ~CrowdNavController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  crowd_nav_observation::WorldState buildWorldState(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity) const;
  geometry_msgs::msg::TwistStamped toTwistStamped(
    const crowd_nav_policy_adapters::Velocity2D & command,
    const geometry_msgs::msg::PoseStamped & pose) const;
  // Lets `debug_inject_decision_delay_s` (and the other numeric params below) actually be
  // changed live via `ros2 param set` - without this, configure() reads each parameter exactly
  // once and the corresponding member never sees a later change, silently making the debug
  // knob (S4.6) inert at runtime. Found the hard way (docs/phase7-findings.md): a live stall
  // injection had no effect until this callback was added. Pattern verified against
  // nav2_rotation_shim_controller's own dynamicParametersCallback.
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("CrowdNavController")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path current_plan_;

  crowd_nav_policy_adapters::CandidateActionSpaceConfig action_space_config_;
  std::unique_ptr<crowd_nav_policy_adapters::DummyAdapter> adapter_;
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  std::unique_ptr<ControllerDecisionCore> decision_core_;

  pluginlib::ClassLoader<nav2_core::Controller> fallback_loader_;
  nav2_core::Controller::Ptr fallback_controller_;

  // Edge-triggered (logs only on change, not per-tick) so a source switch is visible in the
  // log without spamming it every 20 Hz tick - useful telemetry beyond testing too, and what
  // makes the failover-transition verification (IMPLEMENTATION_PLAN.md S4.6) observable
  // without needing a dedicated debug topic.
  bool logged_source_is_fallback_ = false;

  double watchdog_window_s_ = 0.03;
  double debug_inject_decision_delay_s_ = 0.0;
  double max_linear_vel_mps_ = 1.0;
  double max_angular_vel_rps_ = 2.0;
  double min_linear_vel_mps_ = -0.3;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace crowd_nav_controller

#endif  // CROWD_NAV_CONTROLLER__CROWD_NAV_CONTROLLER_HPP_
