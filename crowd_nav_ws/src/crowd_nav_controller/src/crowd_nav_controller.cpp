#include "crowd_nav_controller/crowd_nav_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

#include "crowd_nav_perception/degradation_params.hpp"
#include "crowd_nav_perception/ground_truth_human_source.hpp"
#include "crowd_nav_policy_adapters/dummy_adapter.hpp"
#include "crowd_nav_policy_adapters/onnx_inference.hpp"
#include "crowd_nav_policy_adapters/sarl_adapter.hpp"
#include "crowd_nav_policy_adapters/shape_validation.hpp"

namespace crowd_nav_controller
{

using crowd_nav_observation::WorldState;
using crowd_nav_perception::DegradationParams;
using crowd_nav_perception::GroundTruthHumanSource;
using crowd_nav_policy_adapters::CandidateActionSpaceConfig;
using crowd_nav_policy_adapters::DummyAdapter;
using crowd_nav_policy_adapters::loadCandidateActionSpaceConfig;
using crowd_nav_policy_adapters::runInference;
using crowd_nav_policy_adapters::SarlAdapter;
using crowd_nav_policy_adapters::validateSessionShapes;
using crowd_nav_policy_adapters::Velocity2D;

CrowdNavController::CrowdNavController()
: fallback_loader_("nav2_core", "nav2_core::Controller")
{
}

void CrowdNavController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  auto node = parent.lock();
  logger_ = node->get_logger();
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".fallback_controller_plugin", rclcpp::PARAMETER_STRING);
  // "dummy" (Phase 6/7) or "sarl" (Phase 8) - the actual exercise of the adapter-swap promise
  // the whole PolicyAdapter interface exists for (S3 Phase 8). Defaults to "dummy" so existing
  // Phase 7 configs keep working unchanged unless explicitly switched.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".adapter_type", rclcpp::ParameterValue(std::string("dummy")));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".policy_adapter_config_path", rclcpp::ParameterValue(std::string()));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".pedestrian_topic", rclcpp::ParameterValue(std::string("/pedestrians")));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_pose_topic",
    rclcpp::ParameterValue(std::string("/ground_truth/robot_pose")));
  // Empty default (not PARAMETER_STRING/required): nav2_params.yaml is loaded as a raw YAML
  // params file with no launch-substitution preprocessing (confirmed against how this
  // project's own launch files pass it to Node(parameters=[...]) - "$(find-pkg-share ...)"
  // syntax only resolves inside a .launch.py, not inside the YAML's own string values), so a
  // portable default has to be resolved here instead, the same way policy_adapter_config_path
  // above already is.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".onnx_model_path", rclcpp::ParameterValue(std::string()));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".watchdog_window_s", rclcpp::ParameterValue(0.03));
  // Test-only diagnostic knob, not a production parameter (IMPLEMENTATION_PLAN.md S4.6): when
  // > 0, every fresh policy decision sleeps this long before running, so the watchdog/failover
  // path can be exercised live against the real Nav2/Gazebo stack. Left at 0.0 does nothing.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".debug_inject_decision_delay_s", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel_mps", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_vel_mps", rclcpp::ParameterValue(-0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel_rps", rclcpp::ParameterValue(2.0));

  const std::string fallback_plugin =
    node->get_parameter(plugin_name_ + ".fallback_controller_plugin").as_string();
  const std::string adapter_type = node->get_parameter(plugin_name_ + ".adapter_type").as_string();
  std::string config_path =
    node->get_parameter(plugin_name_ + ".policy_adapter_config_path").as_string();
  if (config_path.empty()) {
    config_path = ament_index_cpp::get_package_share_directory("crowd_nav_policy_adapters") +
      "/config/policy_adapter.yaml";
  }
  std::string model_path = node->get_parameter(plugin_name_ + ".onnx_model_path").as_string();
  if (model_path.empty()) {
    const std::string default_model = (adapter_type == "sarl") ?
      "/models/sarl_value_net.onnx" : "/models/dummy_policy.onnx";
    model_path = ament_index_cpp::get_package_share_directory("crowd_nav_policy_adapters") +
      default_model;
  }
  const std::string pedestrian_topic =
    node->get_parameter(plugin_name_ + ".pedestrian_topic").as_string();
  const std::string robot_pose_topic =
    node->get_parameter(plugin_name_ + ".robot_pose_topic").as_string();
  watchdog_window_s_ = node->get_parameter(plugin_name_ + ".watchdog_window_s").as_double();
  debug_inject_decision_delay_s_ =
    node->get_parameter(plugin_name_ + ".debug_inject_decision_delay_s").as_double();
  max_linear_vel_mps_ = node->get_parameter(plugin_name_ + ".max_linear_vel_mps").as_double();
  min_linear_vel_mps_ = node->get_parameter(plugin_name_ + ".min_linear_vel_mps").as_double();
  max_angular_vel_rps_ = node->get_parameter(plugin_name_ + ".max_angular_vel_rps").as_double();

  action_space_config_ = loadCandidateActionSpaceConfig(config_path);
  if (adapter_type == "sarl") {
    adapter_ = std::make_unique<SarlAdapter>(action_space_config_);
  } else if (adapter_type == "dummy") {
    adapter_ = std::make_unique<DummyAdapter>(action_space_config_);
  } else {
    RCLCPP_FATAL(
      logger_, "CrowdNavController '%s': unknown adapter_type '%s' (expected 'dummy' or 'sarl')",
      plugin_name_.c_str(), adapter_type.c_str());
    throw std::runtime_error("CrowdNavController: unknown adapter_type " + adapter_type);
  }

  ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, (plugin_name_ + "_onnx").c_str());
  Ort::SessionOptions session_options;
  ort_session_ = std::make_unique<Ort::Session>(*ort_env_, model_path.c_str(), session_options);
  validateSessionShapes(*ort_session_, adapter_->expectedShape());

  decision_core_ = std::make_unique<ControllerDecisionCore>(
    action_space_config_.time_step_s, watchdog_window_s_);

  // Live perception (S3 Phase 8/S4.7): default DegradationParams is oracle passthrough (no
  // noise/dropout/latency) - this phase's focus is the search reimplementation, not perception
  // degradation, which Phase 5 already tested independently. GroundTruthHumanSource's
  // production constructor is now templated (fixed this phase - it previously required
  // rclcpp::Node, which rclcpp_lifecycle::LifecycleNode is not, see docs/phase7-findings.md).
  DegradationParams perception_params;
  human_source_ = std::make_unique<GroundTruthHumanSource>(
    node, pedestrian_topic, robot_pose_topic, perception_params);

  try {
    fallback_controller_ = fallback_loader_.createUniqueInstance(fallback_plugin);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_, "Failed to create fallback controller '%s': %s", fallback_plugin.c_str(),
      ex.what());
    throw;
  }
  RCLCPP_INFO(
    logger_, "CrowdNavController '%s': fallback controller '%s' created",
    plugin_name_.c_str(), fallback_plugin.c_str());
  fallback_controller_->configure(parent, name, tf, costmap_ros);
}

void CrowdNavController::cleanup()
{
  fallback_controller_->cleanup();
  fallback_controller_.reset();
  ort_session_.reset();
  ort_env_.reset();
  adapter_.reset();
  decision_core_.reset();
  human_source_.reset();
}

void CrowdNavController::activate()
{
  fallback_controller_->activate();
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&CrowdNavController::onSetParameters, this, std::placeholders::_1));
}

void CrowdNavController::deactivate()
{
  fallback_controller_->deactivate();
  dyn_params_handler_.reset();
}

rcl_interfaces::msg::SetParametersResult CrowdNavController::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      continue;
    }
    const std::string & name = parameter.get_name();
    if (name == plugin_name_ + ".debug_inject_decision_delay_s") {
      debug_inject_decision_delay_s_ = parameter.as_double();
    } else if (name == plugin_name_ + ".watchdog_window_s") {
      watchdog_window_s_ = parameter.as_double();
      decision_core_->setWatchdogWindow(watchdog_window_s_);
    } else if (name == plugin_name_ + ".max_linear_vel_mps") {
      max_linear_vel_mps_ = parameter.as_double();
    } else if (name == plugin_name_ + ".min_linear_vel_mps") {
      min_linear_vel_mps_ = parameter.as_double();
    } else if (name == plugin_name_ + ".max_angular_vel_rps") {
      max_angular_vel_rps_ = parameter.as_double();
    }
  }
  return result;
}

void CrowdNavController::setPlan(const nav_msgs::msg::Path & path)
{
  current_plan_ = path;
  fallback_controller_->setPlan(path);
}

void CrowdNavController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  fallback_controller_->setSpeedLimit(speed_limit, percentage);
}

WorldState CrowdNavController::buildWorldState(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  const rclcpp::Time & query_time) const
{
  WorldState state;
  const double theta = tf2::getYaw(pose.pose.orientation);

  state.robot.px = pose.pose.position.x;
  state.robot.py = pose.pose.position.y;
  // Body-frame (linear.x, angular.z) -> world-frame (vx, vy) via current heading. The robot
  // can't actually move sideways (diff-drive), so this drops the holonomic vy term entirely -
  // an honest simplification, not a hidden approximation.
  state.robot.vx = velocity.linear.x * std::cos(theta);
  state.robot.vy = velocity.linear.x * std::sin(theta);
  state.robot.radius = action_space_config_.policy_radius_m;
  state.robot.v_pref = action_space_config_.policy_v_pref_mps;
  state.robot.theta = theta;

  if (!current_plan_.poses.empty()) {
    const auto & goal = current_plan_.poses.back().pose.position;
    state.robot.gx = goal.x;
    state.robot.gy = goal.y;
  } else {
    state.robot.gx = state.robot.px;
    state.robot.gy = state.robot.py;
  }

  state.humans = human_source_->getHumans(query_time);
  return state;
}

geometry_msgs::msg::TwistStamped CrowdNavController::toTwistStamped(
  const Velocity2D & command, const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::TwistStamped out;
  out.header.frame_id = pose.header.frame_id;
  out.header.stamp = pose.header.stamp;

  const double speed = std::hypot(command.vx, command.vy);
  if (speed < 1e-6) {
    return out;
  }
  const double current_theta = tf2::getYaw(pose.pose.orientation);
  const double desired_heading = std::atan2(command.vy, command.vx);
  const double heading_error = angles::shortest_angular_distance(current_theta, desired_heading);

  // Simple holonomic -> diff-drive projection: forward speed shrinks (and can reverse) with
  // heading error, angular rate is a plain proportional term - adequate for DummyAdapter's
  // heading-heuristic candidates; SARL's own conversion (S1.9) may need retuning in Phase 8 if
  // this proves too crude for its chosen actions specifically.
  double linear = speed * std::cos(heading_error);
  double angular = std::clamp(2.0 * heading_error, -max_angular_vel_rps_, max_angular_vel_rps_);

  linear = std::clamp(linear, min_linear_vel_mps_, max_linear_vel_mps_);

  out.twist.linear.x = linear;
  out.twist.angular.z = angular;
  return out;
}

geometry_msgs::msg::TwistStamped CrowdNavController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (current_plan_.poses.empty()) {
    return fallback_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  }

  const rclcpp::Time now = node_.lock()->get_clock()->now();

  // Supervisor-check placeholder (IMPLEMENTATION_PLAN.md S1.7/S4.6): Phase 9's forward-sim/
  // costmap check will be called here, inside this same closure, so its cost is covered by the
  // watchdog window from day one rather than requiring this boundary to be reworked later. It
  // is a no-op today because the supervisor doesn't exist yet.
  auto run_policy_decision = [this, pose, velocity, now]() -> Velocity2D {
      if (debug_inject_decision_delay_s_ > 0.0) {
        std::this_thread::sleep_for(
          std::chrono::duration<double>(debug_inject_decision_delay_s_));
      }
      const WorldState state = buildWorldState(pose, velocity, now);
      const auto inputs = adapter_->buildInputs(state);
      // Empty batch (S4.7's zero-humans stopgap: SarlAdapter returns no rows when there's
      // nothing to reason about) - skip inference entirely rather than call the network with
      // a degenerate zero-length input. Adapter-agnostic: DummyAdapter never produces an empty
      // batch (its padding always fills all max_humans slots), so this never triggers for it.
      if (inputs.data.empty() || inputs.data[0].empty()) {
        return {0.0, 0.0};
      }
      const auto outputs = runInference(*ort_session_, inputs, adapter_->expectedShape().output_names);
      // Supervisor check placeholder call site - no-op until Phase 9.
      return adapter_->selectAction(outputs, state);
    };

  const DecisionResult result = decision_core_->decide(now, run_policy_decision);

  const bool source_is_fallback = result.source == DecisionSource::kFallback;
  if (source_is_fallback != logged_source_is_fallback_) {
    RCLCPP_WARN(
      logger_, "CrowdNavController '%s': command source switched to %s",
      plugin_name_.c_str(), source_is_fallback ? "FALLBACK" : "policy");
    logged_source_is_fallback_ = source_is_fallback;
  }

  if (source_is_fallback) {
    return fallback_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  }
  return toTwistStamped(result.command, pose);
}

}  // namespace crowd_nav_controller

PLUGINLIB_EXPORT_CLASS(crowd_nav_controller::CrowdNavController, nav2_core::Controller)
