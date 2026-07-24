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
#include "crowd_nav_safety_supervisor/intervention_cause.hpp"

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
using crowd_nav_safety_supervisor::InterventionCause;
using crowd_nav_safety_supervisor::SafetySupervisor;
using crowd_nav_safety_supervisor::SafetySupervisorConfig;
using crowd_nav_safety_supervisor::SupervisorResult;
using InterventionEvent = crowd_nav_safety_supervisor::msg::InterventionEvent;

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
  // This robot's real sensor geometry (IMPLEMENTATION_PLAN.md S4.8.1), not the SARL checkpoint's
  // training-side D435I spec (85.2 deg / 12m) - a deliberate divergence, not a hand-typed
  // literal with no justification; see S4.8.1 for why. Defaults reflect the ~180 deg (S1's
  // confirmed LiDAR rear-hemisphere masking) / 8m (S3's already-deliberate conservative
  // budget-sensor spec) figures already stated in the README's Known Limitations section.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".perception_fov_half_angle_rad", rclcpp::ParameterValue(M_PI_2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".perception_max_range_m", rclcpp::ParameterValue(8.0));
  // Off (0.0) by default, matching DegradationParams' own "oracle passthrough unless
  // configured" convention (S4.2) - Phase 10's noise sweep (S4.9.2) is what actually turns this
  // on. Found while wiring the sweep, not before: this parameter didn't exist until now, so
  // dropout_prob was silently unreachable from any launch file - the exact "enumerated but
  // unreachable" defect class the Phase 9 addendum's reachability audit was about, caught here
  // before the sweep ran, not after. degradation_seed reuses the scenario's own seed directly -
  // safe despite the RNG-substream-isolation requirement (S4.2), since pedestrian_sim_node.py's
  // RNG (Python) and this class's RNG (C++ std::mt19937_64) are already separate processes with
  // unrelated RNG implementations; substream derivation matters only for two consumers sharing
  // one C++ stream, which doesn't apply across a process boundary.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".perception_dropout_prob", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".perception_degradation_seed", rclcpp::ParameterValue(0));
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
  // Safety supervisor OOD thresholds (IMPLEMENTATION_PLAN.md S4.4/S4.8.5) - independently
  // toggleable/tunable, not literals at the call site. forward_sim_dt_s and
  // max_commanded_speed_mps deliberately have NO separate parameter here - they reuse
  // action_space_config_.time_step_s and max_linear_vel_mps_ respectively (S4.8.3/S4.4), one
  // source for each number rather than a second hand-typed default that could drift from it.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".supervisor_max_train_humans", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".supervisor_min_train_distance_m", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".supervisor_max_train_speed_mps", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".supervisor_forward_sim_steps", rclcpp::ParameterValue(4));
  // Phase 10 (S4.9): launch-time only, see the member's own comment - lets the evaluation
  // harness's `policy_raw` config run with zero supervisor involvement.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".supervisor_enabled", rclcpp::ParameterValue(true));

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
  const double perception_fov_half_angle_rad =
    node->get_parameter(plugin_name_ + ".perception_fov_half_angle_rad").as_double();
  const double perception_max_range_m =
    node->get_parameter(plugin_name_ + ".perception_max_range_m").as_double();
  const double perception_dropout_prob =
    node->get_parameter(plugin_name_ + ".perception_dropout_prob").as_double();
  const int perception_degradation_seed =
    node->get_parameter(plugin_name_ + ".perception_degradation_seed").as_int();
  watchdog_window_s_ = node->get_parameter(plugin_name_ + ".watchdog_window_s").as_double();
  debug_inject_decision_delay_s_ =
    node->get_parameter(plugin_name_ + ".debug_inject_decision_delay_s").as_double();
  max_linear_vel_mps_ = node->get_parameter(plugin_name_ + ".max_linear_vel_mps").as_double();
  min_linear_vel_mps_ = node->get_parameter(plugin_name_ + ".min_linear_vel_mps").as_double();
  max_angular_vel_rps_ = node->get_parameter(plugin_name_ + ".max_angular_vel_rps").as_double();
  const int supervisor_max_train_humans =
    node->get_parameter(plugin_name_ + ".supervisor_max_train_humans").as_int();
  const double supervisor_min_train_distance_m =
    node->get_parameter(plugin_name_ + ".supervisor_min_train_distance_m").as_double();
  const double supervisor_max_train_speed_mps =
    node->get_parameter(plugin_name_ + ".supervisor_max_train_speed_mps").as_double();
  const int supervisor_forward_sim_steps =
    node->get_parameter(plugin_name_ + ".supervisor_forward_sim_steps").as_int();
  supervisor_enabled_ = node->get_parameter(plugin_name_ + ".supervisor_enabled").as_bool();

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

  // Live perception (S3 Phase 8/S4.7): no noise/dropout/latency by default - this project's
  // perception-degradation model (Phase 5) is exercised deliberately by Phase 10's noise sweep,
  // not turned on silently here. FOV/range ARE set here, though (S4.8.1) - unlike noise/dropout,
  // "what this robot's sensor could ever see" isn't a sweep variable, it's this robot's actual,
  // fixed physical sensor geometry, so it defaults on rather than off. GroundTruthHumanSource's
  // production constructor is now templated (fixed Phase 8 - it previously required
  // rclcpp::Node, which rclcpp_lifecycle::LifecycleNode is not, see docs/phase7-findings.md).
  DegradationParams perception_params;
  perception_params.fov_half_angle_rad = perception_fov_half_angle_rad;
  perception_params.max_range_m = perception_max_range_m;
  perception_params.dropout_prob = perception_dropout_prob;
  perception_params.degradation_seed = static_cast<uint64_t>(perception_degradation_seed);
  human_source_ = std::make_unique<GroundTruthHumanSource>(
    node, pedestrian_topic, robot_pose_topic, perception_params);

  // Safety supervisor (S3 Phase 9, S4.8) - a plain class instantiated here, not a second ROS
  // node; forward_sim_dt_s reuses action_space_config_.time_step_s and max_commanded_speed_mps
  // reuses max_linear_vel_mps_ (both already resolved above), one source for each rather than a
  // second hand-typed default (S4.8.3/S4.4).
  SafetySupervisorConfig supervisor_config;
  supervisor_config.max_train_humans = static_cast<uint32_t>(supervisor_max_train_humans);
  supervisor_config.min_train_distance_m = supervisor_min_train_distance_m;
  supervisor_config.max_train_speed_mps = supervisor_max_train_speed_mps;
  supervisor_config.forward_sim_steps = supervisor_forward_sim_steps;
  supervisor_config.forward_sim_dt_s = action_space_config_.time_step_s;
  supervisor_config.max_commanded_speed_mps = max_linear_vel_mps_;
  supervisor_ = std::make_unique<SafetySupervisor>(supervisor_config);

  // Secondary, best-effort cause-labeling lookup (S4.8.3) - transient_local to match
  // nav2_map_server's own latched-map QoS convention (crowd_nav_zones' mask_server is a
  // map_server instance, docs/phase3-findings.md), so a late-starting subscriber still gets the
  // current mask rather than only future updates. A never-received mask just means every
  // rejection on a keep-out cell logs as the generic COSTMAP_COLLISION instead of
  // KEEPOUT_VIOLATION - it never changes the safety decision itself.
  keepout_mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/keepout_filter_mask", rclcpp::QoS(1).transient_local(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {onKeepoutMask(msg);});

  intervention_pub_ = node->create_publisher<InterventionEvent>("/intervention_events", 10);

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
  supervisor_.reset();
  keepout_mask_sub_.reset();
  intervention_pub_.reset();
}

void CrowdNavController::activate()
{
  fallback_controller_->activate();
  intervention_pub_->on_activate();
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&CrowdNavController::onSetParameters, this, std::placeholders::_1));
}

void CrowdNavController::deactivate()
{
  fallback_controller_->deactivate();
  intervention_pub_->on_deactivate();
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

void CrowdNavController::onKeepoutMask(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(keepout_mask_mutex_);
  latest_keepout_mask_ = msg;
}

void CrowdNavController::publishIntervention(
  InterventionCause cause, const Velocity2D & rejected, const Velocity2D & sent)
{
  InterventionEvent msg;
  msg.stamp = node_.lock()->get_clock()->now();
  msg.cause = crowd_nav_safety_supervisor::toMsgValue(cause);
  msg.rejected_vx = rejected.vx;
  msg.rejected_vy = rejected.vy;
  msg.sent_vx = sent.vx;
  msg.sent_vy = sent.vy;
  intervention_pub_->publish(msg);

  // Edge-triggered (S4.8.7), same idea as logged_source_is_fallback_ below - avoids spamming
  // the log every tick while a sustained rejection persists.
  if (!logged_intervention_active_) {
    RCLCPP_WARN(
      logger_, "CrowdNavController '%s': safety supervisor rejected command (cause=%s)",
      plugin_name_.c_str(), crowd_nav_safety_supervisor::toString(cause).c_str());
    logged_intervention_active_ = true;
  }
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

  // Safety supervisor check (IMPLEMENTATION_PLAN.md S1.7/S4.6/S4.8): runs inside this same
  // closure, after selectAction() produces a candidate, so its cost is covered by the watchdog
  // window (S1.7) rather than being a separate, unbounded step. Rejection returns a direct
  // controlled stop rather than delegating to fallback_controller_ - calling into the embedded
  // MPPI from this background thread would race against the main thread's own call to it on a
  // watchdog timeout (S4.8.4), a concurrency hazard S4.6 already went out of its way to avoid
  // for PolicyAdapter's mutable state.
  auto run_policy_decision = [this, pose, velocity, now]() -> Velocity2D {
      if (debug_inject_decision_delay_s_ > 0.0) {
        std::this_thread::sleep_for(
          std::chrono::duration<double>(debug_inject_decision_delay_s_));
      }
      const WorldState state = buildWorldState(pose, velocity, now);
      const auto inputs = adapter_->buildInputs(state);
      // Defensive guard, not a normally-hit path for either adapter (S4.8.1 revised SarlAdapter
      // to inject a dummy human rather than return an empty batch when perception has nothing to
      // report; DummyAdapter's padding always fills all max_humans slots). Kept so a batch
      // that's empty for some other reason (misconfiguration, a future adapter) still skips
      // inference rather than calling the network with a degenerate zero-length input.
      if (inputs.data.empty() || inputs.data[0].empty()) {
        return {0.0, 0.0};
      }
      const auto outputs = runInference(*ort_session_, inputs, adapter_->expectedShape().output_names);
      const Velocity2D candidate = adapter_->selectAction(outputs, state);

      // Phase 10 (S4.9): `policy_raw` vs `policy_supervised` in the evaluation matrix needs a
      // genuine "policy with no safety net at all" config, not just a differently-tuned one -
      // disabling the supervisor entirely (no check, no intervention, no log) is what makes
      // that comparison honest. Default true (supervised), so every existing config/test is
      // unaffected unless explicitly turned off.
      if (!supervisor_enabled_) {
        return candidate;
      }

      // OOD criteria first (S4.4/S4.8.5) - cheap, no costmap needed. `state` here is the
      // ORIGINAL WorldState, never one that's been through an adapter's own dummy-injection
      // (S4.8.1), so CROWD_SIZE/PROXIMITY can't be corrupted by a placeholder human.
      SupervisorResult supervisor_result =
        supervisor_->checkOodCriteria(state, candidate, human_source_->numDegradedLastCall());

      if (supervisor_result.safe) {
        nav_msgs::msg::OccupancyGrid::SharedPtr keepout_mask_snapshot;
        {
          std::lock_guard<std::mutex> lock(keepout_mask_mutex_);
          keepout_mask_snapshot = latest_keepout_mask_;
        }
        // costmap_ros_->getCostmap() fetched fresh this call, not cached at configure() time
        // (S4.8.2) - the SAME Costmap2D* the embedded MPPI itself reads from, since both this
        // controller and fallback_controller_ were configured with the identical costmap_ros_.
        supervisor_result = supervisor_->checkForwardSim(
          state, candidate, costmap_ros_->getCostmap(), keepout_mask_snapshot.get());
      }

      if (!supervisor_result.safe) {
        publishIntervention(*supervisor_result.cause, candidate, {0.0, 0.0});
        return {0.0, 0.0};
      }
      logged_intervention_active_ = false;
      return candidate;
    };

  const DecisionResult result = decision_core_->decide(now, run_policy_decision);

  const bool source_is_fallback = result.source == DecisionSource::kFallback;
  if (source_is_fallback != logged_source_is_fallback_) {
    RCLCPP_WARN(
      logger_, "CrowdNavController '%s': command source switched to %s",
      plugin_name_.c_str(), source_is_fallback ? "FALLBACK" : "policy");
    logged_source_is_fallback_ = source_is_fallback;
    if (source_is_fallback) {
      // INFERENCE_TIMEOUT (S4.8.5) - the one cause that fires on the main thread via the
      // existing watchdog, not inside run_policy_decision; no candidate existed to reject
      // (inference/supervisor didn't finish in time), so rejected/sent are both zeroed.
      publishIntervention(InterventionCause::kInferenceTimeout, {0.0, 0.0}, {0.0, 0.0});
    }
  }

  if (source_is_fallback) {
    return fallback_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  }
  return toTwistStamped(result.command, pose);
}

}  // namespace crowd_nav_controller

PLUGINLIB_EXPORT_CLASS(crowd_nav_controller::CrowdNavController, nav2_core::Controller)
