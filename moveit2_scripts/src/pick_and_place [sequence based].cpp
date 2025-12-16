#include "moveit2_scripts/pick_and_place.hpp"

// Sequence based approach

// Initialize static members
const rclcpp::Logger PickAndPlaceTrajectory::LOGGER =
    rclcpp::get_logger("move_group_node");
const std::string PickAndPlaceTrajectory::PLANNING_GROUP_ROBOT =
    "ur_manipulator";
const std::string PickAndPlaceTrajectory::PLANNING_GROUP_GRIPPER = "gripper";

// ==============================================================================
// CONSTRUCTOR
// ==============================================================================

PickAndPlaceTrajectory::PickAndPlaceTrajectory(
    rclcpp::Node::SharedPtr base_node_)
    : base_node_(base_node_), plan_success_robot_(false),
      plan_success_gripper_(false), plan_fraction_robot_(0.0) {

  RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Trajectory...");

  // Configure node options
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // Initialize move_group node
  move_group_node_ = rclcpp::Node::make_shared("move_group_node", node_options);

  // Start move_group node in a new executor thread and spin it
  executor_.add_node(move_group_node_);
  std::thread([this]() { this->executor_.spin(); }).detach();

  // Initialize move_group interfaces
  move_group_robot_ = std::make_shared<MoveGroupInterface>(
      move_group_node_, PLANNING_GROUP_ROBOT);
  move_group_gripper_ = std::make_shared<MoveGroupInterface>(
      move_group_node_, PLANNING_GROUP_GRIPPER);

  // Get initial state of robot and gripper
  joint_model_group_robot_ =
      move_group_robot_->getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_ROBOT);
  joint_model_group_gripper_ =
      move_group_gripper_->getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Print out basic system information
  RCLCPP_INFO(LOGGER, "Planning Frame: %s",
              move_group_robot_->getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End Effector Link: %s",
              move_group_robot_->getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::vector<std::string> group_names =
      move_group_robot_->getJointModelGroupNames();
  for (size_t i = 0; i < group_names.size(); i++) {
    RCLCPP_INFO(LOGGER, "Group %zu: %s", i, group_names[i].c_str());
  }

  // Get current state of robot and gripper
  current_state_robot_ = move_group_robot_->getCurrentState(10);
  current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                joint_group_positions_robot_);
  current_state_gripper_ = move_group_gripper_->getCurrentState(10);
  current_state_gripper_->copyJointGroupPositions(
      joint_model_group_gripper_, joint_group_positions_gripper_);

  // Set start state of robot and gripper to current state
  move_group_robot_->setStartStateToCurrentState();
  move_group_gripper_->setStartStateToCurrentState();

  // Load parameters from YAML configuration file
  RCLCPP_INFO(LOGGER, "Loading parameters from configuration file...");
  load_parameters();

  // Print loaded configuration
  print_configuration();

  RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Trajectory");
}

// ==============================================================================
// DESTRUCTOR
// ==============================================================================

PickAndPlaceTrajectory::~PickAndPlaceTrajectory() {
  RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Trajectory");
}

// ==============================================================================
// PARAMETER LOADING METHODS
// ==============================================================================

void PickAndPlaceTrajectory::load_parameters() {
  load_timing_parameters();
  load_planning_parameters();
  load_gripper_parameters();
  load_joint_poses();
  load_cartesian_poses();
  load_cartesian_offsets();
  load_sequence();
}

void PickAndPlaceTrajectory::load_timing_parameters() {
  config_.sleep_duration_ms = get_or_declare_parameter<int>(
      "timing.sleep_duration_ms", config_.sleep_duration_ms);

  RCLCPP_INFO(LOGGER, "Loaded timing parameters:");
  RCLCPP_INFO(LOGGER, "  - sleep_duration_ms: %d", config_.sleep_duration_ms);
}

void PickAndPlaceTrajectory::load_planning_parameters() {
  config_.cartesian_end_effector_step =
      get_or_declare_parameter<double>("planning.cartesian.end_effector_step",
                                       config_.cartesian_end_effector_step);
  config_.cartesian_jump_threshold = get_or_declare_parameter<double>(
      "planning.cartesian.jump_threshold", config_.cartesian_jump_threshold);
  config_.kinematics_planning_time = get_or_declare_parameter<double>(
      "planning.kinematics.planning_time", config_.kinematics_planning_time);
  config_.kinematics_num_planning_attempts =
      get_or_declare_parameter<int>("planning.kinematics.num_planning_attempts",
                                    config_.kinematics_num_planning_attempts);

  RCLCPP_INFO(LOGGER, "Loaded planning parameters:");
  RCLCPP_INFO(LOGGER, "  - cartesian_end_effector_step: %.3f",
              config_.cartesian_end_effector_step);
  RCLCPP_INFO(LOGGER, "  - cartesian_jump_threshold: %.3f",
              config_.cartesian_jump_threshold);
  RCLCPP_INFO(LOGGER, "  - kinematics_planning_time: %.1f",
              config_.kinematics_planning_time);
  RCLCPP_INFO(LOGGER, "  - kinematics_num_planning_attempts: %d",
              config_.kinematics_num_planning_attempts);
}

void PickAndPlaceTrajectory::load_gripper_parameters() {
  config_.gripper_open_value = get_or_declare_parameter<double>(
      "gripper.open_value", config_.gripper_open_value);
  config_.gripper_close_value = get_or_declare_parameter<double>(
      "gripper.close_value", config_.gripper_close_value);
  config_.gripper_open_pose_name = get_or_declare_parameter<std::string>(
      "gripper.open_pose_name", config_.gripper_open_pose_name);
  config_.gripper_close_pose_name = get_or_declare_parameter<std::string>(
      "gripper.close_pose_name", config_.gripper_close_pose_name);

  RCLCPP_INFO(LOGGER, "Loaded gripper parameters:");
  RCLCPP_INFO(LOGGER, "  - open_value: %.3f", config_.gripper_open_value);
  RCLCPP_INFO(LOGGER, "  - close_value: %.3f", config_.gripper_close_value);
  RCLCPP_INFO(LOGGER, "  - open_pose_name: %s",
              config_.gripper_open_pose_name.c_str());
  RCLCPP_INFO(LOGGER, "  - close_pose_name: %s",
              config_.gripper_close_pose_name.c_str());
}

void PickAndPlaceTrajectory::load_joint_poses() {
  RCLCPP_INFO(LOGGER, "Loading joint space poses...");

  // Load home pose
  if (move_group_node_->has_parameter("poses.joint_space.home.values")) {
    JointPose home_pose;
    home_pose.key = "home"; // Set the YAML key
    home_pose.name = get_or_declare_parameter<std::string>(
        "poses.joint_space.home.name", "Home Position");
    home_pose.joint_values = get_or_declare_parameter<std::vector<double>>(
        "poses.joint_space.home.values", std::vector<double>(6, 0.0));

    config_.joint_poses.push_back(home_pose);

    RCLCPP_INFO(LOGGER, "  - Loaded joint pose: [%s] %s", home_pose.key.c_str(),
                home_pose.name.c_str());
    RCLCPP_INFO(LOGGER, "    Values: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                home_pose.joint_values[0], home_pose.joint_values[1],
                home_pose.joint_values[2], home_pose.joint_values[3],
                home_pose.joint_values[4], home_pose.joint_values[5]);
  }

  // You can add more joint poses here in the future
}

void PickAndPlaceTrajectory::load_cartesian_poses() {
  RCLCPP_INFO(LOGGER, "Loading cartesian space poses...");

  // Load pregrasp pose
  if (move_group_node_->has_parameter(
          "poses.cartesian_space.pregrasp.position.x")) {
    CartesianPose pregrasp_pose;
    pregrasp_pose.key = "pregrasp"; // Set the YAML key
    pregrasp_pose.name = get_or_declare_parameter<std::string>(
        "poses.cartesian_space.pregrasp.name", "Pregrasp Position");

    pregrasp_pose.position.x = get_or_declare_parameter<double>(
        "poses.cartesian_space.pregrasp.position.x", 0.0);
    pregrasp_pose.position.y = get_or_declare_parameter<double>(
        "poses.cartesian_space.pregrasp.position.y", 0.0);
    pregrasp_pose.position.z = get_or_declare_parameter<double>(
        "poses.cartesian_space.pregrasp.position.z", 0.0);

    pregrasp_pose.orientation.x = get_or_declare_parameter<double>(
        "poses.cartesian_space.pregrasp.orientation.x", 0.0);
    pregrasp_pose.orientation.y = get_or_declare_parameter<double>(
        "poses.cartesian_space.pregrasp.orientation.y", 0.0);
    pregrasp_pose.orientation.z = get_or_declare_parameter<double>(
        "poses.cartesian_space.pregrasp.orientation.z", 0.0);
    pregrasp_pose.orientation.w = get_or_declare_parameter<double>(
        "poses.cartesian_space.pregrasp.orientation.w", 1.0);

    config_.cartesian_poses.push_back(pregrasp_pose);

    RCLCPP_INFO(LOGGER, "  - Loaded cartesian pose: [%s] %s",
                pregrasp_pose.key.c_str(), pregrasp_pose.name.c_str());
    RCLCPP_INFO(LOGGER, "    Position: [%.3f, %.3f, %.3f]",
                pregrasp_pose.position.x, pregrasp_pose.position.y,
                pregrasp_pose.position.z);
    RCLCPP_INFO(LOGGER, "    Orientation: [%.3f, %.3f, %.3f, %.3f]",
                pregrasp_pose.orientation.x, pregrasp_pose.orientation.y,
                pregrasp_pose.orientation.z, pregrasp_pose.orientation.w);
  }

  // Load grasp pose
  if (move_group_node_->has_parameter(
          "poses.cartesian_space.grasp.position.x")) {
    CartesianPose grasp_pose;
    grasp_pose.key = "grasp"; // Set the YAML key
    grasp_pose.name = get_or_declare_parameter<std::string>(
        "poses.cartesian_space.grasp.name", "Grasp Position");

    grasp_pose.position.x = get_or_declare_parameter<double>(
        "poses.cartesian_space.grasp.position.x", 0.0);
    grasp_pose.position.y = get_or_declare_parameter<double>(
        "poses.cartesian_space.grasp.position.y", 0.0);
    grasp_pose.position.z = get_or_declare_parameter<double>(
        "poses.cartesian_space.grasp.position.z", 0.0);

    grasp_pose.orientation.x = get_or_declare_parameter<double>(
        "poses.cartesian_space.grasp.orientation.x", 0.0);
    grasp_pose.orientation.y = get_or_declare_parameter<double>(
        "poses.cartesian_space.grasp.orientation.y", 0.0);
    grasp_pose.orientation.z = get_or_declare_parameter<double>(
        "poses.cartesian_space.grasp.orientation.z", 0.0);
    grasp_pose.orientation.w = get_or_declare_parameter<double>(
        "poses.cartesian_space.grasp.orientation.w", 1.0);

    config_.cartesian_poses.push_back(grasp_pose);

    RCLCPP_INFO(LOGGER, "  - Loaded cartesian pose: [%s] %s",
                grasp_pose.key.c_str(), grasp_pose.name.c_str());
  }
}

void PickAndPlaceTrajectory::load_cartesian_offsets() {
  RCLCPP_INFO(LOGGER, "Loading cartesian offsets...");

  // Load approach offset
  if (move_group_node_->has_parameter("cartesian_offsets.approach.delta.x")) {
    CartesianOffset approach_offset;
    approach_offset.key = "approach"; // Set the YAML key
    approach_offset.name = get_or_declare_parameter<std::string>(
        "cartesian_offsets.approach.name", "Approach Movement");

    approach_offset.delta.x = get_or_declare_parameter<double>(
        "cartesian_offsets.approach.delta.x", 0.0);
    approach_offset.delta.y = get_or_declare_parameter<double>(
        "cartesian_offsets.approach.delta.y", 0.0);
    approach_offset.delta.z = get_or_declare_parameter<double>(
        "cartesian_offsets.approach.delta.z", 0.0);

    config_.cartesian_offsets.push_back(approach_offset);

    RCLCPP_INFO(LOGGER, "  - Loaded cartesian offset: [%s] %s",
                approach_offset.key.c_str(), approach_offset.name.c_str());
    RCLCPP_INFO(LOGGER, "    Delta: [%.3f, %.3f, %.3f]",
                approach_offset.delta.x, approach_offset.delta.y,
                approach_offset.delta.z);
  }

  // Load retreat offset
  if (move_group_node_->has_parameter("cartesian_offsets.retreat.delta.x")) {
    CartesianOffset retreat_offset;
    retreat_offset.key = "retreat"; // Set the YAML key
    retreat_offset.name = get_or_declare_parameter<std::string>(
        "cartesian_offsets.retreat.name", "Retreat Movement");

    retreat_offset.delta.x = get_or_declare_parameter<double>(
        "cartesian_offsets.retreat.delta.x", 0.0);
    retreat_offset.delta.y = get_or_declare_parameter<double>(
        "cartesian_offsets.retreat.delta.y", 0.0);
    retreat_offset.delta.z = get_or_declare_parameter<double>(
        "cartesian_offsets.retreat.delta.z", 0.0);

    config_.cartesian_offsets.push_back(retreat_offset);

    RCLCPP_INFO(LOGGER, "  - Loaded cartesian offset: [%s] %s",
                retreat_offset.key.c_str(), retreat_offset.name.c_str());
    RCLCPP_INFO(LOGGER, "    Delta: [%.3f, %.3f, %.3f]", retreat_offset.delta.x,
                retreat_offset.delta.y, retreat_offset.delta.z);
  }
}

void PickAndPlaceTrajectory::load_sequence() {
  RCLCPP_INFO(LOGGER, "Loading sequence steps...");

  try {
    // First, get the number of steps
    int num_steps = get_or_declare_parameter<int>("sequence.num_steps", 9);

    RCLCPP_INFO(LOGGER, "Number of sequence steps: %d", num_steps);

    // Load each step using the flattened structure
    for (int i = 0; i < num_steps; i++) {
      SequenceStep step;

      std::string step_prefix = "sequence.step_" + std::to_string(i);

      // Check if this step exists
      if (!move_group_node_->has_parameter(step_prefix + ".step_number")) {
        RCLCPP_WARN(LOGGER, "Step %d not found, skipping", i);
        continue;
      }

      // Load step number
      step.step_number =
          get_or_declare_parameter<int>(step_prefix + ".step_number", i + 1);

      // Load action
      std::string action_str =
          get_or_declare_parameter<std::string>(step_prefix + ".action", "");
      step.action = string_to_action_type(action_str);

      // Load description
      step.description = get_or_declare_parameter<std::string>(
          step_prefix + ".description", "");

      // Load all fields (they're all defined in YAML, even if empty strings)
      step.pose_name =
          get_or_declare_parameter<std::string>(step_prefix + ".pose_name", "");

      step.offset_name = get_or_declare_parameter<std::string>(
          step_prefix + ".offset_name", "");

      step.gripper_command = get_or_declare_parameter<std::string>(
          step_prefix + ".gripper_command", "");

      config_.sequence.push_back(step);

      RCLCPP_INFO(LOGGER, "  Step %d: %s - %s", step.step_number,
                  action_str.c_str(), step.description.c_str());
    }

  } catch (const std::exception &e) {
    RCLCPP_WARN(LOGGER, "Error loading sequence: %s", e.what());
  }
}

ActionType
PickAndPlaceTrajectory::string_to_action_type(const std::string &action_str) {
  if (action_str == "move_to_joint_pose")
    return ActionType::MOVE_TO_JOINT_POSE;
  if (action_str == "move_to_cartesian_pose")
    return ActionType::MOVE_TO_CARTESIAN_POSE;
  if (action_str == "cartesian_offset")
    return ActionType::CARTESIAN_OFFSET;
  if (action_str == "gripper_action")
    return ActionType::GRIPPER_ACTION;
  if (action_str == "move_to_place_position")
    return ActionType::MOVE_TO_PLACE_POSITION;

  return ActionType::UNKNOWN;
}

void PickAndPlaceTrajectory::print_configuration() {
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "CONFIGURATION SUMMARY");
  RCLCPP_INFO(LOGGER, "========================================");

  RCLCPP_INFO(LOGGER, "Joint Poses: %zu", config_.joint_poses.size());
  for (const auto &pose : config_.joint_poses) {
    RCLCPP_INFO(LOGGER, "  - %s", pose.name.c_str());
  }

  RCLCPP_INFO(LOGGER, "Cartesian Poses: %zu", config_.cartesian_poses.size());
  for (const auto &pose : config_.cartesian_poses) {
    RCLCPP_INFO(LOGGER, "  - %s", pose.name.c_str());
  }

  RCLCPP_INFO(LOGGER, "Cartesian Offsets: %zu",
              config_.cartesian_offsets.size());
  for (const auto &offset : config_.cartesian_offsets) {
    RCLCPP_INFO(LOGGER, "  - %s", offset.name.c_str());
  }

  RCLCPP_INFO(LOGGER, "Sequence Steps: %zu", config_.sequence.size());
  for (const auto &step : config_.sequence) {
    RCLCPP_INFO(LOGGER, "  Step %d: %s", step.step_number,
                step.description.c_str());
  }

  RCLCPP_INFO(LOGGER, "========================================");
}

// ==============================================================================
// TRAJECTORY EXECUTION METHOD - SEQUENCE DRIVEN
// ==============================================================================

void PickAndPlaceTrajectory::execute_trajectory_plan() {
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "STARTING PICK AND PLACE SEQUENCE");
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "Total steps: %zu", config_.sequence.size());

  // Execute each step in the sequence
  for (const auto &step : config_.sequence) {
    RCLCPP_INFO(LOGGER, "");
    RCLCPP_INFO(LOGGER, ">>> STEP %d: %s", step.step_number,
                step.description.c_str());
    RCLCPP_INFO(LOGGER, "");

    // Sleep before each step (except the first)
    if (step.step_number > 1) {
      rclcpp::sleep_for(std::chrono::milliseconds(config_.sleep_duration_ms));
    }

    // Execute the step
    bool success = execute_sequence_step(step);

    if (!success) {
      RCLCPP_ERROR(LOGGER, "Step %d FAILED! Aborting sequence.",
                   step.step_number);
      return;
    }

    RCLCPP_INFO(LOGGER, "Step %d completed successfully ✓", step.step_number);
  }

  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "PICK AND PLACE SEQUENCE COMPLETE!");
  RCLCPP_INFO(LOGGER, "========================================");
}

// ==============================================================================
// HELPER METHODS - FIND POSES BY NAME
// ==============================================================================

const JointPose *
PickAndPlaceTrajectory::find_joint_pose(const std::string &name) const {
  for (const auto &pose : config_.joint_poses) {
    if (pose.key == name || pose.name == name) { // Match by key OR name
      return &pose;
    }
  }
  RCLCPP_WARN(LOGGER, "Joint pose '%s' not found!", name.c_str());
  return nullptr;
}

const CartesianPose *
PickAndPlaceTrajectory::find_cartesian_pose(const std::string &name) const {
  for (const auto &pose : config_.cartesian_poses) {
    if (pose.key == name || pose.name == name) { // Match by key OR name
      return &pose;
    }
  }
  RCLCPP_WARN(LOGGER, "Cartesian pose '%s' not found!", name.c_str());
  return nullptr;
}

const CartesianOffset *
PickAndPlaceTrajectory::find_cartesian_offset(const std::string &name) const {
  for (const auto &offset : config_.cartesian_offsets) {
    if (offset.key == name || offset.name == name) { // Match by key OR name
      return &offset;
    }
  }
  RCLCPP_WARN(LOGGER, "Cartesian offset '%s' not found!", name.c_str());
  return nullptr;
}

// ==============================================================================
// SEQUENCE EXECUTION METHODS
// ==============================================================================

bool PickAndPlaceTrajectory::execute_sequence_step(const SequenceStep &step) {
  switch (step.action) {
  case ActionType::MOVE_TO_JOINT_POSE:
    return execute_move_to_joint_pose(step.pose_name);

  case ActionType::MOVE_TO_CARTESIAN_POSE:
    return execute_move_to_cartesian_pose(step.pose_name);

  case ActionType::CARTESIAN_OFFSET:
    return execute_cartesian_offset(step.offset_name);

  case ActionType::GRIPPER_ACTION:
    return execute_gripper_action(step.gripper_command);

  case ActionType::MOVE_TO_PLACE_POSITION:
    return execute_move_to_place_position();

  default:
    RCLCPP_ERROR(LOGGER, "Unknown action type!");
    return false;
  }
}

bool PickAndPlaceTrajectory::execute_move_to_joint_pose(
    const std::string &pose_name) {
  RCLCPP_INFO(LOGGER, "  → Moving to joint pose: %s", pose_name.c_str());

  // Find the pose
  const JointPose *pose = find_joint_pose(pose_name);
  if (!pose) {
    return false;
  }

  // Verify we have 6 joint values
  if (pose->joint_values.size() != 6) {
    RCLCPP_ERROR(LOGGER, "  ✗ Invalid joint values (expected 6, got %zu)",
                 pose->joint_values.size());
    return false;
  }

  // Setup target
  setup_joint_value_target(pose->joint_values[0], pose->joint_values[1],
                           pose->joint_values[2], pose->joint_values[3],
                           pose->joint_values[4], pose->joint_values[5]);

  // Plan
  RCLCPP_INFO(LOGGER, "  → Planning...");
  plan_trajectory_kinematics();

  // Execute
  RCLCPP_INFO(LOGGER, "  → Executing...");
  execute_trajectory_kinematics();

  return plan_success_robot_;
}

bool PickAndPlaceTrajectory::execute_move_to_cartesian_pose(
    const std::string &pose_name) {
  RCLCPP_INFO(LOGGER, "  → Moving to cartesian pose: %s", pose_name.c_str());

  // Find the pose
  const CartesianPose *pose = find_cartesian_pose(pose_name);
  if (!pose) {
    return false;
  }

  // Setup target
  setup_goal_pose_target(pose->position.x, pose->position.y, pose->position.z,
                         pose->orientation.x, pose->orientation.y,
                         pose->orientation.z, pose->orientation.w);

  // Plan
  RCLCPP_INFO(LOGGER, "  → Planning...");
  plan_trajectory_kinematics();

  // Execute
  RCLCPP_INFO(LOGGER, "  → Executing...");
  execute_trajectory_kinematics();

  return plan_success_robot_;
}

bool PickAndPlaceTrajectory::execute_cartesian_offset(
    const std::string &offset_name) {
  RCLCPP_INFO(LOGGER, "  → Applying cartesian offset: %s", offset_name.c_str());

  // Find the offset
  const CartesianOffset *offset = find_cartesian_offset(offset_name);
  if (!offset) {
    return false;
  }

  // Setup waypoints
  setup_waypoints_target(offset->delta.x, offset->delta.y, offset->delta.z);

  // Plan
  RCLCPP_INFO(LOGGER, "  → Planning cartesian path...");
  plan_trajectory_cartesian();

  // Execute
  RCLCPP_INFO(LOGGER, "  → Executing...");
  execute_trajectory_cartesian();

  return (plan_fraction_robot_ >= 0.0);
}

bool PickAndPlaceTrajectory::execute_gripper_action(
    const std::string &command) {
  RCLCPP_INFO(LOGGER, "  → Gripper action: %s", command.c_str());

  if (command == "open") {
    // Use named pose for opening
    setup_named_pose_gripper(config_.gripper_open_pose_name);
  } else if (command == "close") {
    // Use joint value for closing
    setup_joint_value_gripper(config_.gripper_close_value);
  } else {
    RCLCPP_ERROR(LOGGER, "  ✗ Unknown gripper command: %s", command.c_str());
    return false;
  }

  // Plan
  RCLCPP_INFO(LOGGER, "  → Planning gripper action...");
  plan_trajectory_gripper();

  // Execute
  RCLCPP_INFO(LOGGER, "  → Executing...");
  execute_trajectory_gripper();

  return plan_success_gripper_;
}

bool PickAndPlaceTrajectory::execute_move_to_place_position() {
  RCLCPP_INFO(LOGGER, "  → Moving to place position (rotate shoulder)");

  // Get current joint state
  current_state_robot_ = move_group_robot_->getCurrentState(10);
  current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                joint_group_positions_robot_);

  // Rotate shoulder pan joint by ~90 degrees (pi/2 radians)
  // Keep all other joints the same
  setup_joint_value_target(
      +1.5708,                          // shoulder_pan rotated
      joint_group_positions_robot_[1],  // shoulder_lift (keep current)
      joint_group_positions_robot_[2],  // elbow (keep current)
      joint_group_positions_robot_[3],  // wrist_1 (keep current)
      joint_group_positions_robot_[4],  // wrist_2 (keep current)
      joint_group_positions_robot_[5]); // wrist_3 (keep current)

  // Plan
  RCLCPP_INFO(LOGGER, "  → Planning...");
  plan_trajectory_kinematics();

  // Execute
  RCLCPP_INFO(LOGGER, "  → Executing...");
  execute_trajectory_kinematics();

  return plan_success_robot_;
}

// ==============================================================================
// TRAJECTORY PLANNING METHODS (UNCHANGED)
// ==============================================================================

// ==============================================================================
// EXISTING TRAJECTORY PLANNING METHODS
// ==============================================================================

void PickAndPlaceTrajectory::setup_joint_value_target(
    float angle0, float angle1, float angle2, float angle3, float angle4,
    float angle5) {
  joint_group_positions_robot_[0] = angle0;
  joint_group_positions_robot_[1] = angle1;
  joint_group_positions_robot_[2] = angle2;
  joint_group_positions_robot_[3] = angle3;
  joint_group_positions_robot_[4] = angle4;
  joint_group_positions_robot_[5] = angle5;
  move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
}

void PickAndPlaceTrajectory::setup_goal_pose_target(float pos_x, float pos_y,
                                                    float pos_z, float quat_x,
                                                    float quat_y, float quat_z,
                                                    float quat_w) {
  target_pose_robot_.position.x = pos_x;
  target_pose_robot_.position.y = pos_y;
  target_pose_robot_.position.z = pos_z;
  target_pose_robot_.orientation.x = quat_x;
  target_pose_robot_.orientation.y = quat_y;
  target_pose_robot_.orientation.z = quat_z;
  target_pose_robot_.orientation.w = quat_w;
  move_group_robot_->setPoseTarget(target_pose_robot_);
}

void PickAndPlaceTrajectory::plan_trajectory_kinematics() {
  plan_success_robot_ = (move_group_robot_->plan(kinematics_trajectory_plan_) ==
                         moveit::core::MoveItErrorCode::SUCCESS);
}

void PickAndPlaceTrajectory::execute_trajectory_kinematics() {
  if (plan_success_robot_) {
    move_group_robot_->execute(kinematics_trajectory_plan_);
    RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
  } else {
    RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
  }
}

void PickAndPlaceTrajectory::setup_waypoints_target(float x_delta,
                                                    float y_delta,
                                                    float z_delta) {
  target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
  cartesian_waypoints_.push_back(target_pose_robot_);
  target_pose_robot_.position.x += x_delta;
  target_pose_robot_.position.y += y_delta;
  target_pose_robot_.position.z += z_delta;
  cartesian_waypoints_.push_back(target_pose_robot_);
}

void PickAndPlaceTrajectory::plan_trajectory_cartesian() {
  plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
      cartesian_waypoints_, config_.cartesian_end_effector_step,
      config_.cartesian_jump_threshold, cartesian_trajectory_plan_);
}

void PickAndPlaceTrajectory::execute_trajectory_cartesian() {
  if (plan_fraction_robot_ >= 0.0) {
    move_group_robot_->execute(cartesian_trajectory_plan_);
    RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
  } else {
    RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
  }
  cartesian_waypoints_.clear();
}

void PickAndPlaceTrajectory::setup_joint_value_gripper(float angle) {
  joint_group_positions_gripper_[2] = angle;
  move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
}

void PickAndPlaceTrajectory::setup_named_pose_gripper(std::string pose_name) {
  move_group_gripper_->setNamedTarget(pose_name);
}

void PickAndPlaceTrajectory::plan_trajectory_gripper() {
  plan_success_gripper_ =
      (move_group_gripper_->plan(gripper_trajectory_plan_) ==
       moveit::core::MoveItErrorCode::SUCCESS);
}

void PickAndPlaceTrajectory::execute_trajectory_gripper() {
  if (plan_success_gripper_) {
    move_group_gripper_->execute(gripper_trajectory_plan_);
    RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
  } else {
    RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
  }
}

// ==============================================================================
// MAIN FUNCTION
// ==============================================================================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place");

  PickAndPlaceTrajectory pick_and_place_trajectory_node(base_node);

  pick_and_place_trajectory_node.execute_trajectory_plan();

  rclcpp::shutdown();

  return 0;
}

// End of Code