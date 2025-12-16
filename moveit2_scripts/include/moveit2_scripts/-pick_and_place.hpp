#ifndef PICK_AND_PLACE_HPP
#define PICK_AND_PLACE_HPP

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ==============================================================================
// POSE STRUCTURE DEFINITIONS
// ==============================================================================

/**
 * @brief Structure to hold a 3D position (x, y, z)
 */
typedef struct Position {
  double x;
  double y;
  double z;

  // Constructor with default values
  Position() : x(0.0), y(0.0), z(0.0) {}
  Position(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
} Position;

/**
 * @brief Structure to hold a quaternion orientation (x, y, z, w)
 */
typedef struct Orientation {
  double x;
  double y;
  double z;
  double w;

  // Constructor with default values
  Orientation() : x(0.0), y(0.0), z(0.0), w(1.0) {}
  Orientation(double x_, double y_, double z_, double w_)
      : x(x_), y(y_), z(z_), w(w_) {}
} Orientation;

/**
 * @brief Structure to hold a Cartesian pose (position + orientation)
 */
typedef struct CartesianPose {
  std::string key;  // YAML key (e.g., "pregrasp")
  std::string name; // Display name (e.g., "Pregrasp Position")
  Position position;
  Orientation orientation;

  // Constructor
  CartesianPose() : key(""), name("Unnamed Pose") {}
  CartesianPose(const std::string &key_, const std::string &name_, 
                const Position &pos, const Orientation &orient)
      : key(key_), name(name_), position(pos), orientation(orient) {}
} CartesianPose;

/**
 * @brief Structure to hold a Joint Space pose (all 6 joint angles)
 */
typedef struct JointPose {
  std::string key;  // YAML key (e.g., "home")
  std::string name; // Display name (e.g., "Home Position")
  std::vector<double> joint_values; // Size should be 6 for UR3e

  // Constructor
  JointPose() : key(""), name("Unnamed Joint Pose"), joint_values(6, 0.0) {}
  JointPose(const std::string &key_, const std::string &name_, 
            const std::vector<double> &values)
      : key(key_), name(name_), joint_values(values) {}
} JointPose;

/**
 * @brief Structure to hold a Cartesian offset (relative movement)
 */
typedef struct CartesianOffset {
  std::string key;  // YAML key (e.g., "approach")
  std::string name; // Display name (e.g., "Approach Movement")
  Position delta; // Delta values for x, y, z

  // Constructor
  CartesianOffset() : key(""), name("Unnamed Offset") {}
  CartesianOffset(const std::string &key_, const std::string &name_, 
                  const Position &delta_)
      : key(key_), name(name_), delta(delta_) {}
} CartesianOffset;

/**
 * @brief Enum to define different action types in sequence
 */
enum class ActionType {
  MOVE_TO_JOINT_POSE,
  MOVE_TO_CARTESIAN_POSE,
  CARTESIAN_OFFSET,
  GRIPPER_ACTION,
  MOVE_TO_PLACE_POSITION,
  UNKNOWN
};

/**
 * @brief Structure to hold a sequence step
 */
typedef struct SequenceStep {
  int step_number;
  ActionType action;
  std::string pose_name;       // For joint/cartesian poses
  std::string offset_name;     // For cartesian offsets
  std::string gripper_command; // "open" or "close"
  std::string description;

  // Constructor
  SequenceStep()
      : step_number(0), action(ActionType::UNKNOWN), pose_name(""),
        offset_name(""), gripper_command(""), description("") {}
} SequenceStep;

/**
 * @brief Structure to hold all configuration parameters
 */
typedef struct PickAndPlaceConfig {
  // Timing parameters
  int sleep_duration_ms;

  // Planning parameters
  double cartesian_end_effector_step;
  double cartesian_jump_threshold;
  double kinematics_planning_time;
  int kinematics_num_planning_attempts;

  // Gripper parameters
  double gripper_open_value;
  double gripper_close_value;
  std::string gripper_open_pose_name;
  std::string gripper_close_pose_name;

  // Pose storage
  std::vector<JointPose> joint_poses;
  std::vector<CartesianPose> cartesian_poses;
  std::vector<CartesianOffset> cartesian_offsets;

  // Sequence of operations
  std::vector<SequenceStep> sequence;

  // Constructor with default values
  PickAndPlaceConfig()
      : sleep_duration_ms(3000), cartesian_end_effector_step(0.01),
        cartesian_jump_threshold(0.0), kinematics_planning_time(5.0),
        kinematics_num_planning_attempts(10), gripper_open_value(0.0),
        gripper_close_value(0.5), gripper_open_pose_name("gripper_open"),
        gripper_close_pose_name("gripper_close") {}
} PickAndPlaceConfig;

// ==============================================================================
// MAIN CLASS DECLARATION
// ==============================================================================

class PickAndPlaceTrajectory {
public:
  PickAndPlaceTrajectory(rclcpp::Node::SharedPtr base_node_);
  ~PickAndPlaceTrajectory();

  void execute_trajectory_plan();

private:
  // Using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  // ROS2 nodes
  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // MoveIt interfaces
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  // Trajectory planning variables
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_;

  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_;

  // Cartesian trajectory planning variables
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  double plan_fraction_robot_;

  // Configuration structure
  PickAndPlaceConfig config_;

  // Logger
  static const rclcpp::Logger LOGGER;
  static const std::string PLANNING_GROUP_ROBOT;
  static const std::string PLANNING_GROUP_GRIPPER;

  // ============================================================================
  // PARAMETER LOADING METHODS
  // ============================================================================

  /**
   * @brief Load all parameters from ROS2 parameter server
   */
  void load_parameters();

  /**
   * @brief Load timing parameters
   */
  void load_timing_parameters();

  /**
   * @brief Load planning parameters
   */
  void load_planning_parameters();

  /**
   * @brief Load gripper parameters
   */
  void load_gripper_parameters();

  /**
   * @brief Load joint space poses
   */
  void load_joint_poses();

  /**
   * @brief Load cartesian space poses
   */
  void load_cartesian_poses();

  /**
   * @brief Load cartesian offsets
   */
  void load_cartesian_offsets();

  /**
   * @brief Load sequence steps
   */
  void load_sequence();

  /**
   * @brief Helper function to convert action string to ActionType enum
   */
  ActionType string_to_action_type(const std::string &action_str);

  /**
   * @brief Helper to safely get or declare a parameter
   * @tparam T Parameter type
   * @param name Parameter name
   * @param default_value Default value if parameter doesn't exist
   * @return Parameter value
   */
  template <typename T>
  T get_or_declare_parameter(const std::string &name, const T &default_value) {
    try {
      if (move_group_node_->has_parameter(name)) {
        T value;
        move_group_node_->get_parameter(name, value);
        return value;
      } else {
        return move_group_node_->declare_parameter(name, default_value);
      }
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      T value;
      move_group_node_->get_parameter(name, value);
      return value;
    }
  }

  /**
   * @brief Print loaded configuration for debugging
   */
  void print_configuration();

  // ============================================================================
  // HELPER METHODS FOR FINDING POSES BY NAME
  // ============================================================================

  /**
   * @brief Find a joint pose by name
   * @param name Name of the joint pose
   * @return Pointer to JointPose if found, nullptr otherwise
   */
  const JointPose* find_joint_pose(const std::string &name) const;

  /**
   * @brief Find a cartesian pose by name
   * @param name Name of the cartesian pose
   * @return Pointer to CartesianPose if found, nullptr otherwise
   */
  const CartesianPose* find_cartesian_pose(const std::string &name) const;

  /**
   * @brief Find a cartesian offset by name
   * @param name Name of the cartesian offset
   * @return Pointer to CartesianOffset if found, nullptr otherwise
   */
  const CartesianOffset* find_cartesian_offset(const std::string &name) const;

  // ============================================================================
  // SEQUENCE EXECUTION METHODS
  // ============================================================================

  /**
   * @brief Execute a single sequence step
   * @param step The sequence step to execute
   * @return true if successful, false otherwise
   */
  bool execute_sequence_step(const SequenceStep &step);

  /**
   * @brief Execute move to joint pose action
   * @param pose_name Name of the joint pose
   * @return true if successful
   */
  bool execute_move_to_joint_pose(const std::string &pose_name);

  /**
   * @brief Execute move to cartesian pose action
   * @param pose_name Name of the cartesian pose
   * @return true if successful
   */
  bool execute_move_to_cartesian_pose(const std::string &pose_name);

  /**
   * @brief Execute cartesian offset action
   * @param offset_name Name of the cartesian offset
   * @return true if successful
   */
  bool execute_cartesian_offset(const std::string &offset_name);

  /**
   * @brief Execute gripper action
   * @param command "open" or "close"
   * @return true if successful
   */
  bool execute_gripper_action(const std::string &command);

  /**
   * @brief Execute move to place position (rotate shoulder 180 degrees)
   * @return true if successful
   */
  bool execute_move_to_place_position();

  // ============================================================================
  // TRAJECTORY PLANNING METHODS (existing methods)
  // ============================================================================

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                 float angle3, float angle4, float angle5);

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w);

  void plan_trajectory_kinematics();
  void execute_trajectory_kinematics();

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta);
  void plan_trajectory_cartesian();
  void execute_trajectory_cartesian();

  void setup_joint_value_gripper(float angle);
  void setup_named_pose_gripper(std::string pose_name);
  void plan_trajectory_gripper();
  void execute_trajectory_gripper();

}; // class PickAndPlaceTrajectory

#endif // PICK_AND_PLACE_HPP