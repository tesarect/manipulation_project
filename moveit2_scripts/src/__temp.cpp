#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

// Include custom message header
#include "custom_msgs/msg/detected_objects.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>
#include <mutex>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlaceTrajectory {
public:
  PickAndPlaceTrajectory(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Trajectory...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    
    // Create subscriber for object detection
    object_detected_sub_ = move_group_node_->create_subscription<custom_msgs::msg::DetectedObjects>(
        "/object_detected",
        10,
        std::bind(&PickAndPlaceTrajectory::object_detected_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(LOGGER, "Subscribed to /object_detected topic");
    
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of robot and gripper
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot and gripper
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // set start state of robot and gripper to current state
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Trajectory");
  }

  ~PickAndPlaceTrajectory() {
    RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Trajectory");
  }

  // Callback for object detection
  void object_detected_callback(const custom_msgs::msg::DetectedObjects::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(object_mutex_);
    
    object_detected_ = true;
    detected_object_x_ = msg->position.x;
    detected_object_y_ = msg->position.y;
    detected_object_z_ = msg->position.z;
    detected_object_height_ = msg->height;
    detected_object_width_ = msg->width;
    detected_object_thickness_ = msg->thickness;
    
    RCLCPP_INFO(LOGGER, "Object detected at position: [%.3f, %.3f, %.3f]",
                detected_object_x_, detected_object_y_, detected_object_z_);
    RCLCPP_INFO(LOGGER, "Object dimensions: H=%.3f, W=%.3f, T=%.3f",
                detected_object_height_, detected_object_width_, detected_object_thickness_);
  }

  // Wait for object detection with timeout
  bool wait_for_object_detection(double timeout_seconds = 10.0) {
    RCLCPP_INFO(LOGGER, "Waiting for object detection...");
    
    auto start_time = std::chrono::steady_clock::now();
    while (!object_detected_) {
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start_time).count();
      
      if (elapsed >= timeout_seconds) {
        RCLCPP_ERROR(LOGGER, "Timeout waiting for object detection!");
        return false;
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO(LOGGER, "Object detection received!");
    return true;
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "");
    RCLCPP_INFO(LOGGER, "========================================");
    RCLCPP_INFO(LOGGER, "PICK AND PLACE WITH PERCEPTION");
    RCLCPP_INFO(LOGGER, "========================================");
    
    // Wait for object detection
    if (!wait_for_object_detection(10.0)) {
      RCLCPP_ERROR(LOGGER, "Cannot proceed without object detection!");
      return;
    }

    // Get the detected object position (thread-safe)
    double obj_x, obj_y, obj_z, obj_thickness;
    {
      std::lock_guard<std::mutex> lock(object_mutex_);
      obj_x = detected_object_x_;
      obj_y = detected_object_y_;
      obj_z = detected_object_z_;
      obj_thickness = detected_object_thickness_;
    }

    // Calculate pregrasp position (above the object)
    double pregrasp_offset = 0.08;  // 8cm above object center
    double pregrasp_x = obj_x;
    double pregrasp_y = obj_y;
    double pregrasp_z = obj_z + pregrasp_offset;

    // Calculate grasp approach distance (move down to grasp)
    double approach_distance = -(pregrasp_offset - obj_thickness/2.0);

    RCLCPP_INFO(LOGGER, "");
    RCLCPP_INFO(LOGGER, "Object position: [%.3f, %.3f, %.3f]", obj_x, obj_y, obj_z);
    RCLCPP_INFO(LOGGER, "Pregrasp position: [%.3f, %.3f, %.3f]", pregrasp_x, pregrasp_y, pregrasp_z);
    RCLCPP_INFO(LOGGER, "Approach distance: %.3f", approach_distance);
    RCLCPP_INFO(LOGGER, "");

    // ========================================================================
    // STEP 1: Move to Pregrasp Position
    // ========================================================================
    RCLCPP_INFO(LOGGER, ">>> STEP 1: Moving to Pregrasp Position...");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    
    RCLCPP_INFO(LOGGER, "  → Setting target pose");
    setup_goal_pose_target(pregrasp_x, pregrasp_y, pregrasp_z, 
                          -1.000, +0.000, +0.000, +0.000);
    
    RCLCPP_INFO(LOGGER, "  → Planning...");
    plan_trajectory_kinematics();
    
    RCLCPP_INFO(LOGGER, "  → Executing...");
    execute_trajectory_kinematics();
    
    RCLCPP_INFO(LOGGER, "Step 1 complete ✓");
    RCLCPP_INFO(LOGGER, "");

    // ========================================================================
    // STEP 2: Open Gripper
    // ========================================================================
    RCLCPP_INFO(LOGGER, ">>> STEP 2: Opening Gripper...");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    
    RCLCPP_INFO(LOGGER, "  → Setting gripper to open");
    setup_named_pose_gripper("gripper_open");
    
    RCLCPP_INFO(LOGGER, "  → Planning...");
    plan_trajectory_gripper();
    
    RCLCPP_INFO(LOGGER, "  → Executing...");
    execute_trajectory_gripper();
    
    RCLCPP_INFO(LOGGER, "Step 2 complete ✓");
    RCLCPP_INFO(LOGGER, "");

    // ========================================================================
    // STEP 3: Approach Object
    // ========================================================================
    RCLCPP_INFO(LOGGER, ">>> STEP 3: Approaching Object...");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    
    RCLCPP_INFO(LOGGER, "  → Setting cartesian offset");
    setup_waypoints_target(+0.000, +0.000, approach_distance);
    
    RCLCPP_INFO(LOGGER, "  → Planning cartesian path...");
    plan_trajectory_cartesian();
    
    RCLCPP_INFO(LOGGER, "  → Executing...");
    execute_trajectory_cartesian();
    
    RCLCPP_INFO(LOGGER, "Step 3 complete ✓");
    RCLCPP_INFO(LOGGER, "");

    // ========================================================================
    // STEP 4: Close Gripper
    // ========================================================================
    RCLCPP_INFO(LOGGER, ">>> STEP 4: Closing Gripper...");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    
    RCLCPP_INFO(LOGGER, "  → Gradually closing gripper");
    for (double i = 0.50; i <= 0.6525; i += 0.030) {
      setup_joint_value_gripper(i);
      plan_trajectory_gripper();
      execute_trajectory_gripper();
      rclcpp::sleep_for(std::chrono::milliseconds(300));
    }
    
    RCLCPP_INFO(LOGGER, "Step 4 complete ✓");
    RCLCPP_INFO(LOGGER, "");

    // ========================================================================
    // STEP 5: Retreat with Object
    // ========================================================================
    RCLCPP_INFO(LOGGER, ">>> STEP 5: Retreating with Object...");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    
    RCLCPP_INFO(LOGGER, "  → Setting cartesian offset");
    setup_waypoints_target(+0.000, +0.000, +0.060);
    
    RCLCPP_INFO(LOGGER, "  → Planning cartesian path...");
    plan_trajectory_cartesian();
    
    RCLCPP_INFO(LOGGER, "  → Executing...");
    execute_trajectory_cartesian();
    
    RCLCPP_INFO(LOGGER, "Step 5 complete ✓");
    RCLCPP_INFO(LOGGER, "");

    // ========================================================================
    // STEP 6: Move to Place Position
    // ========================================================================
    RCLCPP_INFO(LOGGER, ">>> STEP 6: Moving to Place Position...");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    
    // Get current state
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    
    RCLCPP_INFO(LOGGER, "  → Rotating shoulder 180 degrees");
    setup_joint_value_target(
        +3.1416, joint_group_positions_robot_[1],
        joint_group_positions_robot_[2], joint_group_positions_robot_[3],
        joint_group_positions_robot_[4], joint_group_positions_robot_[5]);
    
    RCLCPP_INFO(LOGGER, "  → Planning...");
    plan_trajectory_kinematics();
    
    RCLCPP_INFO(LOGGER, "  → Executing...");
    execute_trajectory_kinematics();
    
    RCLCPP_INFO(LOGGER, "Step 6 complete ✓");
    RCLCPP_INFO(LOGGER, "");

    // ========================================================================
    // STEP 7: Open Gripper (Release Object)
    // ========================================================================
    RCLCPP_INFO(LOGGER, ">>> STEP 7: Opening Gripper (Release)...");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    
    RCLCPP_INFO(LOGGER, "  → Setting gripper to open");
    setup_named_pose_gripper("gripper_open");
    
    RCLCPP_INFO(LOGGER, "  → Planning...");
    plan_trajectory_gripper();
    
    RCLCPP_INFO(LOGGER, "  → Executing...");
    execute_trajectory_gripper();
    
    RCLCPP_INFO(LOGGER, "Step 7 complete ✓");
    RCLCPP_INFO(LOGGER, "");

    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(LOGGER, "");
    RCLCPP_INFO(LOGGER, "========================================");
    RCLCPP_INFO(LOGGER, "PICK AND PLACE COMPLETE!");
    RCLCPP_INFO(LOGGER, "========================================");
    RCLCPP_INFO(LOGGER, "");
  }

  // ... [Rest of the methods remain the same: setup_joint_value_target, 
  //      setup_goal_pose_target, etc. - keep all existing methods]

private:
  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Subscriber for object detection
  rclcpp::Subscription<custom_msgs::msg::DetectedObjects>::SharedPtr object_detected_sub_;

  // Object detection variables
  std::mutex object_mutex_;
  bool object_detected_ = false;
  double detected_object_x_ = 0.0;
  double detected_object_y_ = 0.0;
  double detected_object_z_ = 0.0;
  double detected_object_height_ = 0.0;
  double detected_object_width_ = 0.0;
  double detected_object_thickness_ = 0.0;

  // MoveIt interfaces
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  // ... [Rest of the private members remain the same]
  
  const moveit::core::JointModelGroup *joint_model_group_robot_;
  const moveit::core::JointModelGroup *joint_model_group_gripper_;

  moveit::core::RobotStatePtr current_state_robot_;
  moveit::core::RobotStatePtr current_state_gripper_;

  std::vector<double> joint_group_positions_robot_;
  std::vector<double> joint_group_positions_gripper_;

  MoveGroupInterface::Plan plan_robot_;
  MoveGroupInterface::Plan plan_gripper_;

  bool plan_success_robot_;
  bool plan_success_gripper_;
  double plan_fraction_robot_;

  int sleep2_ = 3000;  // milliseconds
};

// ... [Keep all the remaining method implementations like setup_joint_value_target, etc.]

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  // Create base node
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto base_node = rclcpp::Node::make_shared("base_node", node_options);
  
  // Create pick and place trajectory object
  auto pick_and_place = std::make_shared<PickAndPlaceTrajectory>(base_node);
  
  // Execute trajectory
  pick_and_place->execute_trajectory_plan();
  
  // Shutdown
  rclcpp::shutdown();
  return 0;
}