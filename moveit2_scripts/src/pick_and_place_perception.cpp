#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "custom_msgs/msg/detected_objects.hpp"
#include "custom_msgs/msg/detected_surfaces.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// program variables
// static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_node");
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

    move_group_node_ =
        rclcpp::Node::make_shared("pick_and_place_node", node_options);

    // Create subscriber for object detection
    object_detected_sub_ =
        move_group_node_
            ->create_subscription<custom_msgs::msg::DetectedObjects>(
                "/object_detected", 10,
                std::bind(&PickAndPlaceTrajectory::object_detected_callback,
                          this, std::placeholders::_1));

    surface_detected_sub_ =
        move_group_node_
            ->create_subscription<custom_msgs::msg::DetectedSurfaces>(
                "/surface_detected", 10,
                std::bind(&PickAndPlaceTrajectory::surface_detected_callback,
                          this, std::placeholders::_1));

    RCLCPP_INFO(LOGGER,
                "Subscribed to /object_detected topic & /surface_detected");

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
    // more efficient method than std::copy() method used in the docs
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
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Trajectory");
  }

  // Callback for object detection
  void object_detected_callback(
      const custom_msgs::msg::DetectedObjects::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(object_mutex_);

    object_detected_ = true;
    detected_object_x_ = msg->position.x;
    detected_object_y_ = msg->position.y;
    detected_object_z_ = msg->position.z;
    detected_object_height_ = msg->height;
    detected_object_width_ = msg->width;
    detected_object_thickness_ = msg->thickness;

    auto &clk = *move_group_node_->get_clock();
    //  Object detected at position: [0.328, -0.011, 0.028]
    RCLCPP_INFO_THROTTLE(
        LOGGER, clk, 30000, " Object detected at position: [%.3f, %.3f, %.3f]",
        detected_object_x_, detected_object_y_, detected_object_z_);
    RCLCPP_INFO_THROTTLE(LOGGER, clk, 30000,
                         " Object dimensions: H=%.3f, W=%.3f, T=%.3f",
                         detected_object_height_, detected_object_width_,
                         detected_object_thickness_);
  }

  void surface_detected_callback(
      const custom_msgs::msg::DetectedSurfaces::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    surface_detected_ = true;
    detected_surface_x_ = msg->position.x;
    detected_surface_y_ = msg->position.y;
    detected_surface_z_ = msg->position.z;
    detected_surface_length_ = msg->height;
    detected_surface_width_ = msg->width;

    RCLCPP_INFO(LOGGER, " Surface at [%.3f, %.3f, %.3f], L=%.3f, W=%.3f",
                detected_surface_x_, detected_surface_y_, detected_surface_z_,
                detected_surface_length_, detected_surface_width_);
  }

  // Wait for object detection with timeout
  bool wait_for_object_detection(double timeout_seconds = 10.0) {
    RCLCPP_INFO(LOGGER, "Waiting for object detection...");

    auto start_time = std::chrono::steady_clock::now();
    while (!object_detected_) {
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::steady_clock::now() - start_time)
                         .count();

      if (elapsed >= timeout_seconds) {
        RCLCPP_ERROR(LOGGER, "Timeout waiting for object detection!");
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(LOGGER, "Object detection received!");
    return true;
  }

  bool wait_for_surface_detection(double timeout_sec = 10.0) {
    RCLCPP_INFO(LOGGER, "Waiting for surface detection...");
    auto start = std::chrono::steady_clock::now();

    while (!surface_detected_) {
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::steady_clock::now() - start)
                         .count();
      if (elapsed >= timeout_sec) {
        RCLCPP_ERROR(LOGGER, "Timeout waiting for surface!");
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
  }

  void add_table_collision_object() {
    // Get surface info (thread-safe)
    double surf_x, surf_y, surf_z, surf_length, surf_width;
    {
      std::lock_guard<std::mutex> lock(surface_mutex_);
      surf_x = detected_surface_x_;
      surf_y = detected_surface_y_;
      surf_z = detected_surface_z_;
      surf_length = detected_surface_length_;
      surf_width = detected_surface_width_;
    }

    double table_thickness = 0.05; // 5cm

    RCLCPP_INFO(LOGGER, "Adding table collision object [%.3f x %.3f x %.3f]",
                surf_length, surf_width, table_thickness);

    // Create collision object
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_robot_->getPlanningFrame();
    collision_object.id = "table";

    // Box shape
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = surf_length;
    primitive.dimensions[primitive.BOX_Y] = surf_width;
    primitive.dimensions[primitive.BOX_Z] = table_thickness;

    // Box pose
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = surf_x;
    box_pose.position.y = surf_y;
    box_pose.position.z = surf_z - table_thickness / 2.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add to scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface_.addCollisionObjects(collision_objects);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(LOGGER, " Table added to planning scene");
  }

  void remove_table_collision_object() {
    std::vector<std::string> object_ids;
    object_ids.push_back("table");
    planning_scene_interface_.removeCollisionObjects(object_ids);
    RCLCPP_INFO(LOGGER, "Table collision object removed");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Pick And Place Trajectory...");

    RCLCPP_INFO(LOGGER, "-- Going to Home Position...");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    // setup the joint value target
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    setup_joint_value_target(+0.0000, -2.3562, +1.5708, -1.5708, -1.5708,
                             +0.0000);
    // setup_goal_pose_target(0.104, 0.131, 0.470, 0.653, -0.653, 0.271,
    // -0.271);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    auto joints = move_group_robot_->getCurrentJointValues();
    // ✎ Home pos Joints: [-0.0000, -2.3563, 1.5708, -1.5707, -1.5707, -0.0001]
    RCLCPP_INFO(
        LOGGER, "✎ Home pos Joints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

    // Wait for object detection

    if (!wait_for_surface_detection(10.0))
      return;
    add_table_collision_object();
    if (!wait_for_object_detection(10.0)) {
      RCLCPP_ERROR(LOGGER, "Cannot proceed without object detection!");
      return;
    }

    //  Object detected at position: [x: 0.328, y: -0.011, z: 0.028]
    // Object dimensions: H=0.020, W=0.019, T=0.052

    // Get the detected object position (thread-safe)
    double obj_x, obj_y, obj_z, obj_thickness, obj_width, obj_height;
    {
      std::lock_guard<std::mutex> lock(object_mutex_);
      // tolarance/ correction addition for simulation purpose
      obj_x = detected_object_x_ + 0.002;
      obj_y = detected_object_y_;
      obj_z = detected_object_z_ + 0.11;
      obj_thickness = detected_object_thickness_;
      obj_width = detected_object_width_;
      obj_height = detected_object_height_;
    }

    // Calculate pregrasp position (above the object)
    // double pregrasp_offset = 0.08; // 8cm above object center
    // double pregrasp_offset = 0.25;
    double pregrasp_offset = 0.15;
    double pregrasp_x = obj_x + obj_height / 2.0;
    double pregrasp_y = obj_y - obj_width / 2.0;
    double pregrasp_z = obj_z + pregrasp_offset;

    // Calculate grasp approach distance (move down to grasp)
    double approach_distance = -(pregrasp_offset - obj_thickness);

    RCLCPP_INFO(LOGGER, "-- Going to Pregrasp Position...");
    // rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    // setup the goal pose target
    RCLCPP_INFO(LOGGER, "Preparing Goal Pose Trajectory...");

    setup_joint_value_target(-0.4544, -1.5023, 1.5699, -1.6383, -1.5715,
                             -2.0256);
    // setup_goal_pose_target(pregrasp_x, pregrasp_y, pregrasp_z, -1.000,
    // +0.000,
    //                        +0.000, +0.000);

    // RCLCPP_INFO(LOGGER, " >>>> %f, %f, %f", pregrasp_x, pregrasp_y,
    // pregrasp_z); RCLCPP_INFO(LOGGER, " >>>> 0.340, -0.020, 0.264");

    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Goal Pose Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Goal Pose Trajectory...");
    execute_trajectory_kinematics();

    joints = move_group_robot_->getCurrentJointValues();
    // ✎ PreGrsp pos Joints: [-0.4544, -1.5023, 1.5699, -1.6383, -1.5715,
    // -2.0256]
    RCLCPP_INFO(
        LOGGER, "✎ PreGrsp pos Joints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

    // open the gripper
    RCLCPP_INFO(LOGGER, "-- Opening Gripper...");
    // rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    // setup the gripper target by pose name
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    // setup_named_pose_gripper("open");
    setup_named_pose_gripper("gripper_open");
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Opened");

    RCLCPP_INFO(LOGGER, "-- Approaching...");
    // rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    // setup the cartesian target
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory...");
    // setup_waypoints_target(+0.000, +0.000, -0.078);
    setup_waypoints_target(+0.000, +0.000, approach_distance);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Cartesian Trajectory...");
    plan_trajectory_cartesian();
    RCLCPP_INFO(LOGGER, "Executing Cartesian Trajectory...");
    execute_trajectory_cartesian();

    // throw
    // std::runtime_error("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");

    // close the gripper
    RCLCPP_INFO(LOGGER, "-- Closing Gripper...");
    // rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    // setup the gripper joint value
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    // for (double i = 0.50; i <= 0.6525; i += 0.030) {
    // for (double i = 0.60; i <= 0.652; i += 0.013) {
    // for (double i = 0.60; i <= 0.646; i += 0.0092) {
    for (double i = 0.60; i <= 0.6480; i += 0.0096) {
      // for (double i = 0.60; i <= 0.6455; i += 0.0091) { -----
      // for (double i = 0.60; i <= 0.645; i += 0.009) {
      // for (double i = 0.60; i <= 0.638; i += 0.0065) {
      // for (double i = 0.60; i <= 0.635; i += 0.012) {
      setup_joint_value_gripper(i);
      plan_trajectory_gripper();
      execute_trajectory_gripper();
      //   rclcpp::sleep_for(std::chrono::milliseconds(850));
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(LOGGER, "Gripper Closed");

    remove_table_collision_object();

    RCLCPP_INFO(LOGGER, "-- Retreating...");
    //   rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    // setup the cartesian target
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory...");
    // setup_waypoints_target(+0.000, +0.000, +0.060);
    setup_waypoints_target(+0.000, +0.000, pregrasp_z);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Cartesian Trajectory...");
    plan_trajectory_cartesian();
    RCLCPP_INFO(LOGGER, "Executing Cartesian Trajectory...");
    execute_trajectory_cartesian();

    joints = move_group_robot_->getCurrentJointValues();
    // ✎ retreated pos Joints: [-0.4544, -1.1809, 0.3071, -0.6964, -1.5716,
    // -2.0256]
    RCLCPP_INFO(
        LOGGER, "✎ retreated pos Joints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

    RCLCPP_INFO(LOGGER, "-- Going to Place Position...");
    //   rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    // setup the joint value target
    RCLCPP_INFO(LOGGER, "-- Preparing Joint Value Trajectory...");
    setup_joint_value_target(
        +3.1416, joint_group_positions_robot_[1],
        joint_group_positions_robot_[2], joint_group_positions_robot_[3],
        joint_group_positions_robot_[4], joint_group_positions_robot_[5]);
    // setup_goal_pose_target(-0.315, -0.131, 0.448, -0.225, 0.974, 0.000,
    // 0.001);
    RCLCPP_INFO(LOGGER, "-- Planning Joint Value Trajectory...");
    //   rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    joints = move_group_robot_->getCurrentJointValues();
    // ✎ Drop pos Joints: [3.1415, -1.1809, 0.3071, -0.6963, -1.5717, -2.0256]
    RCLCPP_INFO(
        LOGGER, "✎ Drop pos Joints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

    rclcpp::sleep_for(std::chrono::milliseconds(5000));

    // open the gripper
    RCLCPP_INFO(LOGGER, "-- Opening Gripper...");
    //   rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    // setup the gripper target by pose name
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    // setup_named_pose_gripper("open");
    setup_named_pose_gripper("gripper_open");
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Opened");

    // wait for few seconds
    //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(LOGGER, "-- Going Back to Home Position...");
    rclcpp::sleep_for(std::chrono::milliseconds(sleep2_));
    // setup the joint value target
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    // ✎ back2Home pos Joints: [3.1415, -1.1809, 0.3071, -0.6963, -1.5717,
    // -2.0256] setup_joint_value_target(3.1415, -1.1809, 0.3071, -0.6963,
    // -1.5717, -2.0256);
    setup_joint_value_target(+0.0000, -2.3562, +1.5708, -1.5708, -1.5708,
                             +0.0000);
    // setup_goal_pose_target(0.104, 0.131, 0.470, 0.653, -0.653, 0.271,
    // -0.271);

    // plan and execute the trajectory
    joints = move_group_robot_->getCurrentJointValues();
    RCLCPP_INFO(
        LOGGER, "✎ back2Home pos Joints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    RCLCPP_INFO(LOGGER, "Pick And Place Trajectory Execution Complete");
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using PlanningSceneInterface =
      moveit::planning_interface::PlanningSceneInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Subscriber for object detection
  rclcpp::Subscription<custom_msgs::msg::DetectedObjects>::SharedPtr
      object_detected_sub_;
  rclcpp::Subscription<custom_msgs::msg::DetectedSurfaces>::SharedPtr
      surface_detected_sub_;
  PlanningSceneInterface planning_scene_interface_;

  // Object detection variables
  std::mutex object_mutex_;
  bool object_detected_ = false;
  double detected_object_x_ = 0.0;
  double detected_object_y_ = 0.0;
  double detected_object_z_ = 0.0;
  double detected_object_height_ = 0.0;
  double detected_object_width_ = 0.0;
  double detected_object_thickness_ = 0.0;
  std::mutex surface_mutex_;
  bool surface_detected_ = false;
  double detected_surface_x_ = 0.0;
  double detected_surface_y_ = 0.0;
  double detected_surface_z_ = 0.0;
  double detected_surface_length_ = 0.0;
  double detected_surface_width_ = 0.0;

  // MoveIt interfaces
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  // declare joint_model_group for robot and gripper
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  // declare trajectory planning variables for robot and gripper
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;

  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;

  const int sleep2_ = 1500;

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void plan_trajectory_kinematics() {
    // plan the trajectory to target using kinematics
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    // execute the planned trajectory to target using kinematics
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    // add the current pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
    // calculate the desired pose from delta value for the axis
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    // add the desired pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    // plan the trajectory to target using cartesian path
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    // execute the planned trajectory to target using cartesian path
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }

  void setup_joint_value_gripper(float angle) {
    // set the joint values for each joint of gripper
    // based on values provided
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void setup_named_pose_gripper(std::string pose_name) {
    // set the joint values for each joint of gripper
    // based on predefined pose names
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    // plan the gripper action
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    // execute the planned gripper action
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }

}; // class PickAndPlaceTrajectory

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place");

  // instantiate class
  PickAndPlaceTrajectory pick_and_place_trajectory_node(base_node);

  // execute trajectory plan
  pick_and_place_trajectory_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}
