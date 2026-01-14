import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from os.path import join as pthJoin

def generate_launch_description():

    # config_pkg = get_package_share_directory('sim_real_config')
    # param_config = pthJoin(config_pkg, 'config', 'sim.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # rviz_config = os.path.join(get_package_share_directory('my_moveit_config'), 'config', 'moveit_testing.rviz')
    rviz_config = os.path.join(get_package_share_directory('real_moveit_config'), 'config', 'moveit.rviz')

    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arm2ptc_static_tf',
        arguments=[
            '0.338', '0.450', '0.100',              # translation (x, y, z)
            '0.000', '0.866', '-0.500', '0.000',    # quaternion (qx qy qz qw)
            'base_link',
            'wrist_rgbd_camera_depth_optical_frame'
        ])
    
    rviz =  Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config]
        )

    scripted_static_tf = Node(
            package='object_detection',
            executable='static_transform_publisher_real',
            output='screen',
            name='arm2ptc_static_tf',
            parameters=[{'use_sim_time': use_sim_time}],
            emulate_tty=True,
        )

    
    obj_detect = Node(
            package='object_detection',
            executable='object_detection_real',
            output='screen',
            name='object_detection_real',
            parameters=[{'use_sim_time': use_sim_time}],
            emulate_tty=True,
        )

    # MoveItCpp demo executable
    pick_and_place_node = Node(
        name="pick_and_place_perception_real",
        package="moveit2_scripts",
        executable="pick_and_place_perception_real",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # param_config,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription(
        [
            # static_tf,
            scripted_static_tf,
            # rviz,
            obj_detect,
            pick_and_place_node
        ]
    )
