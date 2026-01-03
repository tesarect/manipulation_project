import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory
from os.path import join as pthJoin

def generate_launch_description():

    # config_pkg = get_package_share_directory('sim_real_config')
    # param_config = pthJoin(config_pkg, 'config', 'sim.yaml')

    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    # MoveItCpp demo executable
    pick_and_place_node = Node(
        name="pick_and_place_perception",
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # param_config,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription(
        [pick_and_place_node]
    )
