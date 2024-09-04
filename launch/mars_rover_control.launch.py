import os
import pathlib

from http.server import executable
from launch import LaunchDescription

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnExecutionComplete

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    isaac_mars_rover_description_path = os.path.join(
        get_package_share_directory('curiosity_rover_description'))

    xacro_file = os.path.join(isaac_mars_rover_description_path,
                              'models',
                              'curiosity_path',
                              'urdf',
                              'isaac_curiosity_mars_rover.xacro')
    urdf_path = os.path.join(isaac_mars_rover_description_path,
                              'models',
                              'curiosity_path',
                              'urdf',
                              'isaac_curiosity_mars_rover.urdf')
    
    # Load xacro
    doc = xacro.process_file(xacro_file)
    # Expand xacro and generate URDF
    robot_desc = doc.toprettyxml(indent='  ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    params = {'robot_description': robot_desc}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mars_rover_control"),
            "config",
            "mars_rover_control.yaml",
        ]
    )

    arm_node = Node(
        package="mars_rover_control",
        executable="move_arm",
        output='screen'
    )

    mast_node = Node(
        package="mars_rover_control",
        executable="move_mast",
        output='screen'
    )

    wheel_node = Node(
        package="mars_rover_control",
        executable="move_wheel",
        output='screen'
    )
    
    run_node = Node(
        package="mars_rover_control",
        executable="run_demo",
        output='screen'
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    load_joint_state_broadcaster = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    output="screen"
    )

    load_arm_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    load_mast_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mast_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    load_wheel_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    load_steer_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steer_position_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    load_suspension_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_tree_position_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
        
    return LaunchDescription([
        control_node,
        load_joint_state_broadcaster,
        load_mast_joint_traj_controller,
        load_wheel_joint_traj_controller,
        load_steer_joint_traj_controller,
        load_suspension_joint_traj_controller,
        load_arm_joint_traj_controller,
        arm_node,
        mast_node,
        wheel_node,
        run_node
    ])
