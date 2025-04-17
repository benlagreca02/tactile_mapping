import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None




def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="my_robot_cell", package_name="my_robot_cell_moveit_config")
        .robot_description() # had filename to URDF
        .to_moveit_configs()  # makes it return a moveit config
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("servoing", "config/arm_servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )


    my_node = Node(
        package="scanning",
        executable="fake_freedrive",
        name="fake_freedrive",
        output="screen"
    )

    return LaunchDescription([
        rsp,
        servo_node,
        my_node
    ])

