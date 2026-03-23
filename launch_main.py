import os
import sys
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

sys.path.append('/home/carm/dev_ws/src/xarm_ros2/xarm_moveit_config/launch/lib')
from robot_moveit_config_lib import get_xarm_robot_description_parameters

def load_yaml(package_name, *paths):
    pkg_share = get_package_share_directory(package_name)
    filepath = os.path.join(pkg_share, *paths)
    with open(filepath, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    moveit_args = get_xarm_robot_description_parameters(
        urdf_arguments={'dof': '6', 'add_gripper': 'true', 'prefix': ''},
        srdf_arguments={'dof': '6', 'add_gripper': 'true', 'prefix': ''},
        arguments={'xarm_type': 'xarm'}
    )

    ompl_planning_yaml = load_yaml('xarm_moveit_config', 'config', 'xarm6', 'ompl_planning.yaml')
    joint_limits_yaml = load_yaml('xarm_moveit_config', 'config', 'xarm6', 'joint_limits.yaml')
    
    # The parameters list will include structural dictionaries directly compatible with ROS2
    params = [
        moveit_args,
        ompl_planning_yaml,
        {'robot_description_planning': joint_limits_yaml},
        {'use_sim_time': False}
    ]

    main_node = Node(
        name='moveit_py',
        executable=sys.executable,
        arguments=['/home/carm/carm-project/main/main.py'],
        output='screen',
        parameters=params,
        # Execute it in the workspace directory so relative paths work if needed
        cwd='/home/carm/carm-project'
    )

    return LaunchDescription([main_node])
