from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config(mappings):
    description_share = Path(
        get_package_share_directory('curtmini_piper_description')
    )
    moveit_mappings = dict(mappings)
    moveit_mappings['simulation'] = 'False'
    moveit_share = Path(
        get_package_share_directory('curtmini_piper_moveit_config')
    )
    moveit_config = (
        MoveItConfigsBuilder(
            'curtmini_piper',
            package_name='curtmini_piper_moveit_config',
        )
        .robot_description(
            file_path=str(
                description_share
                / 'urdf'
                / 'curtmini_piper.urdf.xacro'
            ),
            mappings=moveit_mappings,
        )
        .robot_description_semantic(
            file_path='config/curtmini_piper.srdf'
        )
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(
            file_path=str(
                moveit_share / 'config' / 'moveit_controllers_sim.yaml'
            )
        )
        .to_moveit_configs()
    )
    moveit_config.sensors_3d = {}
    return moveit_config


def rviz_action(moveit_config):
    moveit_share = Path(
        get_package_share_directory('curtmini_piper_moveit_config')
    )
    return Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=[
            '-d',
            str(moveit_share / 'config' / 'moveit.rviz'),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
        remappings=[('joint_states', '/joint_states')],
    )
