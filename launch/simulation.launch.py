from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

from curtmini_piper_gz_sim.apriltags import apriltag_actions
from curtmini_piper_gz_sim.cameras import camera_mount_actions
from curtmini_piper_gz_sim.gazebo import (
    bridge_action,
    gazebo_start_actions,
    resolve_world,
)
from curtmini_piper_gz_sim.moveit import (
    build_moveit_config,
    rviz_action,
)
from curtmini_piper_gz_sim.robot import (
    controller_spawner_action,
    load_initial_state,
    robot_mappings,
    robot_spawn_action,
    robot_state_publisher_action,
    simulation_description,
)


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _launch_setup(context):
    sim_share = Path(get_package_share_directory('curtmini_piper_gz_sim'))
    neo_worlds_share = Path(get_package_share_directory('neo_gz_worlds'))
    arm_model = LaunchConfiguration('arm_model').perform(context)

    initial_state = load_initial_state(
        LaunchConfiguration('initial_state_file').perform(context)
    )
    mappings = robot_mappings(
        sim_share,
        arm_model=arm_model,
        use_hokuyo=_as_bool(context, 'use_hokuyo'),
        use_cameras=_as_bool(context, 'use_cameras'),
    )
    full_description = simulation_description(
        sim_share, mappings, initial_state
    )
    moveit_config = build_moveit_config(mappings)
    world = resolve_world(
        LaunchConfiguration('world').perform(context),
        sim_share,
        neo_worlds_share,
    )

    actions = gazebo_start_actions(
        sim_share, neo_worlds_share, world,
        paused=_as_bool(context, 'paused'),
        gui=_as_bool(context, 'gui'),
    )
    actions.extend([
        robot_state_publisher_action(full_description),
        bridge_action(sim_share),
        robot_spawn_action(initial_state),
        controller_spawner_action(
            'joint_state_broadcaster',
            'base_controller',
            'arm_controller',
            'gripper_controller',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory('curtmini_piper_moveit_config'))
                    / 'launch' / 'move_group.launch.py'
                )
            ),
            launch_arguments={
                'controller_mode': 'sim',
                'arm_model': arm_model,
                'use_sim_time': 'true',
                'use_hokuyo': str(_as_bool(context, 'use_hokuyo')).lower(),
                'use_sensors_3d': 'false',
            }.items(),
        ),
    ])

    if _as_bool(context, 'use_cameras'):
        description_share = Path(
            get_package_share_directory('curtmini_piper_description')
        )
        actions.extend(camera_mount_actions(description_share))

    if _as_bool(context, 'use_apriltags'):
        actions.extend(apriltag_actions(
            world, sim_share,
            LaunchConfiguration('apriltag_layout_file').perform(context),
        ))

    if _as_bool(context, 'start_joystick'):
        actions.append(
            GroupAction(
                [
                    SetRemap(
                        src='/joy_teleop/cmd_vel',
                        dst='/base_controller/cmd_vel',
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(
                                Path(get_package_share_directory('curt_mini_teleop'))
                                / 'launch'
                                / 'joystick.launch.py'
                            )
                        )
                    ),
                ]
            )
        )

    if _as_bool(context, 'use_rviz'):
        actions.append(rviz_action(moveit_config))

    return actions


def generate_launch_description():
    sim_share = Path(
        get_package_share_directory('curtmini_piper_gz_sim')
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'arm_model',
                default_value='piper_x',
                choices=['piper_x', 'piper_l', 'piper_h'],
                description='Piper arm and AGX gripper model to simulate.',
            ),
            DeclareLaunchArgument(
                'world',
                default_value='curtmini_piper',
                description=(
                    'World name from this package or neo_gz_worlds, with '
                    'optional .sdf extension, or a path to a Gazebo world file.'
                ),
            ),
            DeclareLaunchArgument(
                'gui',
                default_value='true',
                choices=['true', 'false'],
                description='Start the Gazebo graphical client.',
            ),
            DeclareLaunchArgument(
                'paused',
                default_value='false',
                choices=['true', 'false'],
                description='Start Gazebo paused.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Start RViz with the MoveIt panel.',
            ),
            DeclareLaunchArgument(
                'start_joystick',
                default_value='false',
                choices=['true', 'false'],
                description='Start Curt Mini joystick teleoperation.',
            ),
            DeclareLaunchArgument(
                'use_hokuyo',
                default_value='true',
                choices=['true', 'false'],
                description='Add the Hokuyo UTM-30LX-EW simulation.',
            ),
            DeclareLaunchArgument(
                'use_cameras',
                default_value='true',
                choices=['true', 'false'],
                description='Add the two simulated D435i-like RGBD and IMU sensors.',
            ),
            DeclareLaunchArgument(
                'use_apriltags',
                default_value='false',
                choices=['true', 'false'],
                description='Spawn the AprilTag exercise panels.',
            ),
            DeclareLaunchArgument(
                'apriltag_layout_file',
                default_value=str(sim_share / 'config' / 'apriltags.yaml'),
                description='YAML file with AprilTag IDs, sizes, and world poses.',
            ),
            DeclareLaunchArgument(
                'initial_state_file',
                default_value=str(
                    sim_share / 'config' / 'initial_state.yaml'
                ),
                description=(
                    'YAML file with the base x/y/z/orientation and six Piper '
                    'joint positions used at simulation startup.'
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
