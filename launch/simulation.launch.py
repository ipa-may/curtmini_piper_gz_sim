import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
import yaml


ARM_PREFIX = 'piper_'
ARM_JOINTS = tuple(f'{ARM_PREFIX}joint{i}' for i in range(1, 7))


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _resolve_world(value, sim_share):
    requested = Path(value).expanduser()

    if requested.is_absolute() or requested.parent != Path('.'):
        resolved = requested.resolve()
        if resolved.is_file():
            return str(resolved)
        raise RuntimeError(f'Gazebo world file does not exist: {resolved}')

    filename = requested
    if not filename.suffix:
        filename = filename.with_suffix('.sdf')
    resolved = sim_share / 'worlds' / filename
    if resolved.is_file():
        return str(resolved)

    available = ', '.join(
        path.stem for path in sorted((sim_share / 'worlds').glob('*.sdf'))
    )
    raise RuntimeError(
        f'Unknown packaged world {value!r}. Available worlds: {available}'
    )


def _require_keys(value, expected, label):
    if not isinstance(value, dict):
        raise RuntimeError(f'{label} must be a YAML mapping')
    missing = set(expected) - set(value)
    extra = set(value) - set(expected)
    if missing or extra:
        raise RuntimeError(
            f'{label} has missing keys {sorted(missing)} and '
            f'unknown keys {sorted(extra)}'
        )


def _finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f'{label} must be a number')
    if not math.isfinite(value):
        raise RuntimeError(f'{label} must be finite')
    return float(value)


def _load_initial_state(path):
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f'Initial state file does not exist: {config_path}')
    with config_path.open(encoding='utf-8') as stream:
        state = yaml.safe_load(stream)
    _require_keys(state, ('base', 'arm'), str(config_path))
    _require_keys(state['base'], ('x', 'y', 'z', 'orientation'), 'base')
    _require_keys(state['arm'], ARM_JOINTS, 'arm')

    base = {
        name: _finite_number(state['base'][name], f'base.{name}')
        for name in ('x', 'y', 'z', 'orientation')
    }
    limits_path = (
        Path(get_package_share_directory('agx_arm_urdf'))
        / 'piper' / 'config' / 'joint_position_limits.yaml'
    )
    with limits_path.open(encoding='utf-8') as stream:
        limits = yaml.safe_load(stream)['joint_limits']
    arm = {}
    for index, joint in enumerate(ARM_JOINTS, start=1):
        value = _finite_number(state['arm'][joint], f'arm.{joint}')
        bounds = limits[f'joint{index}']
        if not bounds['lower'] <= value <= bounds['upper']:
            raise RuntimeError(
                f'arm.{joint}={value} is outside the robot limits '
                f"[{bounds['lower']}, {bounds['upper']}]"
            )
        arm[joint] = value
    return {'base': base, 'arm': arm}


def _build_descriptions(context, sim_share, initial_state):
    description_share = Path(
        get_package_share_directory('curtmini_piper_description')
    )
    mappings = {
        'simulation': 'True',
        'use_sim_time': 'True',
        'use_simplified_collision': 'True',
        'use_hokuyo': (
            'True' if _as_bool(context, 'use_hokuyo') else 'False'
        ),
        'gazebo_controllers': str(
            sim_share / 'config' / 'simulation_controllers.yaml'
        ),
        'arm_prefix': ARM_PREFIX,
    }

    simulation_mappings = dict(mappings)
    simulation_mappings.update({
        f'{joint}_initial_value': str(value)
        for joint, value in initial_state['arm'].items()
    })
    full_description = xacro.process_file(
        str(
            sim_share
            / 'urdf'
            / 'curtmini_piper_gz.urdf.xacro'
        ),
        mappings=simulation_mappings,
    ).toxml()

    moveit_mappings = dict(mappings)
    moveit_mappings['simulation'] = 'False'
    moveit_config = (
        MoveItConfigsBuilder(
            'curtmini_piper',
            package_name='curtmini_piper_moveit_config',
        )
        .robot_description(
            file_path=str(
                description_share
                / 'urdf'
                / 'curtmini_piper_moveit.urdf.xacro'
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
                sim_share / 'config' / 'moveit_controllers.yaml'
            )
        )
        .to_moveit_configs()
    )
    moveit_config.sensors_3d = {}
    return full_description, moveit_config


def _controller_spawner(name):
    return Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            name,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
    )


def _launch_setup(context):
    sim_share = Path(
        get_package_share_directory('curtmini_piper_gz_sim')
    )
    moveit_share = Path(
        get_package_share_directory('curtmini_piper_moveit_config')
    )
    curt_share = Path(get_package_share_directory('curt_mini_description'))
    agx_urdf_share = Path(
        get_package_share_directory('agx_arm_urdf')
    )
    neo_worlds_share = Path(
        get_package_share_directory('neo_gz_worlds')
    )
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))

    initial_state = _load_initial_state(
        LaunchConfiguration('initial_state_file').perform(context)
    )
    full_description, moveit_config = _build_descriptions(
        context, sim_share, initial_state
    )

    world = _resolve_world(
        LaunchConfiguration('world').perform(context), sim_share
    )
    gz_args = []
    if not _as_bool(context, 'paused'):
        gz_args.append('-r')
    if not _as_bool(context, 'gui'):
        gz_args.append('-s')
    gz_args.append(world)

    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
        'use_sim_time': True,
    }

    actions = [
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(curt_share.parent),
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(agx_urdf_share.parent),
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(neo_worlds_share / 'models'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_gz_share / 'launch' / 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ' '.join(gz_args),
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': full_description,
                    'use_sim_time': True,
                }
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[
                {
                    'config_file': str(
                        sim_share / 'config' / 'gz_bridge.yaml'
                    ),
                    'use_sim_time': True,
                }
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name',
                'curtmini_piper',
                '-topic',
                'robot_description',
                '-x',
                str(initial_state['base']['x']),
                '-y',
                str(initial_state['base']['y']),
                '-z',
                str(initial_state['base']['z']),
                '-Y',
                str(initial_state['base']['orientation']),
            ],
        ),
        _controller_spawner('joint_state_broadcaster'),
        _controller_spawner('base_controller'),
        _controller_spawner('arm_controller'),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                move_group_configuration,
            ],
            remappings=[('joint_states', '/joint_states')],
            additional_env={
                'DISPLAY': os.environ.get('DISPLAY', '')
            },
        ),
    ]

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
        actions.append(
            Node(
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
        )

    return actions


def generate_launch_description():
    sim_share = Path(
        get_package_share_directory('curtmini_piper_gz_sim')
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value='curtmini_piper',
                description=(
                    'Packaged world name, with optional .sdf extension, or '
                    'a path to an external Gazebo world file.'
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
