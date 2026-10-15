from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro
import yaml

from .validation import finite_number, require_keys


ARM_PREFIX = 'piper_'
ARM_JOINTS = tuple(f'{ARM_PREFIX}joint{i}' for i in range(1, 7))


def load_initial_state(path):
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f'Initial state file does not exist: {config_path}')
    with config_path.open(encoding='utf-8') as stream:
        state = yaml.safe_load(stream)
    require_keys(state, ('base', 'arm'), str(config_path))
    require_keys(state['base'], ('x', 'y', 'z', 'orientation'), 'base')
    require_keys(state['arm'], ARM_JOINTS, 'arm')

    base = {
        name: finite_number(state['base'][name], f'base.{name}')
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
        value = finite_number(state['arm'][joint], f'arm.{joint}')
        bounds = limits[f'joint{index}']
        if not bounds['lower'] <= value <= bounds['upper']:
            raise RuntimeError(
                f'arm.{joint}={value} is outside the robot limits '
                f"[{bounds['lower']}, {bounds['upper']}]"
            )
        arm[joint] = value
    return {'base': base, 'arm': arm}


def robot_mappings(sim_share, use_hokuyo, use_cameras):
    return {
        'simulation': 'True',
        'use_sim_time': 'True',
        'use_simplified_collision': 'True',
        'use_hokuyo': 'True' if use_hokuyo else 'False',
        'use_cameras': 'True' if use_cameras else 'False',
        'gazebo_controllers': str(
            sim_share / 'config' / 'simulation_controllers.yaml'
        ),
        'arm_prefix': ARM_PREFIX,
    }


def simulation_description(sim_share, mappings, initial_state):
    simulation_mappings = dict(mappings)
    simulation_mappings.update({
        f'{joint}_initial_value': str(value)
        for joint, value in initial_state['arm'].items()
    })
    return xacro.process_file(
        str(sim_share / 'urdf' / 'curtmini_piper_gz.urdf.xacro'),
        mappings=simulation_mappings,
    ).toxml()


def robot_state_publisher_action(full_description):
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': full_description,
                'use_sim_time': True,
            }
        ],
    )


def robot_spawn_action(initial_state):
    return Node(
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
    )


def controller_spawner_action(*controller_names):
    return Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            *controller_names,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
            '--switch-timeout',
            '30',
            '--service-call-timeout',
            '40',
        ],
    )
