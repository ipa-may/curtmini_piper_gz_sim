from pathlib import Path
from xml.etree import ElementTree

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
    arm = {}
    for joint in ARM_JOINTS:
        arm[joint] = finite_number(state['arm'][joint], f'arm.{joint}')
    return {'base': base, 'arm': arm}


def robot_mappings(sim_share, arm_model, use_hokuyo, use_cameras):
    return {
        'simulation': 'True',
        'arm_model': arm_model,
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
    description = xacro.process_file(
        str(sim_share / 'urdf' / 'curtmini_piper_gz.urdf.xacro'),
        mappings=simulation_mappings,
    ).toxml()
    robot = ElementTree.fromstring(description)
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    for name, value in initial_state['arm'].items():
        joint = joints.get(name)
        limit = joint.find('limit') if joint is not None else None
        if limit is None:
            raise RuntimeError(f'No position limits found for {name}')
        lower = float(limit.get('lower'))
        upper = float(limit.get('upper'))
        if not lower <= value <= upper:
            raise RuntimeError(
                f'arm.{name}={value} is outside the robot limits '
                f'[{lower}, {upper}]'
            )
    return description


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
