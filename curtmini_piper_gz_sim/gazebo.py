from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def resolve_world(value, sim_share, neo_worlds_share):
    requested = Path(value).expanduser()

    if requested.is_absolute() or requested.parent != Path('.'):
        resolved = requested.resolve()
        if resolved.is_file():
            return str(resolved)
        raise RuntimeError(f'Gazebo world file does not exist: {resolved}')

    filename = requested
    if not filename.suffix:
        filename = filename.with_suffix('.sdf')
    world_dirs = (sim_share / 'worlds', neo_worlds_share / 'worlds')
    for world_dir in world_dirs:
        resolved = world_dir / filename
        if resolved.is_file():
            return str(resolved)

    available = ', '.join(
        sorted({
            path.stem
            for world_dir in world_dirs
            for path in world_dir.glob('*.sdf')
        })
    )
    raise RuntimeError(
        f'Unknown packaged world {value!r}. Available worlds: {available}'
    )


def gazebo_start_actions(sim_share, neo_worlds_share, world, paused, gui):
    curt_share = Path(get_package_share_directory('curt_mini_description'))
    agx_urdf_share = Path(get_package_share_directory('agx_arm_urdf'))
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))

    gz_args = []
    if not paused:
        gz_args.append('-r')
    if not gui:
        gz_args.append('-s')
    gz_args.append(world)

    return [
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
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(sim_share / 'models'),
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
    ]


def bridge_action(sim_share):
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': str(sim_share / 'config' / 'gz_bridge.yaml'),
                'use_sim_time': True,
            }
        ],
    )
