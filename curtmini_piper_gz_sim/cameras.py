import math

from launch_ros.actions import Node
import yaml

from .validation import finite_number


CAMERA_PARENTS = {
    'base_camera': 'chassis',
    'tool_camera': 'piper_tcp_link',
}


def camera_mount_actions(description_share):
    geometry_path = description_share / 'config' / 'geometry.yaml'
    with geometry_path.open(encoding='utf-8') as stream:
        geometry = yaml.safe_load(stream)

    actions = []
    for name, parent in CAMERA_PARENTS.items():
        mount = geometry[f'{name}_mount']
        position = mount['position']
        rotation = mount['rotation']
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{name}_mount_tf',
                arguments=[
                    '--x', str(finite_number(position['x'], f'{name}.x')),
                    '--y', str(finite_number(position['y'], f'{name}.y')),
                    '--z', str(finite_number(position['z'], f'{name}.z')),
                    '--roll', str(finite_number(rotation['r'], f'{name}.r')),
                    '--pitch', str(finite_number(rotation['p'], f'{name}.p')),
                    '--yaw', str(finite_number(rotation['y'], f'{name}.y')),
                    '--frame-id', parent,
                    '--child-frame-id', f'{name}_link',
                ],
                parameters=[{'use_sim_time': True}],
            )
        )
        # Gazebo renders along +X. ROS camera and IMU images use +Z optical.
        for suffix in ('color', 'imu'):
            actions.append(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'{name}_{suffix}_optical_tf',
                    arguments=[
                        '--x', '0', '--y', '0', '--z', '0',
                        '--roll', str(-math.pi / 2),
                        '--pitch', '0',
                        '--yaw', str(-math.pi / 2),
                        '--frame-id', f'{name}_link',
                        '--child-frame-id', f'{name}_{suffix}_optical_frame',
                    ],
                    parameters=[{'use_sim_time': True}],
                )
            )
    return actions
