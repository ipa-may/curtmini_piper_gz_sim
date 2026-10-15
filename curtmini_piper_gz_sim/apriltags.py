from pathlib import Path
from xml.sax.saxutils import escape
import xml.etree.ElementTree as ET

from launch_ros.actions import Node
import yaml

from .validation import finite_number, require_keys


TAG_IDS = frozenset((0, 1, 2))


def load_apriltags(path):
    layout_path = Path(path).expanduser().resolve()
    if not layout_path.is_file():
        raise RuntimeError(f'AprilTag layout file does not exist: {layout_path}')
    with layout_path.open(encoding='utf-8') as stream:
        layout = yaml.safe_load(stream)
    require_keys(layout, ('tags',), str(layout_path))
    if not isinstance(layout['tags'], list):
        raise RuntimeError('AprilTag layout tags must be a list')
    tags = []
    seen = set()
    for index, tag in enumerate(layout['tags']):
        label = f'tags[{index}]'
        require_keys(tag, ('id', 'size', 'pose'), label)
        tag_id = tag['id']
        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            raise RuntimeError(f'{label}.id must be an integer')
        if tag_id not in TAG_IDS:
            raise RuntimeError(f'{label}.id={tag_id} has no packaged texture; use 0, 1, or 2')
        if tag_id in seen:
            raise RuntimeError(f'AprilTag id {tag_id} appears more than once')
        seen.add(tag_id)
        size = finite_number(tag['size'], f'{label}.size')
        if size <= 0:
            raise RuntimeError(f'{label}.size must be positive')
        require_keys(tag['pose'], ('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
                     f'{label}.pose')
        pose = {
            axis: finite_number(tag['pose'][axis], f'{label}.pose.{axis}')
            for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
        }
        tags.append({'id': tag_id, 'size': size, 'pose': pose})
    return tags


def apriltag_model_sdf(tag, texture):
    size = tag['size']
    # The source bitmap is 10x10 cells: eight black-square cells plus one
    # white margin cell on each side. 'size' names the black-square edge.
    texture_size = size * 10 / 8
    panel_size = size * 1.35
    # The +X face has the tag; a white backing extends its quiet zone.
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="apriltag_36h11_{tag['id']}">
    <static>true</static>
    <link name="panel">
      <collision name="panel_collision">
        <geometry><box><size>0.006 {panel_size} {panel_size}</size></box></geometry>
      </collision>
      <visual name="white_backing">
        <geometry><box><size>0.006 {panel_size} {panel_size}</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
      </visual>
      <visual name="tag">
        <pose>0.004 0 0 0 0 0</pose>
        <geometry><box><size>0.001 {texture_size} {texture_size}</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>{escape(str(texture))}</albedo_map>
            <metalness>0</metalness><roughness>1</roughness></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''


def apriltag_actions(world_file, sim_share, layout_file):
    root = ET.parse(world_file).getroot()
    world = root.find('world')
    if world is None or not world.get('name'):
        raise RuntimeError(f'Gazebo world has no name: {world_file}')
    actions = []
    for tag in load_apriltags(layout_file):
        texture = sim_share / 'models' / 'apriltags' / f"tag36_11_{tag['id']:05d}.png"
        if not texture.is_file():
            raise RuntimeError(f'AprilTag texture is missing: {texture}')
        pose = tag['pose']
        actions.append(Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-world', world.get('name'),
                '-name', f"apriltag_36h11_{tag['id']}",
                '-string', apriltag_model_sdf(
                    tag, f'model://apriltags/{texture.name}'
                ),
                '-x', str(pose['x']), '-y', str(pose['y']), '-z', str(pose['z']),
                '-R', str(pose['roll']), '-P', str(pose['pitch']),
                '-Y', str(pose['yaw']),
            ],
        ))
    return actions
