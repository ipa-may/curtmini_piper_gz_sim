"""Static wheel loads of the simulated robot from its URDF.

Reads a URDF on stdin, poses the joints (arm at the sim initial state, others 0),
computes total mass and centre of mass in base_link, and splits the weight onto
the four wheel contact points with a rigid-chassis lever model.

Usage (from curtmini_piper_containers):
  docker compose exec -T gz-sim /opt/container/ros_entrypoint.sh bash -c \
    'ros2 param get --hide-type /robot_state_publisher robot_description' \
    | python3 -I ../rob4_fraunhofer_ws/src/curtmini_piper_simulation/curtmini_piper_gz_sim/doc/wheel_loads.py

Runs on the host; only the URDF comes from the container. Update JOINT_VALUES if
config/initial_state.yaml changes, then copy the loads into wheel_normal_force in
urdf/curtmini_piper_gz.urdf.xacro. See odometry_calibration.md in this folder.
"""
import math
import sys
import xml.etree.ElementTree as ET

G = 9.80665
BASE = 'base_link'
WHEELS = {  # wheel link -> motor joint
    'front_left_wheel': 'front_left_motor',
    'front_right_wheel': 'front_right_motor',
    'back_left_wheel': 'back_left_motor',
    'back_right_wheel': 'back_right_motor',
}
# curtmini_piper_gz_sim/config/initial_state.yaml
JOINT_VALUES = {
    'piper_joint1': 0.0, 'piper_joint2': 0.1, 'piper_joint3': -0.1,
    'piper_joint4': 0.0, 'piper_joint5': 0.0, 'piper_joint6': -1.95,
}


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0, 0, 0, 1],
    ]


def axis_angle(axis, q):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
        [0, 0, 0, 1],
    ]


def parse_origin(elem):
    o = elem.find('origin') if elem is not None else None
    if o is None:
        return transform()
    xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
    return transform(xyz, rpy)


def main():
    robot = ET.fromstring(sys.stdin.read())
    children = {}
    joints = {}
    for j in robot.findall('joint'):
        joints[j.get('name')] = j
        children.setdefault(j.find('parent').get('link'), []).append(j)

    poses = {BASE: transform()}
    stack = [BASE]
    while stack:
        parent = stack.pop()
        for j in children.get(parent, []):
            t = matmul(poses[parent], parse_origin(j))
            q = JOINT_VALUES.get(j.get('name'), 0.0)
            axis = [float(v) for v in (j.find('axis').get('xyz') if j.find('axis') is not None
                                       else '1 0 0').split()]
            if j.get('type') in ('revolute', 'continuous'):
                t = matmul(t, axis_angle(axis, q))
            elif j.get('type') == 'prismatic':
                t = matmul(t, transform([a * q for a in axis]))
            child = j.find('child').get('link')
            poses[child] = t
            stack.append(child)

    total, cx, cy, cz = 0.0, 0.0, 0.0, 0.0
    rows = []
    unreached = []
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None or inertial.find('mass') is None:
            continue
        name = link.get('name')
        m = float(inertial.find('mass').get('value'))
        if name not in poses:
            unreached.append(name)
            continue
        c = matmul(poses[name], parse_origin(inertial))
        x, y, z = c[0][3], c[1][3], c[2][3]
        total += m
        cx, cy, cz = cx + m * x, cy + m * y, cz + m * z
        rows.append((m, name, x, y, z))
    cx, cy, cz = cx / total, cy / total, cz / total

    print('Heaviest links (mass, CoM in base_link):')
    for m, name, x, y, z in sorted(rows, reverse=True)[:8]:
        print(f'  {m:7.3f} kg  {name:28s} x={x:+.3f} y={y:+.3f} z={z:+.3f}')
    if unreached:
        print(f'WARNING: links not connected to {BASE}: {unreached}')
    print(f'\nTotal mass {total:.3f} kg, weight {total * G:.1f} N')
    print(f'CoM in {BASE}: x={cx:+.4f} y={cy:+.4f} z={cz:+.4f} m')

    contact = {}
    for link, joint in WHEELS.items():
        t = poses[link]
        contact[link] = (t[0][3], t[1][3])
    x_f = (contact['front_left_wheel'][0] + contact['front_right_wheel'][0]) / 2
    x_r = (contact['back_left_wheel'][0] + contact['back_right_wheel'][0]) / 2
    y_l = (contact['front_left_wheel'][1] + contact['back_left_wheel'][1]) / 2
    y_r = (contact['front_right_wheel'][1] + contact['back_right_wheel'][1]) / 2
    f = (cx - x_r) / (x_f - x_r)
    left = (cy - y_r) / (y_l - y_r)
    w = total * G
    loads = {
        'front_left_wheel': w * f * left,
        'front_right_wheel': w * f * (1 - left),
        'back_left_wheel': w * (1 - f) * left,
        'back_right_wheel': w * (1 - f) * (1 - left),
    }
    print(f'\nAxles: front x={x_f:+.3f}, rear x={x_r:+.3f}; '
          f'front share {f * 100:.1f} %, left share {left * 100:.1f} %')
    print(f'Equal-load reference: {w / 4:.1f} N per wheel')
    for link, n in loads.items():
        print(f'  {link:18s} {n:6.1f} N  (x={contact[link][0]:+.3f}, y={contact[link][1]:+.3f})')


if __name__ == '__main__':
    main()
