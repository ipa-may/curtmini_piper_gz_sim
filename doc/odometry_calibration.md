# Wheel odometry calibration (simulation)

This note explains how the `base_controller` odometry parameters in
[`config/simulation_controllers.yaml`](../config/simulation_controllers.yaml)
were measured against Gazebo ground truth, and how to repeat the measurement.

## Problem

While Nav2 was running, the local costmap (global frame `odom`) smeared obstacles
whenever the robot turned, and the `odom` frame drifted visibly against `map`.

The base has four fixed wheels and is driven by
`diff_drive_controller/DiffDriveController`, which estimates yaw rate as

```
omega = (v_right - v_left) / (wheel_separation * wheel_separation_multiplier)
```

A skid-steer base has to scrub its wheels sideways to turn, so it rotates less
than the wheel speeds suggest. It behaves like a differential drive with a wider
*effective* track. The previous configuration also used a separation
(`0.44 m`) smaller than the geometric track (`0.516 m`, wheel joints at
`y = ±0.258 m` in `curt_mini_description`). Together, these made odometry
overestimate every rotation.

## Ground-truth source

Gazebo's `OdometryPublisher` system publishes the model's true pose from the
physics engine. It is attached in
[`urdf/curtmini_piper_gz.urdf.xacro`](../urdf/curtmini_piper_gz.urdf.xacro):

| Setting | Value |
|---|---|
| Gazebo topic | `/ground_truth/odom` (`gz.msgs.Odometry`, 50 Hz sim time) |
| ROS topic | `/ground_truth/odom` (`nav_msgs/msg/Odometry`), via [`config/gz_bridge.yaml`](../config/gz_bridge.yaml) |
| Frames | `world` → `base_link` (Gazebo world, **not** `map` or `odom`) |
| TF | Published on `/ground_truth/tf`, which is deliberately **not** bridged, so it cannot conflict with `odom → base_link` |

The pose is in Gazebo's world frame, so the comparison uses heading change and
path length, never absolute positions.

## Procedure

All commands run from `curtmini_piper_containers` with the workspace overlay, so
the containers use the local sources:

```bash
export COMPOSE_FILE=compose.yaml:compose.cyclonedds.yaml:compose.workspace.yaml:compose.gui.yaml
docker compose run --rm workspace-builder          # after every source edit
SIM_WORLD=curtmini_piper HEADLESS=true SIM_USE_CAMERAS=false docker compose up gz-sim
```

- **`SIM_WORLD=curtmini_piper`:** an empty world (ground plane only), so the
  robot cannot hit walls during the straight run.
- **Nav2 is not started.** Then nothing else publishes on
  `/base_controller/cmd_vel` during the test.
- **Headless, no cameras:** raises the real-time factor. It does not affect the
  result, because both odometry sources use sim time.

`docker compose exec` skips the image entrypoint, so prefix each command with
`/opt/container/ros_entrypoint.sh` to load the ROS environment.

1. Check that ground truth is available:
   ```bash
   docker compose exec gz-sim /opt/container/ros_entrypoint.sh gz topic -l | grep ground_truth
   docker compose exec gz-sim /opt/container/ros_entrypoint.sh ros2 topic hz /ground_truth/odom
   ```
2. Start the comparison script ([appendix](#appendix-comparison-script)). It
   integrates the unwrapped yaw change and the 2D path length of both topics and
   prints `odom / ground truth` once per sim second. The argument is the run
   duration in sim seconds:
   ```bash
   docker compose exec -T gz-sim /opt/container/ros_entrypoint.sh python3 - 300 < odom_check.py
   ```
3. **Idle check:** send no commands for about 20 s. Neither distance should grow.
4. **Spin test:** turn in place for at least two full turns:
   ```bash
   docker compose exec gz-sim /opt/container/ros_entrypoint.sh ros2 topic pub -r 20 \
     /base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {angular: {z: 0.5}}}"
   ```
5. **Straight test:** restart the script, then drive for about 3 m:
   ```bash
   docker compose exec gz-sim /opt/container/ros_entrypoint.sh timeout 10 ros2 topic pub -r 20 \
     /base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}}}"
   ```

## Results (2026-11-11, before calibration)

The configuration under test was `wheel_separation: 0.44`,
`wheel_separation_multiplier: 1.0` and `wheel_radius: 0.13`.

| Test | Ground truth | Odometry | Ratio odom / gt |
|---|---|---|---|
| Spin in place, `ω_cmd = 0.5 rad/s` | 654.2° | 856.3° | **1.309** |
| Straight, `v_cmd = 0.3 m/s` | 2.82 m | 2.82 m | **1.000** |

- **Rotation:** the ratio stayed between 1.309 and 1.310 throughout two
  independent runs. Odometry reported exactly the commanded 0.5 rad/s, while
  the base actually turned at about 0.38 rad/s.
- **Translation:** the wheel radius is correct, so the radius multipliers stay
  at 1.0.
- **Idle:** in the empty world without Nav2, ground truth and odometry agreed to
  the centimetre, so Gazebo itself does not make the robot creep. An earlier run
  with Nav2 active showed slow motion while "idle". That came from velocity
  commands reaching the controller, not from odometry.

## Derived parameters

To make odometry match ground truth, the effective separation has to be scaled
by the measured yaw ratio:

```
effective separation = 0.44 m × 1.309 = 0.576 m
multiplier          = 0.576 m / 0.516 m = 1.127
```

The geometry and the slip correction are kept as separate parameters:

```yaml
wheel_separation: 0.516              # geometric track
wheel_radius: 0.13
wheel_separation_multiplier: 1.127   # skid-steer slip
```

## Verifying the calibration

Repeat the spin test after rebuilding the workspace and restarting `gz-sim`. The
yaw ratio should be about 1.00, and the straight ratio should still be 1.00.

## Limitations

- **Turn radius:** skid-steer slip depends on turn radius, speed and
  acceleration. A pure spin is the worst case, so on gentle arcs this
  correction slightly overcorrects. Run the same script with combined
  `linear.x` and `angular.z` commands to check typical Nav2 motions.
- **Real robot:** the real robot
  (`curt_mini_bringup/config/ros2_control.yaml`) still uses `0.44`. Real tyres
  and floors slip differently from Gazebo's contact model (`mu1=20`,
  `mu2=0.6`), so it needs its own measurement, for example against a motion
  capture system or a lidar scan-matching reference.
- **More robust option:** for robustness across all manoeuvres, fuse the IMU
  yaw rate with the wheel odometry's forward speed in a `robot_localization`
  EKF, and let the EKF publish `odom → base_link`.

## Appendix: comparison script

```python
"""Compare wheel odometry against Gazebo ground truth.

Accumulates unwrapped yaw change and 2D path length for both topics,
starting from the first message of each, and prints a running comparison.
Usage: docker compose exec -T gz-sim /opt/container/ros_entrypoint.sh \
         python3 - [duration_sim_s] < odom_check.py
"""
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

GT_TOPIC = '/ground_truth/odom'
ODOM_TOPIC = '/base_controller/odom'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Track:
    def __init__(self):
        self.last_yaw = None
        self.last_xy = None
        self.yaw = 0.0   # signed, unwrapped
        self.dist = 0.0  # path length
        self.count = 0

    def update(self, msg):
        p = msg.pose.pose.position
        y = yaw_of(msg.pose.pose.orientation)
        if self.last_yaw is not None:
            self.yaw += wrap(y - self.last_yaw)
            self.dist += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_yaw = y
        self.last_xy = (p.x, p.y)
        self.count += 1


class OdomCheck(Node):
    def __init__(self, duration):
        super().__init__('odom_check', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.duration = duration
        self.gt = Track()
        self.odom = Track()
        self.start = None
        self.done = False
        self.create_subscription(Odometry, GT_TOPIC, self.gt.update, 50)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom.update, 50)
        self.create_timer(1.0, self.report)

    @staticmethod
    def ratio(a, b, min_b):
        return f'{a / b:6.3f}' if abs(b) >= min_b else '   -  '

    def report(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start = now
        g, o = self.gt, self.odom
        if g.count == 0 or o.count == 0:
            print(f'waiting for messages: {GT_TOPIC}={g.count} {ODOM_TOPIC}={o.count}',
                  flush=True)
            return
        gy, oy = math.degrees(g.yaw), math.degrees(o.yaw)
        print(f't={now - self.start:6.1f}s | '
              f'yaw gt={gy:8.1f}° odom={oy:8.1f}° ratio={self.ratio(oy, gy, 10.0)} | '
              f'dist gt={g.dist:6.2f}m odom={o.dist:6.2f}m '
              f'ratio={self.ratio(o.dist, g.dist, 0.2)}', flush=True)
        if now - self.start >= self.duration:
            self.done = True


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    node = OdomCheck(duration)
    print(f'Comparing {ODOM_TOPIC} against {GT_TOPIC} for {duration:.0f} sim-s. '
          'Ratio = odom / ground truth.', flush=True)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    print('--- final ---', flush=True)
    node.report()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```
