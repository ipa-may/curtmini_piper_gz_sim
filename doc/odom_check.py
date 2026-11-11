"""Compare wheel odometry against Gazebo ground truth.

Accumulates unwrapped yaw change and 2D path length for both topics,
starting from the first message of each, and prints a running comparison.

Usage (from curtmini_piper_containers):
  docker compose exec -T gz-sim /opt/container/ros_entrypoint.sh \
    python3 - [duration_sim_s] \
    < ../rob4_fraunhofer_ws/src/curtmini_piper_simulation/curtmini_piper_gz_sim/doc/odom_check.py

See odometry_calibration.md in this folder for the test procedure.

Default duration: 120 s of simulation time, then prints a final summary
and exits (Ctrl+C on a piped `exec -T` may not reach the container).
"""
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

GT_TOPIC = '/ground_truth/odom'
ODOM_TOPIC = '/base_controller/odom'
ICR_MIN_RATE = 0.1  # rad/s; below this the ICR is ill-defined (straight driving)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Track:
    def __init__(self):
        self.last_yaw = None
        self.last_xy = None
        self.start_xy = None
        self.yaw = 0.0      # signed, unwrapped
        self.dist = 0.0     # path length
        self.net = 0.0      # straight-line distance from start
        self.max_net = 0.0  # pivoting at radius r about a point gives max_net ~ 2r
        self.count = 0
        self.last_t = None
        # Instantaneous centre of rotation in the base frame, from pose deltas:
        # v = w x (0 - c)  =>  c_x = -v_y / w,  c_y = v_x / w
        self.icr = [0, 0.0, 0.0, 0.0, 0.0]  # n, sum x, sum y, sum x^2, sum y^2

    def update(self, msg):
        p = msg.pose.pose.position
        y = yaw_of(msg.pose.pose.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_xy is None:
            self.start_xy = (p.x, p.y)
        self.net = math.hypot(p.x - self.start_xy[0], p.y - self.start_xy[1])
        self.max_net = max(self.max_net, self.net)
        if self.last_yaw is not None:
            dyaw = wrap(y - self.last_yaw)
            dx, dy = p.x - self.last_xy[0], p.y - self.last_xy[1]
            self.yaw += dyaw
            self.dist += math.hypot(dx, dy)
            dt = t - self.last_t
            if dt > 0.0 and abs(dyaw / dt) > ICR_MIN_RATE:
                w = dyaw / dt
                ym = self.last_yaw + 0.5 * dyaw  # mid-step heading
                vx = (math.cos(ym) * dx + math.sin(ym) * dy) / dt
                vy = (-math.sin(ym) * dx + math.cos(ym) * dy) / dt
                cx, cy = -vy / w, vx / w
                s = self.icr
                s[0] += 1
                s[1] += cx
                s[2] += cy
                s[3] += cx * cx
                s[4] += cy * cy
        self.last_yaw = y
        self.last_xy = (p.x, p.y)
        self.last_t = t
        self.count += 1

    def icr_text(self):
        n, sx, sy, sxx, syy = self.icr
        if n == 0:
            return 'icr   -  '
        mx, my = sx / n, sy / n
        dx = math.sqrt(max(sxx / n - mx * mx, 0.0))
        dy = math.sqrt(max(syy / n - my * my, 0.0))
        return f'icr x={mx:+5.3f}±{dx:4.3f}m y={my:+5.3f}±{dy:4.3f}m (n={n})'


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
              f'ratio={self.ratio(o.dist, g.dist, 0.2)} | '
              f'net gt={g.net:5.2f}m (max {g.max_net:5.2f}m) odom={o.net:5.2f}m | '
              f'gt {g.icr_text()}',
              flush=True)
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
