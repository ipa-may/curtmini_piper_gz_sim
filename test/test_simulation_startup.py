"""Check the installed simulation's readiness within 80 wall-clock seconds."""

import atexit
import os
from pathlib import Path
import re
import subprocess
import time
import unittest
import uuid

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
import domain_coordinator
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


def generate_test_description():
    # Isolate both ROS and Gazebo from any interactive simulation on this host.
    domain = domain_coordinator.domain_id()
    os.environ['ROS_DOMAIN_ID'] = str(domain.__enter__())
    atexit.register(domain.__exit__, None, None, None)
    os.environ['GZ_PARTITION'] = 'startup_test_' + uuid.uuid4().hex
    # rclpy may already have loaded an RMW before this function is called.
    # Launch child processes with that same implementation.
    os.environ['RMW_IMPLEMENTATION'] = rclpy.get_rmw_implementation_identifier()
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
                'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(key, None)
    # Gazebo's sensors system can initialize rendering even without a GUI.
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    share = Path(get_package_share_directory('curtmini_piper_gz_sim'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/simulation.launch.py')),
            launch_arguments={
                'gui': 'false', 'use_rviz': 'false', 'paused': 'false',
                'start_joystick': 'false',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {'deadline': time.monotonic() + 80.0}


class TestSimulationStartup(unittest.TestCase):
    def test_readiness(self, deadline):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = rclpy.create_node('simulation_startup_test')
        self.addCleanup(node.destroy_node)
        clocks = []
        joints = set()

        def clock_received(msg):
            value = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
            if not clocks:
                clocks.append(value)
            elif value > clocks[0]:
                clocks[:] = [clocks[0], value]

        def joints_received(msg):
            joints.clear()
            joints.update(msg.name)

        node.create_subscription(Clock, '/clock', clock_received, qos_profile_sensor_data)
        node.create_subscription(
            JointState, '/joint_states', joints_received, qos_profile_sensor_data)
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        expected_joints = {
            'front_left_motor', 'back_left_motor',
            'front_right_motor', 'back_right_motor',
            *(f'piper_joint{i}' for i in range(1, 7)),
        }
        expected_controllers = {
            'joint_state_broadcaster', 'base_controller', 'arm_controller',
        }
        active = set()
        future = None
        request_started = 0.0
        controller_status = 'service not discovered'
        model_found = False
        model_output = 'Gazebo model list not received'
        next_model_check = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            if future is not None and future.done():
                response = future.result()
                active = {c.name for c in response.controller if c.state == 'active'}
                controller_status = str({c.name: c.state for c in response.controller})
                future = None
            if future is not None and time.monotonic() - request_started > 2.0:
                client.remove_pending_request(future)
                future.cancel()
                future = None
                controller_status = 'service response timed out; retrying'
            if future is None and client.service_is_ready():
                future = client.call_async(ListControllers.Request())
                request_started = time.monotonic()

            if not model_found and time.monotonic() >= next_model_check:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    result = subprocess.run(
                        ['gz', 'model', '--list'], capture_output=True, text=True,
                        timeout=min(2.0, remaining), check=False)
                    model_output = result.stdout + result.stderr
                    model_found = result.returncode == 0 and bool(re.search(
                        r'^\s*-\s+curtmini_piper\s*$', result.stdout, re.MULTILINE))
                except subprocess.TimeoutExpired:
                    model_output = 'gz model --list timed out'
                next_model_check = time.monotonic() + 2.0

            if (model_found and len(clocks) == 2
                    and expected_joints <= joints and expected_controllers <= active):
                return

        self.fail(
            'Simulation was not ready within 80 seconds:\n'
            f'  robot present in Gazebo: {model_found}\n'
            f'  simulation clock advanced: {len(clocks) == 2}\n'
            f'  missing joints: {sorted(expected_joints - joints)}\n'
            f'  inactive/missing controllers: {sorted(expected_controllers - active)}\n'
            f'  last controller query: {controller_status}\n'
            f'  last Gazebo query: {model_output[-2000:]}')
