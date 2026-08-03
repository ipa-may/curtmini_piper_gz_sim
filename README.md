# Curtmini Piper Gz Sim

## Gazebo simulation

Start Gazebo Harmonic, the simulated base and arm controllers, MoveIt, and
RViz:

```bash
ros2 launch curtmini_piper_gz_sim simulation.launch.py
```

The simulation uses one Gazebo-owned controller manager for both the Curt Mini
base and Piper arm. It publishes simulation time on `/clock`, IMU data on
`/imu/data`, wheel odometry on `/base_controller/odom`, and the combined robot
state on `/joint_states`.

Run without Gazebo and RViz windows for headless testing:

```bash
ros2 launch curtmini_piper_gz_sim simulation.launch.py \
  gui:=false use_rviz:=false
```

Joystick teleoperation is disabled by default. Enable it with
`start_joystick:=true`. Mount and TCP arguments are the same as the real robot
bringup.