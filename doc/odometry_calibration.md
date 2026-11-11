# Wheel contact model and odometry calibration (simulation)

This note explains how the simulated wheel contact model and the `base_controller`
odometry parameters were measured against Gazebo ground truth, and how to repeat the
measurements. Both live in this package, and the odometry calibration only holds for
the contact model it was measured with:

| What | Where | Current value |
|---|---|---|
| Wheel friction | [`urdf/curtmini_piper_gz.urdf.xacro`](../urdf/curtmini_piper_gz.urdf.xacro) | `mu1 = mu2 = 1.0`, `fdir1 = 0 0 1` (wheel axle) |
| Tyre slip compliance (WheelSlip) | same file | lateral `1.0`, longitudinal `0.05` |
| Wheel separation | [`config/simulation_controllers.yaml`](../config/simulation_controllers.yaml) | `0.516` m × multiplier `1.147` (effective `0.592` m) |
| Wheel radius | same file | `0.13` m, multipliers `1.0` |

## Problem

While Nav2 was running, the local costmap (global frame `odom`) smeared obstacles
whenever the robot turned, and the `odom` frame drifted visibly against `map`.

The base has four fixed wheels and is driven by
`diff_drive_controller/DiffDriveController`, which estimates yaw rate as

```
omega = (v_right - v_left) / (wheel_separation * wheel_separation_multiplier)
```

and assumes that the robot rotates about `base_link`. A skid-steer base has to scrub
its wheels sideways to turn, so it rotates less than the wheel speeds suggest. It
behaves like a differential drive with a wider *effective* track. The original
configuration also used a separation (`0.44 m`) smaller than the geometric track
(`0.516 m`, wheel joints at `y = ±0.258 m` in `curt_mini_description`). Together,
these made odometry overestimate every rotation.

## Ground-truth source

Gazebo's `OdometryPublisher` system publishes the model's true pose from the physics
engine. It is attached in
[`urdf/curtmini_piper_gz.urdf.xacro`](../urdf/curtmini_piper_gz.urdf.xacro):

| Setting | Value |
|---|---|
| Gazebo topic | `/ground_truth/odom` (`gz.msgs.Odometry`, 50 Hz sim time) |
| ROS topic | `/ground_truth/odom` (`nav_msgs/msg/Odometry`), via [`config/gz_bridge.yaml`](../config/gz_bridge.yaml) |
| Frames | `world` → `base_link` (Gazebo world, **not** `map` or `odom`) |
| TF | Published on `/ground_truth/tf`, which is deliberately **not** bridged, so it cannot conflict with `odom → base_link` |

The pose is in Gazebo's world frame, so the comparison uses heading change, path
length and displacement from the start, never absolute positions.

## Procedure

All commands run from `curtmini_piper_containers` with the workspace overlay, so the
containers use the local sources:

```bash
export COMPOSE_FILE=compose.yaml:compose.cyclonedds.yaml:compose.workspace.yaml:compose.gui.yaml
docker compose run --rm workspace-builder          # after Python/launch edits
SIM_WORLD=curtmini_piper HEADLESS=true SIM_USE_CAMERAS=false docker compose up -d gz-sim
```

- **`SIM_WORLD=curtmini_piper`:** an empty world (ground plane only), so the robot
  cannot hit walls during the straight run.
- **Nav2 is not started.** Then nothing else publishes on `/base_controller/cmd_vel`
  during the test.
- **Headless, no cameras:** raises the real-time factor (about 0.85 here). It does
  not affect the result, because both odometry sources use sim time.

`docker compose exec` skips the image entrypoint, so prefix each command with
`/opt/container/ros_entrypoint.sh` to load the ROS environment.

The `urdf/` folder, `config/simulation_controllers.yaml` and `config/gz_bridge.yaml`
are bind-mounted into `gz-sim` ([`docker/compose.yaml`](../docker/compose.yaml)). The
xacro is expanded at launch, so after editing any of them,
`docker compose up -d --force-recreate gz-sim` is enough; no workspace rebuild is
needed.

1. Check that ground truth is available:
   ```bash
   docker compose exec gz-sim /opt/container/ros_entrypoint.sh gz topic -l | grep ground_truth
   docker compose exec gz-sim /opt/container/ros_entrypoint.sh ros2 topic hz /ground_truth/odom
   ```
2. Start the comparison script [`odom_check.py`](odom_check.py). The argument is
   the run duration in sim seconds:
   ```bash
   docker compose exec -T gz-sim /opt/container/ros_entrypoint.sh python3 - 40 \
     < ../rob4_fraunhofer_ws/src/curtmini_piper_simulation/curtmini_piper_gz_sim/doc/odom_check.py
   ```
   Once per sim second it prints, from the first message onwards:
   - `yaw`: unwrapped heading change of ground truth and odometry, and their ratio;
   - `dist`: 2D path length of both, and their ratio;
   - `net`: current and maximum straight-line distance from the start;
   - `icr`: mean and standard deviation of the ground-truth instantaneous centre of
     rotation in `base_link`, `c_x = -v_y/ω` and `c_y = v_x/ω`, from pose
     differences while turning faster than 0.1 rad/s.
3. **Idle check:** send no commands for about 20 s. Neither distance should grow.
4. **Spin test:** turn in place for about two full turns:
   ```bash
   docker compose exec gz-sim /opt/container/ros_entrypoint.sh timeout 30 ros2 topic pub -r 20 \
     /base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {angular: {z: 0.5}}}"
   ```
5. **Straight test:** restart the script, then drive for about 3 m:
   ```bash
   docker compose exec gz-sim /opt/container/ros_entrypoint.sh timeout 12 ros2 topic pub -r 20 \
     /base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}}}"
   ```

To recalibrate after changing the contact model, run the spin test and multiply
`wheel_separation_multiplier` by the yaw ratio.

## Stage 1: odometry on the original contact model

The original contact model came from `curt_mini_description`: `mu1=20`, `mu2=0.6`,
`fdir1=1 0 0`. The controller had `wheel_separation: 0.44`,
`wheel_separation_multiplier: 1.0` and `wheel_radius: 0.13`.

| Test (2026-11-11) | Ground truth | Odometry | Ratio odom / gt |
|---|---|---|---|
| Spin in place, `ω_cmd = 0.5 rad/s` | 654.2° | 856.3° | **1.309** |
| Straight, `v_cmd = 0.3 m/s` | 2.82 m | 2.82 m | **1.000** |

- **Rotation:** the ratio stayed between 1.309 and 1.310 in two independent runs.
  Odometry reported exactly the commanded 0.5 rad/s, while the base actually turned at
  about 0.38 rad/s.
- **Translation:** the wheel radius is correct.
- **Idle:** without Nav2, ground truth and odometry agreed to the centimetre. An
  earlier run with Nav2 active showed slow motion while "idle". That came from
  velocity commands reaching the controller, not from odometry.

That gave an effective separation of `0.44 × 1.309 = 0.576 m`, a multiplier of
`0.576 / 0.516 = 1.127`, and a verification spin ratio of **0.990** (847.6° ground
truth, 839.5° odometry).

## Stage 2: contact model

### Observation: the robot pivoted about its front axle

Even with the heading calibrated, ground truth recorded about 0.26 m of travel per
radian during spins in place, while odometry reported none. The distance from the
start rose to 0.52 m and fell back to about 0.02 m once per turn. The base was
pivoting about an offset point, not drifting. The measured centre of rotation was
`c_x = +0.260 ± 0.004 m`, `c_y = 0.000 m`: exactly the front axle. Halfway through
every spin in place, `base_link` was 0.52 m away from where odometry put it.

### Cause: rigid friction combined with a front-heavy load

The arm is mounted at `x = +0.25 m`, directly above the front axle.
[`wheel_loads.py`](wheel_loads.py) computes the static loads from the
robot description with the arm in its initial pose:

| | Value |
|---|---|
| Total mass | 16.55 kg (162.3 N) |
| Centre of mass in `base_link` | x = +0.068 m, y = +0.002 m, z = +0.174 m |
| Front left / front right | 51.6 N / 50.7 N (63.0 % on the front axle) |
| Rear left / rear right | 30.3 N / 29.7 N |

Gazebo's default friction is rigid Coulomb friction, applied separately in each
friction direction. A wheel either grips completely or slides with a lateral force of
`μ·N`, however fast it slides. During a spin in place, the lateral forces on the
front and rear axles have to cancel. With unequal loads, they can only cancel if the
heavier front wheels do not slide sideways at all, so the centre of rotation jumps to
the front axle.

A tyre with *slip compliance*, whose lateral force grows linearly with its lateral
slip and whose stiffness scales with its load, balances instead with the centre of
rotation under the centre of mass (x ≈ +0.068 m). The standard skid-steer kinematic
models assume the same.

### Model

[`urdf/curtmini_piper_gz.urdf.xacro`](../urdf/curtmini_piper_gz.urdf.xacro)
overrides the wheel contact settings of `curt_mini_description`. That package is
left unchanged, because these settings only affect simulation.

- **Friction:** a second `<gazebo reference="…_wheel">` block per wheel sets
  `mu1 = mu2 = 1.0` and `fdir1 = 0 0 1`. The URDF→SDF converter applies the later
  block and logs `multiple inconsistent <mu> exists due to fixed joint reduction
  overwriting previous value [20] with [1]` once per value and wheel. These warnings
  are the expected confirmation; despite the wording, they are unrelated to fixed
  joints.
- **Tyre compliance:** the `gz::sim::systems::WheelSlip` system, with each wheel's
  `wheel_normal_force` set to its static load from the table above. At zero wheel
  speed the compliance is zero, so a parked robot does not creep. The compliance
  values are the `wheel_slip_lateral` and `wheel_slip_longitudinal` xacro properties.

To check what reaches Gazebo:

```bash
docker compose exec gz-sim /opt/container/ros_entrypoint.sh bash -c \
  'ros2 param get --hide-type /robot_state_publisher robot_description > /tmp/r.urdf && \
   gz sdf -p /tmp/r.urdf | grep -A20 "<collision name=.front_left_wheel_collision"'
```

> **Pitfall: `fdir1` is in the collision frame, not the link frame.** The wheel
> cylinder is rotated `rpy="π/2 0 0"` relative to the wheel link, so the axle is
> the collision frame's **z** axis. With `fdir1 = 0 1 0`, the primary friction
> direction pointed along the rolling direction, and WheelSlip's *lateral*
> compliance acted longitudinally. More "lateral" compliance then made the wheels
> lose traction instead of grip: the spin yaw ratio rose from 1.443 (lateral 0.5) to
> 4.103 (lateral 1.0), while straight driving was unaffected.

### Compliance sweep

These runs use `fdir1 = 0 0 1`, longitudinal compliance 0.05 and multiplier 1.127.
"Max offset" is the largest distance of `base_link` from its starting point during
the spin. It is also the largest odometry position error during a spin in place.

| Lateral compliance | Centre of rotation `c_x` | Max offset | Spin yaw ratio | Straight ratio |
|---|---|---|---|---|
| original model | +0.260 m (front axle) | 0.52 m | 0.990 | 1.000 |
| 0.5 | +0.172 m | 0.34 m | 1.019 | 1.000 |
| **1.0 (chosen)** | **+0.084 m** | **0.17 m** | **1.018** | **1.000** |
| 2.0 | +0.068 m (centre of mass) | 0.14 m | 0.988 | 1.000 |

The value 1.0 puts the centre of rotation within 16 mm of the centre of mass, with
tyres half as soft as at 2.0. Softer tyres also slide more on curves, and there is no
real-robot data yet that would justify going further.

## Final calibration

For lateral compliance 1.0 the multiplier became `1.127 × 1.018 = 1.147`, an
effective separation of 0.592 m:

```yaml
wheel_separation: 0.516              # geometric track
wheel_radius: 0.13
wheel_separation_multiplier: 1.147   # skid-steer slip, WheelSlip lateral 1.0
```

| Verification (2026-11-11) | Ground truth | Odometry | Ratio odom / gt |
|---|---|---|---|
| Spin in place, `ω_cmd = 0.5 rad/s` | 712.2° | 712.5° | **1.000** |
| Straight, `v_cmd = 0.3 m/s` | 2.95 m | 2.95 m | **1.000** |

The centre of rotation stays at `c_x = +0.084 m`.

To check that the running controller has the new values:

```bash
docker compose exec gz-sim /opt/container/ros_entrypoint.sh ros2 param get /base_controller wheel_separation_multiplier
```

## Limitations

- **The offset centre of rotation is still not modelled.** `diff_drive_controller`
  assumes rotation about `base_link`, so a spin in place still produces up to 0.17 m
  of position error in `odom`. It would take odometry that models the offset centre
  of rotation (`v_y = -c_x·ω`), or lidar odometry fused in an EKF, to remove it.
- **Turn radius:** slip depends on turn radius, speed and acceleration, and the
  calibration was done with spins in place. Check typical Nav2 motions with combined
  `linear.x` and `angular.z` commands.
- **Wheel loads are static.** They assume the arm's initial pose. If the arm, its
  pose while driving or the robot's mass changes, rerun `wheel_loads.py` and update
  `wheel_normal_force`.
- **Real robot:** the real robot (`curt_mini_bringup/config/ros2_control.yaml`) still
  uses `0.44`. Its tyres and floor differ from this model, so it needs its own
  measurement, for example against motion capture or a lidar scan-matching
  reference. That measurement would also show whether lateral compliance 1.0 is
  realistic.

## Scripts

Both scripts are in this folder:

- [`odom_check.py`](odom_check.py) compares wheel odometry with ground truth. It
  runs inside `gz-sim`, read from stdin.
- [`wheel_loads.py`](wheel_loads.py) computes the centre of mass and the static
  wheel loads from the running `robot_description`. It runs on the host.
