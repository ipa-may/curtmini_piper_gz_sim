# Investigating the MoveIt shutdown crash

## Observed shutdown failure

The simulation passes its automated startup checks: the robot appears in
Gazebo, simulation time advances, joint states are published, and the
controllers become active. However, MoveIt’s `move_group` process fails
during shutdown.

An earlier shutdown run reported:

```text
terminate called after throwing an instance of 'std::runtime_error'
  what(): context cannot be slept with because it's invalid
```

We use GDB to capture the function calls and thread states at the failure,
identify which part of MoveIt is involved, and investigate the cause.

## Install GDB and xterm

```sh
sudo apt install gdb xterm
```
GDB stands for GNU Debugger. It lets you run a program, pause it, inspect its variables, and see which functions are executing.
xterm is a graphical terminal emulator, similar to Ubuntu’s Terminal app.

## Launch MoveIt under GDB
Add prefix argument to the Node that launches move_group:

Here in `simulation.launch.py`:
```py
Node(
    package='moveit_ros_move_group',
    executable='move_group',
    output='screen',
    prefix=['xterm -e gdb -q -ex run --args'],
    # Keep the existing parameters, remappings, etc.
),
```

In the command `xterm -e gdb ...`, it opens a separate terminal window running GDB, where you can type debugger commands while the original terminal runs ROS launch.

## Run the application and save a backtrace file

Rebuild and source the workspace (from the ROS 2 workspace root)
```sh
colcon build
source install/setup.bash
```

```sh
ros2 launch curtmini_piper_gz_sim simulation.launch.py
```

Leave the original ROS launch terminal running. Inside the xterm window running GDB:

1. Press **Ctrl+C** to pause MoveIt
2. Wait for the GDB prompt
3. type each command, pressing Enter after each time

```txt
set pagination off
set logging file /tmp/moveit-backtrace.txt
set logging enabled on
catch throw
handle SIGABRT SIGSEGV stop print nopass
signal SIGINT
```

4. When GDB stops on SIGABRT or SIGSEGV, enter the following commands.
```txt
bt full
thread apply all bt full
set logging enabled off
```

The trace will be saved in `/tmp/moveit-backtrace.txt`.

## Intermediate conclusion

The simulation passes its startup checks, but MoveIt fails during shutdown.

GDB captured a `SIGABRT` in MoveIt 2.12.3’s planning-scene publishing
thread. The relevant call chain was:

```text
PlanningSceneMonitor::scenePublishingThread()
  → rclcpp::Rate::sleep()
  → rclcpp::Clock::sleep_for()
  → C++ exception
  → std::terminate()
  → SIGABRT
```

This is consistent with the publishing thread attempting to sleep after its ROS context became invalid. 

The exception was thrown in thread 21 through this exact path:
```txt
PlanningSceneMonitor::scenePublishingThread()
  → rclcpp::Rate::sleep()
  → rclcpp::Clock::sleep_for()
  → __cxa_throw()
```

At that same moment, thread 5 was executing:
```
rclcpp::SignalHandler::deferred_signal_handler()
  → rclcpp::Context::shutdown()
```

This proves that:
- The exception originates from the planning-scene publishing thread’s Rate::sleep().
- The ROS context was being shut down concurrently.
- The exception escapes the publishing thread and later causes std::terminate() and SIGABRT.

The check does not prevent Rate::sleep() from executing during context shutdown


## Investigations and results

The experiments below used MoveIt 2.12.3 built in the local overlay:

```text
~/moveit_ws
```

After rebuilding MoveIt, source the workspaces in this order:

```sh
source /opt/ros/jazzy/setup.bash
source ~/moveit_ws/install/local_setup.bash
source ~/ROB4/rob4_fraunhofer_ws/install/local_setup.bash
```

### Replace `Rate` with `WallRate`

The planning-scene publishing thread was temporarily changed from
`rclcpp::Rate` to `rclcpp::WallRate`.

This did not solve the shutdown failure. `WallRate` still uses the same
sleeping mechanism, so changing the clock used by the rate did not coordinate
the publishing thread with context shutdown. The change was reverted.

### Stop planning-scene publishing before context shutdown

A pre-shutdown callback was added to `move_group.cpp`:

```cpp
const std::weak_ptr<::planning_scene_monitor::PlanningSceneMonitor> weak_monitor = planning_scene_monitor;
nh->get_node_base_interface()->get_context()->add_pre_shutdown_callback([weak_monitor]() {
  if (const auto monitor = weak_monitor.lock())
    monitor->stopPublishingPlanningScene();
});
```

The weak pointer prevents the callback from extending the lifetime of the planning-scene monitor.

#### Result

After this change, GDB no longer stopped on an exception from:

```text
PlanningSceneMonitor::scenePublishingThread()
  → rclcpp::Rate::sleep()
```

This verifies that stopping planning-scene publishing before invalidating the context prevents the original exception. Shutdown then exposed a second failure:

```text
SIGSEGV
  → rclcpp::Executor::~Executor()
  → main() at move_group.cpp
```

### Backport the trajectory-execution cleanup

The cleanup order from MoveIt PR [#3828](https://github.com/moveit/moveit2/pull/3828) was applied to
`TrajectoryExecutionManager::~TrajectoryExecutionManager()`.

The callback, controller-manager node, executor, and controller manager are now explicitly released in a defined order:

```cpp
callback_handler_.reset();
if (private_executor_ && controller_mgr_node_)
  private_executor_->remove_node(controller_mgr_node_);
private_executor_.reset();
controller_manager_.reset();
controller_mgr_node_.reset();
```

#### Result

The next backtrace still ended in:

```text
rclcpp::Executor::~Executor()
  → main() at move_group.cpp
```

The GDB local variables showed that `moveit_cpp` was still alive when the main executor crashed. The patched `TrajectoryExecutionManager` destructor had not been reached yet. This patch may prevent a later trajectory-execution cleanup problem, but it does not fix the main executor crash.

### Upgrade `rclcpp`

The installed Jazzy `rclcpp` package was upgraded:

```text
28.1.9 → 28.1.22
```

#### Result

The updated backtrace was unchanged:

The newer Jazzy `rclcpp` package therefore does not fix this crash.

### Explicitly remove the node from the executor

The main MoveIt node was explicitly removed from the executor after
`executor.spin()` returned:

```cpp
executor.add_node(nh);
executor.spin();
executor.remove_node(nh);

rclcpp::shutdown();
```

Only `moveit_ros_move_group` needed to be rebuilt:

```sh
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build \
    --packages-select moveit_ros_move_group \
    --executor sequential
```

The build completed successfully, and `ros2 pkg prefix moveit_ros_move_group` resolved to:

```text
~/moveit_ws/install/moveit_ros_move_group
```

#### Result

The backtrace in `/tmp/moveit-throw-6.txt` showed that `Executor::~Executor()` completed without crashing. Cleanup proceeded until the final reference to the main ROS node was released:

```text
rclcpp::CallbackGroup::~CallbackGroup()
  → rclcpp::node_interfaces::NodeBase::~NodeBase()
  → rclcpp::Node::~Node()
  → main() at move_group.cpp
```

In `rclcpp` 28.1.22, `CallbackGroup::~CallbackGroup()` triggers the callback group's notification guard condition. 
The result indicates that callback-group cleanup is still occurring after the ROS context has become invalid.

The explicit `remove_node()` call improves the cleanup sequence, but it does not provide a clean shutdown.

## Current conclusion

The investigation identified successive failures in the shutdown sequence.

The original exception occurred because the planning-scene publishing thread
called `Rate::sleep()` while the ROS context was being invalidated. Stopping
planning-scene publishing in a pre-shutdown callback prevents that exception.

A separate segmentation fault then occurred in `rclcpp::Executor::~Executor()`.
Explicitly removing the node allowed executor destruction to finish, but the
process subsequently crashed in `rclcpp::CallbackGroup::~CallbackGroup()` while
destroying the node.

Changing the rate type, backporting the trajectory-execution cleanup, and
upgrading `rclcpp` did not produce a clean shutdown. The remaining problem is
the lifetime order between the ROS context and the MoveIt node, callback groups, and executor. 
These objects need to be released before the context is invalidated.
