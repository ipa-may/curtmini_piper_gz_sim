# Simulation launch helpers

These Python modules provide the actions used by
[`launch/simulation.launch.py`](../launch/simulation.launch.py). That launch
file declares the user-facing arguments and assembles the actions in startup
order. CMake installs this directory as a Python package so the launch file
can import the same modules from a local ROS build or a Docker image.

| File | Responsibility |
| --- | --- |
| `__init__.py` | Marks this directory as an importable Python package. |
| `apriltags.py` | Validates `config/apriltags.yaml`, builds the textured panel models, and creates their Gazebo spawn actions. The AprilTag detector runs in a separate Compose service. |
| `cameras.py` | Reads camera mounts from the robot description's `geometry.yaml` and creates static transforms for the camera links and optical frames. The camera sensors themselves are defined in the robot description. |
| `gazebo.py` | Resolves the selected world and creates the Gazebo startup, resource path, and ROS bridge actions. |
| `moveit.py` | Selects the arm model's MoveIt configuration and creates the RViz action. |
| `robot.py` | Validates the initial base and arm pose against the selected URDF, prepares the simulation robot description, and creates robot publisher, spawn, and controller actions. |
| `validation.py` | Provides YAML key and finite-number checks shared by the other modules. |

For normal use, run or configure `simulation.launch.py`; these modules are
helpers rather than separate launch files.
