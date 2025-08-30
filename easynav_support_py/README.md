# easynav_support_py

Python client compatible with the C++ `GoalManager` in EasyNav.

## Features

- `GoalManagerClient` publishes/consumes `easynav_interfaces/msg/NavigationControl` on `easynav_control`.
- Also publishes commanded goal to `goal_pose` as `geometry_msgs/PoseStamped` (parity with C++).
- Maintains client-side state and caches last FEEDBACK/RESULT messages.
- Integration tests (pytest + launch_testing) that interact with the C++ GoalManager.

## Build & Test

```bash
# Inside your ROS 2 workspace
colcon build --packages-select easynav_support_py
source install/setup.bash

# Run tests (ensure the C++ system node is built and runnable)
colcon test --packages-select easynav_support_py --event-handlers console_direct+
colcon test-result --all
```
