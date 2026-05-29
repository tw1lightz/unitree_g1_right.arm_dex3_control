# Conventions

## Code Style

### C++
- **Standard**: C++17
- **Compiler flags**: `-Wall -Wextra -Wpedantic`
- Classes use PascalCase (`Dex3Controller`, `IKFCLPlannerNode`)
- Member variables use trailing underscore (`hand_cmd_pub_`, `side`, `tactile_threshold_`)
- Methods use camelCase (`handCmdCallback`, `lowstateCallback`)
- Enums use `kPascalCase` values (`kLeftShoulderPitch`, `kThumb0`)
- `using namespace std::chrono_literals` is used in control nodes

### Python
- Standard Python 3 style
- ROS 2 node class inherits from `rclpy.node.Node`
- Parameters declared with `self.declare_parameter()`

## Patterns

### URDF-at-Runtime
All control/planning nodes fetch the URDF from the `/robot_state_publisher` service at startup:
```cpp
auto client = this->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");
// wait for service, request "robot_description" parameter, parse URDF
```
This pattern is repeated in `joint_state_publisher.cpp`, `dex3_controller.cpp`, `joint_trajectory_executor.cpp`, and `ik_fcl_ompl_planner.cpp`.

### QoS Best-Effort
Hardware-facing publishers/subscribers use best-effort QoS:
```cpp
rclcpp::QoS qos_profile(10);
qos_profile.best_effort();
```

### Side-Parameterized Nodes
The `dex3_controller` is designed to run as two instances (left/right) configured by a `side` parameter. Topic names are constructed dynamically from the side.

### Launch File Pattern
All launch files use `OpaqueFunction` to resolve `LaunchConfiguration` values at launch time:
```python
def launch_setup(context, *args, **kwargs):
    value = LaunchConfiguration('param').perform(context)
    return [Node(...)]
```

## Error Handling

- Fatal errors (missing URDF, service unavailable) call `RCLCPP_FATAL()` then `rclcpp::shutdown()`
- Service wait timeouts vary: some nodes wait indefinitely, others timeout after N retries
- Joint commands are clamped to URDF limits before sending to hardware
- Python detector wraps `rclpy.spin()` in try/except for clean KeyboardInterrupt shutdown

## Logging

- Uses ROS 2 logging: `RCLCPP_INFO`, `RCLCPP_WARN`, `RCLCPP_ERROR`, `RCLCPP_FATAL`
- Throttled warnings for service wait loops: `RCLCPP_WARN_THROTTLE`
- No custom logging framework
