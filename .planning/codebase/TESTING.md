# Testing

## Framework

- **ament_lint_auto** — Configured in `CMakeLists.txt` under `BUILD_TESTING` guard, but no custom tests implemented
- No unit test files found in the main package
- No CI/CD configuration for this workspace

## Test Nodes (Manual/Integration)

The project relies on dedicated tester nodes for manual integration testing:

### `visual_detection_tester`
- Source: `src/unitree_g1_dex3_stack-main/src/visual_detection_tester.cpp`
- Purpose: Click-based detection testing — click on RGB image to get 3D position
- Launch: `visual_detect_click.launch.py`

### `visual_detection_yolo_tester`
- Source: `src/unitree_g1_dex3_stack-main/src/visual_detection_yolo_tester.cpp`
- Purpose: YOLO detection testing — monitors target class detections, displays TF-transformed 3D positions, keyboard-triggered output
- Launch: `visual_detect_yolo.launch.py`

### `right_hand_pressure_monitor`
- Source: `src/unitree_g1_dex3_stack-main/src/right_hand_pressure_monitor.cpp`
- Purpose: Logs pressure sensor values at 1Hz for tactile feedback debugging

## Test Coverage

- **No automated unit tests** for any node
- **No integration tests** (e.g., launch_testing)
- Testing is done manually on the physical robot
- The YOLO model (`best.pt`) was custom-trained externally; no training pipeline in this repo

## Mocking

- No mock infrastructure
- Nodes depend on live hardware (robot, camera) or the robot_state_publisher service
