# Pick and Place & Sorting — Mitsubishi RV-2FR (ROS 2 Simulation)

A ROS 2 Python package that performs **vision-guided pick-and-place** and **colour-based sorting** on a simulated **Mitsubishi RV-2FR** 6-DOF industrial robot arm using the `pymoveit2` library and MoveIt 2.

---

## Overview

The system detects coloured objects on a table using a top-down camera, computes their 3D positions in the robot base frame, and commands the robot to pick and place them at a predefined drop location — effectively sorting objects by colour.

```
Camera Feed → Colour Detector → /color_coordinates topic → Pick & Place Node → MoveIt 2 → RV-2FR
```

---

## Package Structure

```
pick_and_place/
├── pick_and_place/
│   ├── pick_place.py            # Main pick-and-place node (MoveIt 2 + pymoveit2)
│   ├── colour_detector.py       # Vision node: HSV detection + TF2 projection
│   ├── goto_pose.py             # MoveIt 2 collision scene / sorting helper
│   ├── commander_subscriber.py  # Joint trajectory slider control node
│   └── __init__.py
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── resource/
├── package.xml
└── setup.py
```

---

## Nodes

### 1. `pick_and_place_node` — `pick_place.py`

The core execution node. Subscribes to detected object coordinates and executes a full pick-and-place motion sequence using `pymoveit2`.

**Key behaviour:**
- Moves to a predefined **start/home** joint configuration on startup.
- Listens to `/color_coordinates` for a target colour match (configurable via ROS 2 parameter).
- Executes a **5-step pick sequence**:
  1. Open gripper and move to home.
  2. IK-based move **above** the object.
  3. Cartesian descent **straight down** onto the object (no wrist spin).
  4. Close gripper (grasp).
  5. Cartesian lift straight up.
- Moves to a **drop joint configuration** and releases the gripper.
- Returns to start position and shuts down.

**ROS 2 Parameter:**

| Parameter      | Default | Description                             |
|----------------|---------|-----------------------------------------|
| `target_color` | `"B"`   | Target colour to pick — `R`, `G`, or `B` |

**Subscribed Topics:**

| Topic               | Type              | Description                             |
|---------------------|-------------------|-----------------------------------------|
| `/color_coordinates` | `std_msgs/String` | Comma-separated `COLOR,x,y,z` string from the detector |

**Robot Configuration (RV-2FR):**

| Parameter                       | Value                              |
|----------------------------------|------------------------------------|
| Move Group (Arm)                 | `rv2fr`                            |
| Move Group (Gripper)             | `gripper`                          |
| Base Link                        | `rv2fr_base`                       |
| End Effector                     | `rv2fr_default_tcp`                |
| Gripper Open Positions           | `[0.0, 0.0]`                       |
| Gripper Closed Positions         | `[-0.20, 0.20]`                    |
| Max Velocity Scaling             | `0.1`                              |
| Max Acceleration Scaling         | `0.1`                              |
| Cartesian Step Size              | `0.005 m`                          |
| Cartesian Jump Threshold         | `0.0` (disabled)                   |

---

### 2. `colour_detector` — `colour_detector.py`

Vision node that detects coloured blocks in a top-down camera stream and publishes their 3D world coordinates.

**Key behaviour:**
- Subscribes to `/camera/image_raw` and converts images using `cv_bridge`.
- Applies HSV colour segmentation for Red, Green, and Blue objects.
- Uses morphological operations (erode + dilate) to clean the mask.
- Looks up the `rv2fr_base → camera_link_optical` TF transform.
- Projects 2D pixel coordinates to 3D robot-base coordinates using the pinhole camera model and TF transform matrix.
- Publishes detections and overlays bounding boxes on a live OpenCV window.

**Camera Intrinsics (derived from URDF — FOV 1.0 rad, 640×320):**

| Parameter | Value |
|-----------|-------|
| `fx`      | 585.9 |
| `fy`      | 585.9 |
| `cx`      | 320.0 |
| `cy`      | 160.0 |

**HSV Colour Ranges:**

| Colour | Lower HSV     | Upper HSV      |
|--------|---------------|----------------|
| Red    | (0, 120, 70)  | (10, 255, 255) |
| Green  | (50, 100, 100)| (75, 255, 255) |
| Blue   | (100, 150, 0) | (140, 255, 255)|

**Subscribed Topics:**

| Topic               | Type                  | Description       |
|---------------------|-----------------------|-------------------|
| `/camera/image_raw` | `sensor_msgs/Image`   | Raw camera feed   |

**Published Topics:**

| Topic                | Type              | Description                              |
|----------------------|-------------------|------------------------------------------|
| `/color_coordinates` | `std_msgs/String` | `COLOR,x,y,z` — detected object in base frame |

---

### 3. `commander` — `commander_subscriber.py`

A utility slider-control node for manual joint control during development and testing.

- Subscribes to `/joint_commands` (`sensor_msgs/JointState`).
- Forwards the first 6 positions to the arm controller (`rv2fr_controller/joint_trajectory`).
- Forwards position index 6 to the gripper controller (`gripper_controller/joint_trajectory`).

---

### 4. `goto_pose` — `goto_pose.py` *(Sorting Scene Helper)*

Manages the MoveIt 2 planning scene for the sorting task.

- Publishes `CollisionObject` messages to add boxes to the scene.
- Publishes `AttachedCollisionObject` messages to attach grasped objects to the robot's flange (`rv7frl_hand_flange`).
- `touch_links` are set to prevent false collision errors during grasping.

---

## Dependencies

### ROS 2 Packages

| Package                         | Purpose                          |
|----------------------------------|----------------------------------|
| `rclpy`                          | ROS 2 Python client library      |
| `moveit_py` / `pymoveit2`        | MoveIt 2 Python interface        |
| `moveit_msgs`                    | MoveIt planning scene messages   |
| `geometry_msgs`                  | Pose, transforms                 |
| `std_msgs`                       | String topic communication       |
| `sensor_msgs`                    | Image, JointState                |
| `tf2_ros` / `tf_transformations` | Frame transforms                 |
| `trajectory_msgs`                | Joint trajectory control         |
| `moveit_ros_planning_interface`  | MoveIt ROS interface             |

### Python Libraries

- `opencv-python` (`cv2`)
- `numpy`
- `cv_bridge`

---

## Installation

### Prerequisites

- ROS 2 Humble (or later)
- MoveIt 2
- Mitsubishi RV-2FR URDF / MoveIt config package
- `pymoveit2` installed in your workspace

### Build

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
# (place or clone the pick_and_place package here)

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select pick_and_place

# Source
source install/setup.bash
```

---

## Usage

### Step 1: Launch the Simulation

Launch your Mitsubishi RV-2FR simulation environment (Gazebo / RViz) with MoveIt 2 and the camera sensor active.

```bash
ros2 launch <your_rv2fr_moveit_config> demo.launch.py
```

### Step 2: Start the Colour Detector

```bash
ros2 run pick_and_place colour_detector
```

This opens a live OpenCV window showing detected objects with bounding boxes and their computed base-frame coordinates.

### Step 3: Run the Pick and Place Node

```bash
# Pick a Blue object (default)
ros2 run pick_and_place pick_and_place_node

# Pick a Red object
ros2 run pick_and_place pick_and_place_node --ros-args -p target_color:=R

# Pick a Green object
ros2 run pick_and_place pick_and_place_node --ros-args -p target_color:=G
```

The robot will:
1. Move to the home position.
2. Wait for a matching colour detection on `/color_coordinates`.
3. Execute the full pick-and-place motion sequence.
4. Shut down automatically upon completion.

### Step 4: (Optional) Manual Joint Control

```bash
ros2 run pick_and_place commander
```

Then publish joint states to `/joint_commands` for manual teleoperation.

---

## Motion Sequence

```
Home Position
     │
     ▼
Open Gripper
     │
     ▼
Move ABOVE object  (IK via MoveIt 2)
     │
     ▼
Cartesian descent DOWN  (no wrist spin)
     │
     ▼
Close Gripper  (grasp)
     │
     ▼
Cartesian lift UP
     │
     ▼
Move to DROP configuration  (joint-space)
     │
     ▼
Open Gripper  (release)
     │
     ▼
Return to Start  →  Shutdown
```

---

## Notes

- The node processes **one object per run** (`already_moved` flag). Restart the node to pick another object.
- Object Z height is hardcoded to `0.05 m` in the detector (table surface assumption). Adjust for different object heights.
- The camera intrinsics (`fx`, `fy`, `cx`, `cy`) are derived from the URDF camera FOV — update these if the camera configuration changes.
- Cartesian motion uses a step size of `0.005 m` with jump threshold disabled (`0.0`) to prevent unexpected wrist reorientation.

---

## Maintainer

**Ayush**  
Email: `ayushjbhandurge@gmail.com`
