# 🐢 Turtle Control – ROS 2 Package
Video Link:  https://drive.google.com/file/d/1zp8NR8dn_Y3sEov5s6nt1Ov2mW2FA83O/view?usp=sharing


# This ROS 2 package commands the turtlesim turtle to draw a **figure-eight** pattern. It demonstrates ROS 2 publishers, subscribers, services, and launch files.

---

## 📁 Workspace Structure

```
ros2_ws/
├── src/
│   └── turtle_control/
│       ├── turtle_control/             # Python module with turtle_controller.py
│       ├── launch/                     # Launch files
│       ├── package.xml
│       ├── setup.py
│       └── README.md                   # ← You are here
```

---

## 🛠️ Build Instructions

1. Navigate to your workspace:
   ```bash
   cd ~/turtle_ws
   ```

2. Build the package:
   ```bash
   colcon build 
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## 🚀 Run the Demo

Launch `turtlesim` and your controller node using:

```bash
	ros2 run turtlesim turtlesim_node
	---------------------------------
	source ~/turtle_ws/install/setup.bash
	ros2 run turtle_control turtle_controller

	
```

> This will start the turtle and make it draw a figure-eight.

---

## 🖋️ Toggle Pen Service

You can toggle the turtle’s pen (drawing on/off) using the following command:

```bash
ros2 service call /toggle_trace std_srvs/srv/Empty
```

This service switches between drawing and invisible movement.

---

## 🔧 Parameters

| Parameter                | Type    | Default | Description                          |
|--------------------------|---------|---------|--------------------------------------|
| `linear_velocity`        | double  | 2.0     | Forward speed of the turtle          |
| `turn_scale`             | double  | 0.85    | Angular speed factor (for turning)   |

---

## 📦 Features

- 🐢 Controls turtle using `/turtle1/cmd_vel`
- 📍 Logs live position using `/turtle1/pose`
- 🔁 Provides `/toggle_trace` service to enable/disable pen
- 🧠 Implements a simple state machine to complete a full figure-eight pattern

---

## 🧪 Tested On

- Ubuntu 22.04
- ROS 2 Humble
- `turtlesim`

---

## 👤 Author

- Your Name  
- BRAC University / CSE 463 Project

---
