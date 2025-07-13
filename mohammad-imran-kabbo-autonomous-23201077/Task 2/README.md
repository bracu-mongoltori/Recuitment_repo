
# Explanation Video : 

https://drive.google.com/file/d/1WFxJB0KfKUDtxDt_8UVPs4CBfX3QkUGo/view?usp=sharing



# Turtle Control - Figure-8 and Trace Toggle in ROS 2 (Humble)

This ROS 2 project allows you to:
- Draw a smooth **figure-8** path using turtlesim.
- Toggle the turtle's **pen trace** (on/off) via a custom service.

---

## ✅ Prerequisites

Make sure you have:
- ROS 2 Humble installed
- Your workspace built (`~/ros2_ws`)
- The `turtle_control` package created and containing:
  - `figure8_driver.py` (draws a figure-8)
  - `trace_toggle.py` (toggles the pen on/off)

---

## 🛠️ Build the Workspace

Open a terminal and run:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🚀 How to Run

Open **multiple terminals** (Ctrl+Shift+T or split windows in WSL). Run these in order:

---

### 1️⃣ Source ROS and your workspace

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

### 2️⃣ Run the turtlesim GUI

```bash
ros2 run turtlesim turtlesim_node
```

This opens the window with the turtle.

---

### 3️⃣ Run the figure-8 driver

In a **new terminal**:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtle_control figure8_driver
```

This moves the turtle in a figure-8 path.

---

### 4️⃣ Run the trace toggle service node

In **another terminal**:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtle_control trace_toggle
```

---

### 5️⃣ Toggle the Pen (Trace) ON or OFF

#### 🖊️ To turn the pen ON:

```bash
ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: true}"
```

You should see:
```
response:
std_srvs.srv.SetBool_Response(success=True, message='Pen turned ON')
```

Now the turtle draws while moving.

#### ❌ To turn the pen OFF:

```bash
ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: false}"
```

You should see:
```
response:
std_srvs.srv.SetBool_Response(success=True, message='Pen turned OFF')
```

The turtle now moves without drawing.

---

## 📌 Notes

- Always source your environment before running anything:
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  ```

- If you make changes to your code, rebuild using:
  ```bash
  colcon build --packages-select turtle_control
  ```

---

## 🧼 Clean Build (Optional)

If you face strange behavior, you can try a clean rebuild:

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build
source install/setup.bash
```

---

## 🐢 Enjoy Watching the Turtle Draw! 🐢
