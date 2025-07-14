# Turtle Control – ROS 2 Package

##Explanation Video(Watch this in 1.5x to ignore my boring voice🙂):
https://drive.google.com/file/d/1epQ92dB818l4Oe_Xc9CLiuMyGgQTaX4s/view?usp=sharing


## Overview

This package demonstrates ROS 2 fundamentals using the `turtlesim` simulator. It includes:

- A node that drives the turtle in a **figure-eight** pattern and logs its pose.
- A service node to **toggle the pen (trace)** on or off.
- A launch file to start all components together.

This project showcases core ROS 2 concepts including publishers, subscribers, services, custom launch files, and modular Python nodes.

---

## Node Descriptions

| Node Name              | Type                   | Purpose                                           |
|------------------------|------------------------|---------------------------------------------------|
| `figure8_driver`       | Publisher/Subscriber   | Drives the turtle in a figure-eight path and logs its pose |
| `trace_toggle_service` | Service                | Toggles the pen on/off using the `/toggle_trace` service   |

---

## Directory Structure


<pre>
ros2_ws/
├── src/
│   └── turtle_control/
│       ├── turtle_control/
│       │   ├── __init__.py
│       │   ├── figure8_driver.py
│       │   └── trace_toggle_service.py
│       ├── launch/
│       │   └── bringup.launch.py
│       ├── package.xml
│       └── setup.py
├── README.md
</pre>

## Build Instructions:

Navigate to your workspace:
<pre>
    cd ~/ros2_ws
</pre>
Build the package:
<pre>colcon build --packages-select turtle_control</pre>
Source your workspace(depending on your shell):
<pre>source install/setup.bash</pre>
or,If you are on zsh shell:
<pre>source install/setup.zsh</pre>

## Running the Demo
Launch turtlesim and your custom nodes:
<pre>ros2 launch turtle_control bringup.launch.py</pre>

Toggle the pen (trace) ON:
<pre>ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: true}"</pre>
Toggle the pen (trace) OFF:
<pre>ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: false}"</pre>

Notes:
- Ensure all dependencies are installed and your ROS 2 environment is properly sourced according to your shell.
- Replace maintainer information in package.xml and setup.py with your own details.
- For development, consider using colcon build --symlink-install for faster iteration.


