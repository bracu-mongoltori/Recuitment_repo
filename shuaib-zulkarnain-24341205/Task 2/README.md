# Turtle Control ROS 2 Package

This ROS 2 package demonstrates control of the turtlesim turtle to draw a figure-eight pattern and toggle pen tracing using a custom service.

## Nodes

### `figure8_driver`
- Publishes velocity commands to move the turtle in a figure-eight path.
- Subscribes to `/turtle1/pose` and logs (x, y, θ) at 1Hz.
- Speed is controlled via the ROS 2 parameter `pattern_speed`.

### `trace_toggle_service`
- Provides a service `/toggle_trace` of type `std_srvs/srv/SetBool`.
- Toggles the turtle’s pen on or off.
- Moves the turtle forward a short distance when toggled.

## Launch File

- `bringup.launch.py` launches:
  - `turtlesim_node`
  - `figure8_driver`
  - `trace_toggle_service`

## Build Instructions

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop -y

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-colcon-common-extensions -y

# Build the workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run the Project

```bash
ros2 launch turtle_control bringup.launch.py
```

To toggle the pen:
```bash
ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: true}"  # Turn ON
ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: false}" # Turn OFF
```

## Demo Videos

1. [Explanation Video Part 1](https://drive.google.com/file/d/14SmwMv3whGuf6uPVDxCPPaETqx-yEdCB/view?usp=sharing)
2. [Explanation Video Part 2](https://drive.google.com/file/d/161jji_Djhs2u1ddrNafGPwWulXYyCEZQ/view?usp=sharing)
