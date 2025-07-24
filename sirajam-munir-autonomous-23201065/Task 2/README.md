### Explanation Video: https://drive.google.com/file/d/1Pw0OmnjxN5y5NpzPXrvKmjRYF717oF73/view?usp=sharing



### How to Run the turtle toggle simulator

## Build Workspace by runnig this in WSL

cd ~/ros2_ws
colcon build

---

## Then source the environment (after every new terminal or build):

source install/setup.bash

---

## Launch the Simulator and Nodes:

ros2 launch turtle_control bringup.launch.py

---

## Use the Toggle Service to Control the Pen

source ~/ros2_ws/install/setup.bash

# Turn the pen ON:

ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: true}"

# Turn the pen OFF:

ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: false}"