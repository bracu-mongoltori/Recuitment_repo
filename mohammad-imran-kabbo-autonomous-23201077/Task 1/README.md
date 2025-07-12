

# 🐢 Turtle Control - ROS 2 Project (Mongol-Tori Task 2)

This project demonstrates core ROS 2 concepts using the `turtlesim` simulator.  
The turtle moves in a **figure-eight path**, and a service allows toggling the pen on/off — to draw or move invisibly.

---

## ✅ Features

- `figure8_driver.py`: Publishes velocity commands to move turtle in figure-eight.
- Subscribes to `/turtle1/pose` to log turtle coordinates every second.
- `trace_toggle.py`: Exposes a ROS 2 service `/toggle_trace` to toggle the pen.
- `bringup.launch.py`: Launches everything together with a single command.

---

## 🧱 Project Structure

```plaintext
turtle_control/
├── launch/
│   └── bringup.launch.py       # Launches turtlesim + figure8 + service
├── turtle_control/
│   ├── __init__.py             # Required for Python modules
│   ├── figure8_driver.py       # Main motion controller node
│   ├── trace_toggle.py         # Pen toggle service node
│   └── turtle_controller.py    # (Optional) basic circular motion node
├── setup.py                    # Python package config
├── setup.cfg                   # Script install paths
└── package.xml                 # ROS 2 package manifest


🔧 Setup Instructions

🟩 Step 1: Create and Navigate to Your Workspacee

mkdir -p ~/ros2_ws/src
# 📁 Create a new ROS 2 workspace called 'ros2_ws' with a 'src' folder inside
cd ~/ros2_ws/src
# 📂 Move into the 'src' folder where packages are kept


🟩 Step 2: Build Your ROS 2 Workspace

cd ~/ros2_ws
# 🔙 Move to the root of the workspace (above 'src')

colcon build --packages-select turtle_control
# 🔧 Build only the 'turtle_control' package using colcon
# 🚫 This avoids building unnecessary packages


🟩 Step 3: Source the Setup File

source install/setup.bash
# ✅ Sets up your ROS 2 environment so ROS can find your package and nodes
# ❗ Must run this EVERY time you open a new terminal

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

🟩 Step 4: Run the Project Using Launch File

ros2 launch turtle_control bringup.launch.py
# 🚀 This will:
#    - Start the turtlesim GUI
#    - Run your figure8 driver node
#    - Run the pen toggle service node

🐢 You’ll see the turtle moving in a figure-eight pattern on screen.



🟩 Step 5: Use Service to Toggle Drawing


source ~/ros2_ws/install/setup.bash
# 🛠 Again, source your environment in the new terminal

ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: false}"
# ✏️ This turns the pen OFF — the turtle moves but doesn't draw

ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: true}"
# 🖍 This turns the pen ON again — the turtle resumes drawing