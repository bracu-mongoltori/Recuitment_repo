to build :
go to the root of the work space 

run = colcon build --symlink-install
source ~/.bashrc

to run:

ros2 run turtlesim turtlesi_node 
ros2 run my_robot_controller figure8_driver
ros2 run my_robot_controller trace_toggle
ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: true/false}
true to turn on pen
false to turn off pen


nodes:

figure8_driver = drives the turtle using parameter based speed
trace_toggle = turns the pen off or on

challenges faced:
took 3 days to install ros2 because of various technical errors
faced lots of error to make the environment run in local drive 
finding out the logic to make the turtle go in 8 pattern 
working around the setpen service having no reponse fields 