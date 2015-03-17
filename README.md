# joint_control
Purpose: To control kuka youbot arm and store the resulting encoder data while robot performs a trajectory
Email: surenkum@buffalo.edu
Website: www.buffalo.edu/~surenkum

Place the code insider src directory of your ROS workspace and then

catkin_make && roslaunch joint_control_kuka kuka_robot_move.launch

Before launching above code, please make sure roscore is running and 
the robot side launch file has been launched

# On Robot PC
Copy the on_youbot/robot_initialize.launch file to Kuka youbot and run
roslaunch robot_initialize.launch on robot PC


