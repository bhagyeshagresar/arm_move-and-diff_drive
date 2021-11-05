# Part 2 - Motion Planning



# arm_move ROS Package
The package consists of `mover` node which makes the `PX100` robot arm follow a sequence of waypoints and grab an object. The simulated world and the real world consists of RealSense Box as an obstacle. The package uses MoveItPython API to do the path planning tasks and visualize it in Rviz using moveit_rviz.

# Instructions to Setup the arm_move ROS Package

1. Compile the workspace using `catkin_make`
2. Source the setup script of the workspace using `source devel/setup.bash`
3. For visualizing the arm in simulation run the fake node using launchfile arm.launch by running `roslaunch arm_move arm.launch use_fake:=true`
4. For visualizing the arm in Gazebo run `roslaunch arm_move arm.launch use_gazebo:=true`
5. For testing the arm in real world run `roslaunch arm_move arm.launch use_actual:=true`


#Video Demonstration of the Robot achieving the task in Gazebo

https://drive.google.com/file/d/1QURlXK5FowxnLMrKvKwXF4iNkWYaso2w/view?usp=sharing



#Video Demonstration of the Robot achieving the task in real world

https://drive.google.com/file/d/17NQ39kb6eSu9Mzt0Y28J7WXMAGL5oIDV/view?usp=sharing