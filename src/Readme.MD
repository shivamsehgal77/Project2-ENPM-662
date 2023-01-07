############## PACKAGE INSTRUCTIONS FOR OUR UR5_MOBILE_ROBOT ##################

The src file contains 4 packages 
1.)batbat this is the mobile robot we built for the first project
2.)ur5_mobile_robot this is the main package that contains all the files for
our mobile robot (the launch files , the code files)
3.)ur5_simple_pick_and_place this contains the worlds for simulation in gazebo
4.)ur5_description this contains the files we got form the offical ros github for the 
ur5 arm.


We also have put the codes for the Forward kinematic and workspace study in the folder
1.) FK_workspace.ipynb use jupyter notebook to open this


********For simulation run the following codes#####################

Before running the launch file 

1.) Source noetic
2.) Source the catkin_ws where you build your packages

### First run the Package in Gazebo using this in terminal
roslaunch ur5_mobile_robot control_2.launch

Then play the the simulation as it would be in pause

### Then use this python code to run the IK solver and Pick and Place simulation
python3 /home/shivam/catkin_ws2/src/ur5_mobile_robot/src/pick_and_place.py


*****************OTHER LAUNCH FILES

### This run the the robotic arm in the gazebo environment
roslaunch ur5_mobile_robot control.launch

