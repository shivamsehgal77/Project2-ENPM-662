#! /usr/bin/env python

#imported libraries
import rospy

from std_msgs.msg import Float64

from gazebo_msgs.msg import ModelStates
# x = 0.0
# y = 0.0 
# theta = 0.0

# def posei(msg):
#     global x
#     global y
#     global theta

#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y
rospy.init_node('joint_state_publisher')

#rospy to make the node as speed_controller


#publishers

pub_right = rospy.Publisher('/front_motor_left/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
pub_left = rospy.Publisher('/front_motor_right/command', Float64, queue_size=10)
pub_left_move = rospy.Publisher('/rear_motor_left/command', Float64, queue_size=10) # Add your topic for move here '' Eg '/my_robot/longitudinal_controller/command'
pub_right_move = rospy.Publisher('/rear_motor_right/command', Float64, queue_size=10) # Add your topic for move here '' Eg '/my_robot/longitudinal_controller/command'
#odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
control_turn=-0.5
control_speed=1

print(f"Speed Published to subscriber = {control_speed} \n")
print(f"Steering angle publishe to subscriber= {control_turn}")

while(True):

    pub_right.publish(control_turn)
    pub_left.publish(control_turn)
    pub_left_move.publish(control_speed)
    pub_right_move.publish(control_speed)




