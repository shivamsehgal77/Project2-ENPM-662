#! /usr/bin/env python

import rospy

from std_msgs.msg import Float64


# x = 0.0
# y = 0.0 
# theta = 0.0

# def newOdom(msg):
#     global x
#     global y
#     global theta

#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y


def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I recieved -- %f",data.data)
    
rospy.init_node('spy')



rospy.Subscriber('/batbat/front_motor_left/command', Float64,callback)
rospy.Subscriber('/batbat/front_motor_right/command', Float64,callback)
rospy.Subscriber('/batbat/rear_motor_left/command', Float64,callback)
rospy.Subscriber('/batbat/rear_motor_right/command', Float64,callback)
rospy.spin()

# model_coordinates = rospy.Subscriber('/gazebo/model_state', ModelStates)


# print(ModelStates().pose)
# # while(True):
#     print(x)
#     print(y)
