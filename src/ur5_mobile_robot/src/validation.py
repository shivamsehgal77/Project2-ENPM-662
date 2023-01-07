
from IK_Nemerical_DH import *
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty


from robot_joint_publisher import *


if __name__ =="__main__":

    rospy.init_node("main_ur5v1")

    rate = rospy.Rate(10)
    joint_publisher = robot_joint_publisher()


    theta_zero = np.array([0,0,0,0,0,0])
    rospy.sleep(1.5)
    joint_publisher.publish_joint(theta_zero)