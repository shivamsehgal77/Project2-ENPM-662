# import rospy
from IK_Nemerical_DH import *
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from robot_joint_publisher import *

# Functions to call ros service for vacuum gripper

def gripper_on():
    # Wait till the srv is available
    rospy.wait_for_service("/robot/vacuum_gripper/on")
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/robot/vacuum_gripper/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)

def gripper_off():
    # Wait till the srv is available
    rospy.wait_for_service("/robot/vacuum_gripper/off")
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/robot/vacuum_gripper/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)


# Main
if __name__ =="__main__":

    #initializing the node 
    rospy.init_node("main_ur5_mobile_robot")

    rate = rospy.Rate(10)
    joint_publisher = robot_joint_publisher()

    #The know position for thetas to pick and place the object
    theta_pick      = np.array([0.8420926, -0.6288,    1.2288,   0.970846,  -1.5707+pi, .72872+pi])
    theta_lean = np.array([ pi/2, -pi/1.5,  pi/1.5, -pi/16, pi/2, pi/2])
    theta_place     = np.array([ 0.8420926+pi/3, -0.6288,    1.2288,   0.970846,  -1.5707+pi, .72872+pi])

    # Home position for the robot
    theta_zero = np.array([0,0,0,0,0,0])
    rospy.sleep(1.5)
    print("\nMoving Robot to Zero location   : ", theta_pick)
    joint_publisher.publish_joint(theta_zero)


    #Seting up the Forward kinematics, DH table
    Tsd_Pick,  _ = FKinDHParam(theta_pick, Table_DHParam)
    Tsd_Lean,  _ = FKinDHParam(theta_lean, Table_DHParam)
    Tsd_Place, _ = FKinDHParam(theta_place, Table_DHParam)

    print(theta_pick)
    print(Tsd_Pick.flatten())
    

    # Putting in the approximate guess values for the pick and place for newton rapson method
    theta_pick_guess     = np.array([-2.2995+pi, -0.6288,    1.2288,   0.970846,  -1.5707+pi, .72872+pi])
    theta_lean_guess     = np.array([pi/1.5, -pi/1.5,  pi/1.6, -pi/16, pi/2, pi/2])
    theta_place_guess    = np.array([0.8420926+pi/3, -0.6288,    1.2288,   0.970846,  -1.5707+pi, .72872+pi])


    #Calculating the inverse kinematics for the pick place and guess using the inverse kinematics solver

    print("\nActual Pick location   : ", theta_pick)
    print("Guess Pick location    : ", theta_pick_guess)
    print("Running IK for Pick location : ")
    theta_pick_sol, done = ik(Table_DHParam,Tsd_Pick, theta_pick_guess, 0.0001)
    if done:
        print("Computed Pick location : ", theta_pick_sol)
    else:
        print("IK failed for Pick")

    print("\nActual Lean location   : ", theta_lean)
    print("Guess Lean location    : ", theta_lean_guess)
    print("Running IK for Lean location : ")
    theta_lean_sol, done = ik(Table_DHParam,Tsd_Lean, theta_lean_guess, 0.0001)
    
    if done:
        print("Computed Lean location : ", theta_lean_sol)
    else:
        print("IK failed for Lean")


    print("\nActual Place location   : ", theta_place)
    print("Guess Place location    : ", theta_place_guess)
    print("Running IK for Place location : ")
    theta_place_sol, done = ik(Table_DHParam,Tsd_Place,theta_place_guess, 0.0001)
    if done:
        print("Computed Place location : ", theta_place_sol)
    else:
        print("IK failed for Place")


    #Publishing the Pick Place lean and home joint angles to the joint publisher
    rospy.sleep(1.5)
    joint_publisher.publish_joint(theta_pick_sol)
    a = gripper_on()
    rospy.sleep(1.5)
    joint_publisher.publish_joint(theta_place_sol)
    rospy.sleep(2)
    b = gripper_off()
    rospy.sleep(1.5)
    joint_publisher.publish_joint(theta_lean_sol)
    rospy.sleep(1.5)
    joint_publisher.publish_joint(theta_zero)

    


    print("Pick and Place completed not return to home")

    while not rospy.is_shutdown():

        rate.sleep()
    


