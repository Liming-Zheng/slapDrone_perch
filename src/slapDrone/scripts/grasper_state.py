#! /usr/bin/env python
"""
    this is a ros node to publish a grasper state:
        "T_off" : trigger off, then the grasper curls
        "T_on" : trigger on, keep the grasper on open state
        "S_open" : slap bracelet will open
        "S_curl" : slap bracelet will curl
        "perch" : start to perch
        "retake off" : take off from the branch

"""
import rospy
from std_msgs.msg import String




if __name__ =="__main__":
    rospy.init_node("grasper_state")
    pub_state = rospy.Publisher("grasper_state", String, queue_size=10)
    # sub_drone = rospy.
    msg = String()
    rate = rospy.Rate(10)


    
    while not rospy.is_shutdown():

        msg.data = "T_off"
        pub_state.publish(msg)
        rate.sleep()
        rospy.loginfo("grasper_state: %s", msg.data)
