#! /usr/bin/env python3

# 180度舵机
from adafruit_servokit import ServoKit
import time
import rospy
from std_msgs.msg import String

slap_grasp = ServoKit(channels=16)
pre_data = ""

def servo_180(kit, channels, angle):

    kit.servo[channels].actuation_range = 180
    kit.servo[channels].set_pulse_width_range(500, 2300)
    kit.servo[channels].angle = angle
    global flag_safe_string
    flag_safe_string = True

# 360舵机
def servo_360(kit, channels, direction, num):
    """
    # 对指定的通道进行旋转，指定转动方向和圈数
    # direction = -1为放松，1为拉紧
    # num=6 是一个合适的选择 +-1 are same：{37.5圈/60s，也就是说一圈1.6s}
    """
    kit.continuous_servo[channels].throttle = direction
    # 这里通过统计得到转动圈数与时间的关系
    time.sleep(num*time_one_round)
    # 这里一定要把throttle设置为0，目的是停止转动舵机
    kit.continuous_servo[channels].throttle = servo_throttle
    # time.sleep(2)
    global flag_safe_string
    flag_safe_string = False

def doMsg(msg):
    global pre_data
    if pre_data != msg.data:
        pre_data = msg.data
        if msg.data == "trigger_on":
            rospy.loginfo("########### trigger_on start ###########")
            servo_180(slap_grasp, channel_180, trigger_on_angle)
            rospy.loginfo("########### trigger_on end ###########")

        elif msg.data == "trigger_off":
            rospy.loginfo("########### trigger_off start ###########")
            servo_180(slap_grasp, channel_180, trigger_off_angle)
            rospy.loginfo("########### trigger_off end ###########")

        elif msg.data == "roll_open":
            if flag_safe_string:
                rospy.loginfo("########### roll_open start ###########")
                servo_360(slap_grasp, channel_360, 1, num_open)
                rospy.loginfo("########### roll_open end ###########")
            else:
                rospy.loginfo(">>>>>>>>>> you cant use same command twice, that can damage the grasper <<<<<<<<<<")

        elif msg.data == "roll_close":
            if flag_safe_string:
                rospy.loginfo("########### roll_close start ###########")
                servo_360(slap_grasp,channel_360, -1, num_close)
                rospy.loginfo("########### roll_close end ###########")
            else:
                rospy.loginfo(">>>>>>>>>> you cant use same command twice, that can damage the grasper <<<<<<<<<<")

        elif msg.data == "perching":
            rospy.loginfo("########### perching start ###########")
            servo_180(slap_grasp, channel_180, trigger_off_angle)
            rospy.loginfo("########### perching end ###########")
        
        elif msg.data == "take_off":
            if flag_safe_string:
                rospy.loginfo("########### take off start ###########")
                servo_360(slap_grasp, channel_360, 1, num_open)
                time.sleep(2)
                servo_180(slap_grasp, channel_180, trigger_on_angle)
                time.sleep(2)
                servo_360(slap_grasp,channel_360, -1, num_close)
                # servo_180(slap_grasp, channel_180, trigger_on_angle)
                rospy.loginfo("########### take off end ###########")
            else:
                rospy.loginfo(">>>>>>>>>> you cant use same command twice, that can damage the grasper <<<<<<<<<<")

        else:
            rospy.loginfo("########### NO ACTION!!!! ###########")
    else:
        rospy.loginfo(">>>>>>>>>> you cant use same command twice, that can damage the grasper <<<<<<<<<<")
        pre_data = msg.data


if __name__ == "__main__":

    rospy.init_node("grasper_action")


    global time_one_round
    time_one_round = rospy.get_param("time_one_round", 1.6)
    global num_close
    num_close = rospy.get_param("num_close", 4.0)
    global num_open
    num_open = rospy.get_param("num_open", 4.5)
    global trigger_on_angle
    trigger_on_angle = rospy.get_param("trigger_on_angle", 95.0)
    global trigger_off_angle
    trigger_off_angle = rospy.get_param("trigger_off_angle", 130.0)
    global servo_throttle
    servo_throttle = rospy.get_param("servo_throttle", -0.1)
    global channel_180
    channel_180 = rospy.get_param("channel_180", 1)
    global channel_360
    channel_360 = rospy.get_param("channel_360", 6)

    sub_grasper = rospy.Subscriber("slap_grasper_command", String, doMsg, queue_size=1)
    rospy.spin()