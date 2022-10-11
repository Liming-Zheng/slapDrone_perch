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
from adafruit_servokit import ServoKit
import time

def servo_rotate(kit, channels, direction, num):
    """
    # 对指定的通道进行旋转，指定转动方向和圈数
    # direction=1为放松，-1为拉紧
    # num=6是一个合适的选择
    """
    kit.continuous_servo[channels].throttle = direction
    # 这里通过统计得到转动圈数与时间的关系
    time.sleep(num*2/3)
    # 这里一定要把throttle设置为0，目的是停止转动舵机
    kit.continuous_servo[channels].throttle = 0
    time.sleep(2)

def doGrasper_callback(msg):

    kit = ServoKit(channels=16)

    if msg.data == "T_off":
        kit.servo[0].angle = 65 # off的角度
    elif msg.data == "T_on":
        kit.servo[0].angle = 40 # on的角度
    elif msg.data == "S_open":
        servo_rotate(kit,channels=3,direction=-1,num=6)
    elif msg.data == "S_curl":
        servo_rotate(kit, channels=3, direction=1, num=6)
    elif msg.data == "perch":
        kit.servo[0].angle = 65 # off的角度
    elif msg.data == "retake off":
        servo_rotate(kit,channels=3,direction=-1,num=6)
        time.sleep(2)
        kit.servo[0].angle = 40 # on的角度
    else:

        pass
        


if __name__ =="__main__":
    rospy.init_node("grasper_action")
    sub_grasper = rospy.Subscriber("grasper_state", String, doGrasper_callback, queue_size=1)
   
    rospy.spin()