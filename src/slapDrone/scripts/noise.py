#! /usr/bin/env python
'''
	This node will publish the noise data from the Drone.
'''
import time
import serial
import sys
import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import Float32

if __name__ == "__main__":

	rospy.init_node("slap_noise")
	pub = rospy.Publisher("/slapDrone/noise", Float32, queue_size=100)
	rate = rospy.Rate(50)
	noise = Float32()
	port="/dev/ttyAMA1"
	usart=serial.Serial(port,9600,timeout=None)
	usart.flushInput()
	sendbuf = bytearray.fromhex("01 03 00 00 00 01 84 0A")
	while not rospy.is_shutdown():
		# print("--------------")
		usart.write(sendbuf)
		recvbuf = bytearray(usart.read(7))
		b1 = int(recvbuf[3])
		b0 = int(recvbuf[4])
		noise.data =((b1<<8) | b0)/10.0
		pub.publish(noise)
		print(noise.data)
		# time.sleep(.05)
	GPIO.cleanup()