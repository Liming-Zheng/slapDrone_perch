#! /usr/bin/env python
'''
	This node will publish the noise data from the Drone.
'''
import time
import serial
import sys
import RPi.GPIO as GPIO

port="/dev/ttyAMA1"
usart=serial.Serial(port,9600,timeout=None)
usart.flushInput()
sendbuf = bytearray.fromhex("01 03 00 00 00 01 84 0A")
while True:
	print("--------------")
	usart.write(sendbuf)
	recvbuf = bytearray(usart.read(7))
	b1 = int(recvbuf[3])
	b0 = int(recvbuf[4])
	noise =((b1<<8) | b0)/10.0
	
	print("Noise: ",noise)
		# time.sleep(.05)
GPIO.cleanup()