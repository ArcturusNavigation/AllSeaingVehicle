#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from mavros_msgs.msg import State

GUIDED = 23
MANUAL = 12

def callback(data):
    rospy.loginfo(f"Guided?: {data.guided}")
    rospy.loginfo(f"Manual?: {data.manual_input}")

    if data.guided:
        GPIO.output(GUIDED, GPIO.HIGH)
    else:
        GPIO.output(GUIDED, GPIO.LOW)

    if data.manual_input:
        GPIO.output(MANUAL, GPIO.HIGH)
    else:
        GPIO.output(MANUAL, GPIO.LOW)

def listener():
    rospy.init_node("mode_listener") 
    rospy.Subscriber("/mavros/state", State, callback)
    rospy.spin()

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM) 
    GPIO.setup(GUIDED, GPIO.OUT)
    GPIO.setup(MANUAL, GPIO.OUT)
    listener()
