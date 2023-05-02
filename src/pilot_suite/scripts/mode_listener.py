import rospy
import RPi.GPIO as GPIO
from mavros_msgs.msg import State

SIG_PIN = 23

def callback(data):
    rospy.loginfo(f"Guided?: {data.guided}")
    rospy.loginfo(f"Manual?: {data.manual_input}")

    if data.guided:
        GPIO.output(SIG_PIN, GPIO.HIGH)
    else:
        GPIO.output(SIG_PIN, GPIO.LOW)

def listener():
    rospy.init_node("mode_listener") 
    rospy.Subscriber("/mavros/state", State, callback)
    rospy.spin()

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM) 
    GPIO.setup(SIG_PIN, GPIO.OUT)
    listener()