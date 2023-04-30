#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandLong
from pilot_suite.srv import PwmCommand

def send_pwm(req):
    proxy = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
    print(f"Sending pwm value {req.value} to channel {req.channel}")
    return proxy(
        command=183,
        param1=float(req.channel),
        param2=float(req.value)
    ).success
    
def pwm_server():
    rospy.init_node("pwm_sender")
    server = rospy.Service("/pilot_suite/request_pwm", PwmCommand, send_pwm)
    rospy.spin()

if __name__ == '__main__':
    pwm_server()