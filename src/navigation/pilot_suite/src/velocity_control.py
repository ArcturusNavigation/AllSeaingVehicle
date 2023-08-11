#!/usr/bin/env python
import rospy
from pilot_suite.srv import PwmCommand

def move_client():

    # Node initialization
    rospy.init_node("velocity_control")

    # Receiving user input
    print("Let's move Ship Happens!")
    pwm = int(input("Input your pwm value: "))
    desired_time = float(input("How Long Do You Want to Drive (s): "))
    current_time = rospy.Time.now().to_sec()
    desired_time += current_time

    rospy.wait_for_service("/pilot_suite/request_pwm")

    try:
        request_pwm = rospy.ServiceProxy("/pilot_suite/request_pwm", PwmCommand)
        while current_time < desired_time:

            current_time = float(rospy.Time.now().to_sec())
            resp1 = request_pwm(channel=2, value=pwm)
            resp2 = request_pwm(channel=3, value=pwm)

            print("Current time:", current_time)
            print("Desired time:", desired_time)
            print("Service response:", resp1 and resp2)

        request_pwm(channel=2, value=1500)
        request_pwm(channel=3, value=1500)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}") 

if __name__ == '__main__':
    move_client()
