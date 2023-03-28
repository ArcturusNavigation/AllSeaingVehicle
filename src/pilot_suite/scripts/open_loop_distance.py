#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('open_loop_velocity', anonymous=True)
    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move Ship Happens Some Distance....Kinda!")
    speed = float(input("Input your speed (m/s):"))
    distance = float(input("Type your Desired Distance (m):"))
    isForward = input("Foward?: ")#True or False

    #Checking if the movement is forward or backwards
    if(isForward == "True"):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    rate = rospy.Rate(10)

    #Setting the current time for distance calculus
    current_distance = 0
    t0 = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():

        #Loop to move the turtle in an specified distance
        if (current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        else:
            #After the loop, stops the robot
            vel_msg.linear.x = 0
            #Force the robot to stop
            velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
