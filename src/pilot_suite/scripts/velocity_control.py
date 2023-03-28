#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('constant_velocity', anonymous=True)
    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
    vel_msg = Twist() 
    #Receiveing the user's input
    print("Let's move Ship Happens!")
    speed = float(input("Input your speed (m/s):"))
    desired_time = float(input("How Long Do You Want to Drive (s):"))
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
    desired_time += rospy.Time.now().to_sec()

    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        current_time = float(rospy.Time.now().to_sec())
        print("Current time:", current_time)
        print("Desired time:", desired_time)

        #Loop to move the turtle in an specified distance
        if (current_time < desired_time):
            
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
        else:  
            #After the loop, stops the robot
            vel_msg.linear.x = 0
            #Force the robot to stop
            #velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
