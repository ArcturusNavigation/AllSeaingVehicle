#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

def shoot():
    pub= rospy.Publisher("/pilot_suite/water_gun_task/shooter_pose", Point, queue_size=10)
    rospy.init_node("shooter_test", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        target = Point()
        target.x = 0
        target.y = 0
        target.z = 0
        pub.publish(target) 
        rate.sleep()

if __name__ == '__main__':
    try:
        shoot()
    except rospy.ROSInterruptException:
        pass
