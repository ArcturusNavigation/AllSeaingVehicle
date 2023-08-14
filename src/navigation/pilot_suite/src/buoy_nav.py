import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from utility.constants import IMG_WIDTH, IMG_HEIGHT

class BuoyNav():
    
    ANG_VEL_SCALE = 0.002
    LIN_SPEED = 2
    
    def __init__(self):
        self.center_sub = rospy.Subscriber(
            "/perception_suite/buoy_center",
            Float64,
            self.send_cmd_vel,
            queue_size=1
        )
        self.cmd_vel_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=1
        )

    def send_cmd_vel(self, center_x):
        center_x = center_x.data
        heading_error_px = IMG_WIDTH / 2 - center_x
        cmd_vel = Twist()
        cmd_vel.linear.x = self.LIN_SPEED
        cmd_vel.angular.z = heading_error_px * self.ANG_VEL_SCALE
        self.cmd_vel_pub.publish(cmd_vel)

if __name__ == "__main__":
    rospy.init_node("buoy_nav") 
    BuoyNav()
    rospy.spin()
