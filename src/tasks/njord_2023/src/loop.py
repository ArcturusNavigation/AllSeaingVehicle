import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class Loop():
    def __init__(self):
        self.state_sub = rospy.Subscriber(
            "/state",
            Int32,
            self.state_callback,
            queue_size=10
        )
        self.cmd_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=1         
        )

    def state_callback(self, state):
        if state.data == 1:
            cmd_vel = Twist()
            cmd_vel.angular.z = 0.6
            self.cmd_pub.publish(cmd_vel)

if __name__ == "__main__":
    rospy.init_node("loop")
    Loop()
    rospy.spin()
