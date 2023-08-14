import rospy
from perception_suite.msg import LabeledBoundingBox2DArray
from std_msgs.msg import Int32

class ActionPlanner():
    def __init__(self):
        self.start = 0 
        self.state = 0
        self.start_bboxes = rospy.Time.now()
        self.start_no_bboxes = rospy.Time.now()
        self.state_pub = rospy.Publisher(
            "/state",
            Int32,
            queue_size = 10,
        )
        self.bbox_sub = rospy.Subscriber(
            "perception_suite/bounding_boxes",
            LabeledBoundingBox2DArray,
            self.bbox_callback,
        )
        rospy.Timer(rospy.Duration(0.1), self.publish_state)

    def publish_state(self, event=None):
        self.state_pub.publish(self.state)

    def bbox_callback(self, bboxes):
        
        if self.state == 0:
            
            if len(bboxes.boxes) > 0:
                self.start_bboxes = bboxes.header.stamp
            else:
                self.start_no_bboxes = bboxes.header.stamp

            if self.start_no_bboxes - self.start_bboxes > rospy.Duration(0.1):
                self.state += 1
                
if __name__ == "__main__":
    rospy.init_node("action_planner")
    action = ActionPlanner()
    rospy.spin()
