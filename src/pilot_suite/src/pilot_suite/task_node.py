import rospy 
from std_msgs.msg import String

class TaskNode():
    def __init__(self, task_name, loop_freq=30):
        self.task_name = task_name 
        self.active = False 
        self.rate = rospy.Rate(loop_freq)
        rospy.Subscriber('/pilot_suite/task', String, self.toggle_callback)
        self.status_pub = rospy.Publisher('/pilot_suite/task_status', String, queue_size=10)

    def toggle_callback(self, msg):
        if msg.data[1:] == self.task_name:
            self.active = True #msg.data[0] == "+"
        rospy.loginfo(f"Task {self.task_name} status switched to {self.active}")

    def complete(self):
        self.status_pub.publish(self.task_name) 

    def run(self):
        raise NotImplementedError("TaskNode.run() not implemented")
