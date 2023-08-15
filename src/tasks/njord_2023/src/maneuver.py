import rospy
from mavros_msgs.srv import CommandLong

STRAIGHT_SEC = 2
ROT_SEC = 5

def send_pwm(channel, value):
    proxy = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
    print(f"Sending pwm value {value} to channel {channel}")
    return proxy(
        command=183,
        param1=float(channel),
        param2=float(value)
    ).success
        
def move():

    rospy.init_node("loop")
    start = rospy.Time.now()

    while rospy.Time.now() - start < rospy.Duration(2.5):
                      
        #send_pwm(channel=2, value=1900)
        #send_pwm(channel=3, value=1900)

        send_pwm(channel=2, value=1550)
        send_pwm(channel=3, value=1550)

    while rospy.Time.now() - start < rospy.Duration(7.5):

        #send_pwm(channel=2, value=1900)
        #send_pwm(channel=3, value=1100)

        send_pwm(channel=2, value=1550)
        send_pwm(channel=3, value=1450)

    while rospy.Time.now() - start < rospy.Duration(10):

        #send_pwm(channel=2, value=1900)
        #send_pwm(channel=3, value=1900)

        send_pwm(channel=2, value=1550)
        send_pwm(channel=3, value=1550)

if __name__ == "__main__":
    move()
