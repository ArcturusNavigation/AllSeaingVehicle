#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;
  
std_msgs::String str_msg;
ros::Publisher testPub("arduino/test_pub", &str_msg);

void testSubCb(const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));
  str_msg.data = "Test test test?";
  testPub.publish(&str_msg);
}

ros::Subscriber<std_msgs::Empty> testSub("arduino/test_sub", &testSubCb);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(testPub);
  nh.subscribe(testSub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
