#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Servo.h>
/*
On bootup:
- Turn on relay to thrusters
- Keep relay off to pump 
- move servos to neutral position 
Callbacks
- /watergun/yaw: value ()
- /watergun/pitch: value
- /watergun/fire: boolean
- /skeeball/release: boolean
- /skeeball/extend: boolean
- /skeeball/angle: value
*/

// ROS objects
ros::NodeHandle  nh;
//std_msgs::String str_msg;
//ros::Publisher debugLog("arduino/log", &str_msg);

//Relay pins and switches
int pump_relay = 11;
int thruster_relay = 12;
bool pump_switch = true;
bool thruster_switch = true;

// Watergun Pins
int yaw_pin = 10; //YAW, Servo 1 socket, on bottom of watergun, Digital 10
int pitch_pin = 9; //PITCH, Servo 2 socket, on top of watergun, Digital 9

// Skeeball Pins TODO
//int release_pin = 6; //PWM0A, Digital 6

// Watergun Servos
Servo yaw_servo; 
Servo pitch_servo;

//Skeeball Servos
//Servo release_servo;


/***********************************
*********WATER GUN FUNCTIONS********
***********************************/
void yaw_cb(const std_msgs::Float32& yaw){
  int servo_command = yaw.data;
  yaw_servo.write(servo_command);
}
void pitch_cb(const std_msgs::Float32& pitch){
  int servo_command = pitch.data;
  pitch_servo.write(servo_command);
}
void fire_cb(const std_msgs::Bool& fire){
  if(fire.data){
    digitalWrite(pump_relay, HIGH);
//    str_msg.data = "Firing water gun!";
  }
  else{
    digitalWrite(pump_relay, LOW);
//    str_msg.data = "Stopping water gun!";
  }
//  debugLog.publish(&str_msg);
}

/***********************************
*********SKEEBALL FUNCTIONS********
***********************************/
//void release_cb(const std_msgs::Bool& release){
//  if(release.data){
//    release_servo.write(90);
////    str_msg.data = "Releasing skeeball!";
//  }
//  else{
//    release_servo.write(0);
////    str_msg.data = "Stopping skeeball!";
//  }
//  debugLog.publish(&str_msg);
//}
//void extend_cb(const std_msgs::Bool& extend){
////  digitalWrite(extend_pin, HIGH);
////  delay(5000);
////  digitalWrite(extend_pin, LOW);
////  str_msg.data = "Extended skeeball arm";
////  debugLog.publish(&str_msg);
//}
//
//void angle_cb(const std_msgs::Float32& angle){
//  float servo_command = angle.data; //Fix this calculation
////  str_msg.data = "Rotating Skeeball Arm ";// TODO: + String(servo_command);
////  debugLog.publish(&str_msg);
//
//}

/***********************************
*********Safety FUNCTIONS********
***********************************/
void safety_cb(const std_msgs::Bool& safety){
  if(safety.data){
//    str_msg.data = "Cutting power to thrusters";
//    debugLog.publish(&str_msg);
    digitalWrite(thruster_relay, LOW);
  }
  else{
//    str_msg.data = "Powering thrusters";
//    debugLog.publish(&str_msg);
    digitalWrite(thruster_relay, HIGH);
  }
}

//Subscribers
ros::Subscriber<std_msgs::Float32> yaw_sub("watergun/yaw", &yaw_cb);
ros::Subscriber<std_msgs::Float32> pitch_sub("watergun/pitch", &pitch_cb);
ros::Subscriber<std_msgs::Bool> fire_sub("watergun/fire", &fire_cb);
//ros::Subscriber<std_msgs::Bool> release_sub("skeeball/release", &release_cb);
//ros::Subscriber<std_msgs::Float32> extend_sub("skeeball/extend", &extend_cb);
//ros::Subscriber<std_msgs::Float32> angle_sub("skeeball/angle", &angle_cb);
ros::Subscriber<std_msgs::Bool> safety_sub("/arcturus_pilot/off", &safety_cb);

/******************************
********* MAIN FUNCTIONS *******
******************************/
void setup()
{


  yaw_servo.attach(yaw_pin);
  pitch_servo.attach(pitch_pin);
//  release_servo.attach(release_pin);

  // Initialize pins 
  pinMode(pump_relay, OUTPUT);
  pinMode(thruster_relay, OUTPUT);;
//  pinMode(release_pin, OUTPUT);
  pinMode(yaw_pin, OUTPUT);
  pinMode(pitch_pin, OUTPUT);

  nh.initNode();
//  nh.advertise(debugLog);
  nh.subscribe(yaw_sub);
  nh.subscribe(pitch_sub);
  nh.subscribe(fire_sub);
//  nh.subscribe(release_sub);
//  nh.subscribe(extend_sub);
//  nh.subscribe(angle_sub);
  nh.subscribe(safety_sub);

  if(thruster_switch){
    digitalWrite(thruster_relay, HIGH);
  }
  else{
    digitalWrite(thruster_relay, LOW);
  }
    if(pump_switch){
    digitalWrite(pump_relay, HIGH);
  }
  else{
    digitalWrite(pump_relay, LOW);
  }
}

void loop()
{
  nh.spinOnce();
  Serial.println("Spinning");
  delay(1);
}
