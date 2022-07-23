#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
/*
On bootup:
- Turn on relay to thrusters
- Keep relay off to pump 
- move servos to neutral position 
Callbacks
- /watergun/angle1: value ()
- /watergun/angle2: value
- /watergun/fire: boolean
- /skeeball/release: boolean
- /skeeball/extend: boolean
- /skeeball/angle: value
*/

// ROS objects
ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher debugLog("arduino/log", &str_msg);
ros::Subscriber angle1_sub("watergun/angle1", &angle1_cb);
ros::Subscriber angle2_sub("watergun/angle2", &angle2_cb);
ros::Subscriber fire_sub("watergun/fire", &fire_cb);
ros::Subscriber release_sub("skeeball/release", &release_cb);
ros::Subscriber extend_sub("skeeball/extend", &extend_cb);
ros::Subscriber angle_sub("skeeball/angle", &angle_cb);
ros::Subscriber safety_sub("/arcturus_pilot/off", &safety_cb);

//Relay pins and switches
int pump_relay = 11;
int thruster_relay = 12;
bool pump_switch = false;
bool thruster_switch = true;

// Watergun Pins
int angle1_pin = 10; //YAW, Servo 1 socket, on bottom of watergun, Digital 10
int angle2_pin = 9; //PITCH, Servo 2 socket, on top of watergun, Digital 9

// Skeeball Pins TODO
int release_pin = 6; //PWM0A, Digital 6
int extend_pin = A4; // dc motor 
int angle_pin = A5; // stepper motor

//Watergun Servos
Servo angle1_servo;
Servo angle2_servo;

//Skeeball Servos
Servo release_servo;
Servo angle_servo; 


/***********************************
*********WATER GUN FUNCTIONS********
***********************************/
void angle1_cb(const std_msgs::Float32& angle1){
  int servo_command = angle1.data;
  angle1_servo.write(servo_command);
}
void angle2_cb(const std_msgs::Float32& angle2){
  int servo_command = angle2.data;
  angle2_servo.write(servo_command);
}
void fire_cb(const std_msgs::Bool& fire){
  if(fire.data){
    digitalWrite(pump_relay, HIGH);
    str_msg.data = "Firing water gun!";
  }
  else{
    digitalWrite(pump_relay, LOW);
    str_msg.data = "Stopping water gun!";
  }
  debugLog.publish(&str_msg);
}

/***********************************
*********SKEEBALL FUNCTIONS********
***********************************/
void release_cb(const std_msgs::Bool& release){
  if(release.data){
    release_servo.write(90);
    str_msg.data = "Releasing skeeball!";
  }
  else{
    release_servo.write(0);
    str_msg.data = "Stopping skeeball!";
  }
  debugLog.publish(&str_msg);
}
void extend_cb(const std_msgs::Bool& extend){
  digitialWrite(extend_pin, HIGH);
  delay(5000);
  digitalWrite(extend_pin, LOW);
  str_msg.data = "Extended skeeball arm";
  debugLog.publish(&str_msg);
}

void angle_cb(const std_msgs::Float32& angle){
  int servo_command = angle.data; //Fix this calculation
  angle_servo.write(servo_command);
  str_msg.data = "Skeeball arm angle: " + String(servo_command);
  debugLog.publish(&str_msg);

}

/***********************************
*********Safety FUNCTIONS********
***********************************/
void safety_cb(const std_msgs::Bool& safety){
  if(safety.data){
    str_msg.data = "Cutting power to thrusters";
    debugLog.publish(&str_msg);
    digitalWrite(thruster_relay, LOW);
  }
  else{
    str_msg.data = "Powering thrusters";
    debugLog.publish(&str_msg);;
    digitalWrite(thruster_relay, HIGH);
  }
}

/******************************
********* MAIN FUNCTIONS *******
******************************/
void setup()
{
  Serial.begin(9600);
  pinMode(kill_pin, OUTPUT);

  angle1_servo.attach(angle1_pin);
  angle2_servo.attach(angle2_pin);
  release_servo.attach(release_pin);
  angle_servo.attach(angle_pin);

  // Initialize pins 
  pinMode(pump_relay, OUTPUT);
  pinMode(thruster_relay, OUTPUT);
  pinMode(pump_relay, OUTPUT);
  pinMode(release_pin, OUTPUT);
  pinMode(extend_pin, OUTPUT);
  pinMode(angle_pin, OUTPUT);
  pinMode(angle1_pin, OUTPUT);
  pinMode(angle2_pin, OUTPUT);

  nh.initNode();
  nh.advertise(debugLog);
  nh.subscribe(angle1_sub);
  nh.subscribe(angle2_sub);
  nh.subscribe(fire_sub);
  nh.subscribe(release_sub);
  nh.subscribe(extend_sub);
  nh.subscribe(angle_sub);
  
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
