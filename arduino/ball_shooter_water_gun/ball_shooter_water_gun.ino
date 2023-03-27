#include <ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <Servo.h>
#include <AFMotor.h>
#include <std_msgs/Empty.h>

ros::NodeHandle node_handle;
std_msgs::Bool validity_msg;
geometry_msgs::Point point_msg;


//water gun setup
Servo water_yaw_servo;
Servo water_pitch_servo; 
bool pump_toggle = false;
int water_yaw_servo_pin = 10; 
int water_pitch_servo_pin = 11; 
int pump_pin = 8;
int pump_count = 0; // indicates that pump starts as off
int water_pitch_zero = 0; //indicates what angle the water pitch is at when straight
int water_yaw_zero = 45; // will find value when we mount to boat
unsigned long time = 0;  

//ball shooter setup
Servo ball_aim_servo; 
int SENSOR_PIN = 0; // center pin of the potentiometer
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
int ball_aim_servo_pin = 7; 

void statusCB( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

void subscriberCallback(const geometry_msgs::Point& point_msg) {
	if (point_msg.x == None || point_msg.y == None || point_msg.z == None) {
		validity_msg = false;			
		pump();
	} else {
		validity_msg = true;
		watergun_aim_shoot(point.msg.x, point.msg.y, point.msg.z); 
	}
}

ros::Publisher validity_publisher("/pilot_suite/water_gun_task/arduino/pump_activator", &validity_msg);
ros::Subscriber<geometry_msgs::Point> point_subscriber("/pilot_suite/water_gun_task/target_center_pose");
ros::Subscriber<std_msgs::Empty> status_sub("toggle_status", &statusCB );


void setup() {
	water_yaw_servo.attach(water_yaw_servo_pin); // left, right
	water_pitch_servo.attach(water_pitch_servo_pin); // up, down
	ball_aim_servo.attach(ball_aim_servo_pin); 
	pinMode(pump_pin, OUTPUT);
	pinMode(RPWM_Output, OUTPUT);
	pinMode(LPWM_Output, OUTPUT);
	Serial.begin(9600);

	// Setting up the ros node
	node_handle.initNode();
	node_handle.advertise(validity_publisher)
	node_handle.subscribe(point_subscriber);
	node_handle.subscribe(status_sub);
}

void loop() { 
	
	validity_publisher.publish(&validity_msg);
	node_handle.spinOnce();
}

void parse_input(){
	String inByte = Serial.readStringUntil('\n'); 
	if (inByte.length() ==1){ // deals with single letter functions
		if (inByte == "P"){ //activate water pump
			pump(); 
		}
		if(inByte == "B"){ // fire a ball
			ball_shoot();
		}
		if(inByte == "D"){ // read current pos
			read_curr_pos(); 
		}
		if(inByte == "R"){ //reset to zeroed values
			water_yaw_servo.write(water_yaw_zero); 
			water_pitch_servo.write(water_pitch_zero); 
		}
		else{
			//Parse x,y,z,mode type input
			//Note: input must be in format x,y,z,mode, with NO SPACES 

			//find x value
			int index = inByte.indexOf(","); 
			float x = inByte.substring(0, index).toFloat();
			// find y value
			String rest = inByte.substring(index+1, -1);
			index = rest.indexOf(","); 
			float y = rest.substring(0, index).toFloat();
			//find z value 
			rest = rest.substring(index+1, -1);
			index = rest.indexOf(","); 
			float z = rest.substring(0, index).toFloat(); 
			// find the mode or display formatting error
			rest = rest.substring(index+1, -1); 
			if (rest.length()!= 1){
				Serial.println("Invalid Format");
			}
			else{
				String mode = rest; 
				if (mode == "P"){
					watergun_aim_shoot(x,y,z); 
				}
				if (mode =="B"){
					ballshooter_aim_shoot(x,y,z); 
				}
			}
		}
	}
}
// x, y, z relative to the water gun
void watergun_aim_shoot(float x, float y, float z){
	//find theta values through trig
	//dependent on zeroed angle values
	float theta1 = atan(x/z); 
	float theta2 = atan(y/z); 
	theta1 = water_yaw_zero + theta1*180/3.14; 
	theta2 = water_pitch_zero + theta2*180/3.14; 
	water_yaw_servo.write(int(theta1)); 
	water_pitch_servo.write(int(theta2)); 
	delay(1000); // not sure if we need a delay, just thought we should give it a sec
	pump(); //turn water pump on 
}

void ballshooter_aim_shoot(float x, float y, float z){
	// find ball shooter theta val
	// need to think a little more about ideal angle since we only have pitch and no yaw
	float theta = atan(x/y); 
	ball_aim_servo.write(int(theta)); 
	delay(1000); 
	ball_shoot(); 
}

void ball_shoot(){
	// fire ball shooter
	int sensorValue = 1023;
	// sensor value is in the range 0 to 1023
	// the lower half of it we use for reverse rotation; the upper half for forward rotation
	if (sensorValue < 512){
		// reverse rotation
		int reversePWM = -(sensorValue - 511) / 2;
		analogWrite(LPWM_Output, 0);
		analogWrite(RPWM_Output, reversePWM);
	}
	else{
		// forward rotation
		int forwardPWM = (sensorValue - 512) / 2;
		analogWrite(RPWM_Output, 0);
		analogWrite(LPWM_Output, forwardPWM);
	}
	Serial.println("Ball shooter fired!"); 
}

void pump(){
	//turn on the water pump 
	pump_toggle = !pump_toggle;
	digitalWrite(8, pump_toggle);
	if(pump_toggle == false){
		time = 0; 
		Serial.println("pump turned OFF"); 
	}
	else{
		time = millis(); // starts pump timer
		Serial.println("pump turned ON"); 
	}
}

void read_curr_pos(){

	// prints out current water gun pos
	int yaw_pos = water_yaw_servo.read(); 
	int pitch_pos = water_pitch_servo.read(); 
	Serial.println("Current Water Gun Angles: ( " + String(yaw_pos) +", " + String(pitch_pos)+ " )"); //print water gun angles
}


