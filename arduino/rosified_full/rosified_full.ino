// Import libraries
#include <ros.h>
#include <ArduPID.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <Servo.h>
#include <AFMotor.h>

// Define pins
#define HALL_EFFECT_BACK 14
#define HALL_EFFECT_FRONT 15
#define HOPPER_SERVO 5
#define BALL_COLLECT_LIMIT_UP 1
#define BALL_COLLECT_LIMIT_DOWN 2
#define BALL_COLLECT_LPWM 3
#define BALL_COLLECT_RPWM 4
#define BALL_COLLECT_SERVO 2
#define BALL_AIM_SERVO 6
#define SENSOR_PIN 0
#define SHOOTER_RPWM 7
#define SHOOTER_LPWM 8
#define WATER_YAW_SERVO 10
#define WATER_PITCH_SERVO 11
#define PUMP 13
#define CH_A 20
#define CH_B 21

// Servo setup
Servo collectorServo;
Servo hopperServo;
Servo ballAimServo;
const int COLLECT_MOTOR_SPEED = 50; // MAX 255
const int COLLECT_SERVO_SPEED = 50; // MAX 360
const int REVERSE_TIME = 5000; // Time the collector will be in reverse mode
const int FORWARD_TIME = 5000; // Time the collector will be in forward mode
unsigned long timeForward = 0;
unsigned long timeReverse = 0;

// Encoder counter
volatile long counter = 0;
long prevCounter = 0;
unsigned long lastTime = 0;
const double PPR = 28.0;
const int TIME_INTERVAL = 100;
const int GEAR_RATIO = 2;
double rpm = 0;

// Ball shooter hopper movement
const int NUM_ROTATION = 3;
const int HOPPER_SERVO_SPEED = 110;
const double SHOOTER_SPEED = 1000;
unsigned long timeHopperMove = 0;
long deltaCounter = 0;
int rotateCounter = 0;
double shooterSpeed = 0;
const int SPEED_THRESHOLD = 50;
ArduPID shooterPID;
const double shooterP = 0.15;
const double shooterI = 0.001;
const double shooterD = 0.005;

// Water gun setup
const int WATER_PITCH_ZERO = 60;
const int WATER_YAW_ZERO = 135;
Servo waterYawServo;
Servo waterPitchServo; 
unsigned long pumpTime = 0;  
bool pump_toggle = false;

//--------- Modes for tasks ---------//

enum WatergunModes {

	SHOOT,
	ZERO

};

enum CollectorModes {

	COLLECT,
	MOVE_DOWN,
	MOVE_UP,
	INACTIVATED,
	REVERSE

};

enum ShooterModes {

	SPEED_UP,
	ROTATE,
	STOPPED	

};

CollectorModes collectorMode = INACTIVATED;
ShooterModes shooterMode = STOPPED;
WatergunModes watergunMode = ZERO;

//---------- ROS setup ----------//

// Messages
ros::NodeHandle nh;
geometry_msgs::Point waterPointMsg;
geometry_msgs::Point ballPointMsg;
std_msgs::Bool collectorMsg;
const int CALLBACK_THRESHOLD = 1; //TODO: Check what value works well

int waterRunCount = 0;
int waterStopCount = CALLBACK_THRESHOLD;
void watergunCallback(const geometry_msgs::Point& waterPointMsg) {

	if (waterPointMsg.x >= 10000 || waterPointMsg.y >= 10000 || waterPointMsg.z >= 10000) {
		waterRunCount = 0;
		waterStopCount++;
	} else {
		waterRunCount++;
		waterStopCount = 0;
	}

	if (waterRunCount == 1) {
		digitalWrite(LED_BUILTIN, HIGH);
		watergunMode = SHOOT;
	} else if (waterStopCount >= CALLBACK_THRESHOLD) {
		digitalWrite(LED_BUILTIN, LOW);
		watergunMode = ZERO;	
	}

	// Water gun modes
	if (watergunMode == ZERO) {
		zeroWater();
	} else if (watergunMode == SHOOT) {
		watergunAimShoot(waterPointMsg.x, waterPointMsg.y, waterPointMsg.z);
	} else {
		//Serial.println("rip watergun");
	}

}

int shooterRunCount = 0;
int shooterStopCount = CALLBACK_THRESHOLD;
void ballshooterCallback(const geometry_msgs::Point& ballPointMsg) {

	if (ballPointMsg.x >= 10000 || ballPointMsg.y >= 10000 || ballPointMsg.z >= 10000) {
		shooterStopCount++;
		shooterRunCount = 0;
	} else {
		shooterRunCount++;
		shooterStopCount = 0;
	}

	if (shooterRunCount == 1) {
		shooterMode = SPEED_UP;
	} 
	if (shooterStopCount >= CALLBACK_THRESHOLD) {
		shooterMode = STOPPED;
	}

	// Hall effect sensor values
	int hallEffectBackVal = digitalRead(HALL_EFFECT_BACK);
	int hallEffectFrontVal = digitalRead(HALL_EFFECT_FRONT);

	// Ball shooter modes
	shooterPID.compute();
	if (shooterMode == SPEED_UP) {

		shooterSpeedUp();
		hopperServo.write(90);
		shooterPID.start();
		aimShooter(ballPointMsg.x, ballPointMsg.y, ballPointMsg.z);

		// Rotate hopper when speed is in threshold
		if (SHOOTER_SPEED - SPEED_THRESHOLD < rpm && rpm < SHOOTER_SPEED + SPEED_THRESHOLD) { 

			timeHopperMove = millis();
			shooterMode = ROTATE;

		}

	} else if (shooterMode == ROTATE) {

		hopperServo.write(HOPPER_SERVO_SPEED);
		aimShooter(ballPointMsg.x, ballPointMsg.y, ballPointMsg.z);

		if ((hallEffectFrontVal == LOW || hallEffectBackVal == LOW) &&
				millis() - timeHopperMove > 500) {

			rotateCounter++;
			if (rotateCounter <= NUM_ROTATION)
				shooterMode = SPEED_UP;
			else
				shooterMode = STOPPED;

		}

	} else if (shooterMode == STOPPED) {

		hopperServo.write(90);
		rotateCounter = 0;
		stopShoot();
		shooterPID.stop();

	} else {
		//Serial.println("sketchy shooter mode");
	}

}

int collectorRunCount = 0;
int collectorStopCount = CALLBACK_THRESHOLD;
void collectorCallback(const std_msgs::Bool& collectorMsg) {

	if (collectorMsg.data) {
		collectorStopCount = 0;
		collectorRunCount++;
	} else {
		collectorStopCount++;
		collectorRunCount = 0;
	}

	if (collectorRunCount == 1) {
		collectorMode = MOVE_DOWN;
	}
	if (collectorStopCount >= CALLBACK_THRESHOLD) {
		collectorMode = INACTIVATED;
		collectorRunCount = 0;
	}

	// Limit switch values
	int collectorLimitUpVal = digitalRead(BALL_COLLECT_LIMIT_UP);
	int collectorLimitDownVal = digitalRead(BALL_COLLECT_LIMIT_DOWN);

	// Ball collector modes
	if (collectorMode == INACTIVATED) {

		analogWrite(BALL_COLLECT_LPWM, 0);
		analogWrite(BALL_COLLECT_RPWM, 0);
		collectorServo.write(0);

	} else if (collectorMode == COLLECT) {

		analogWrite(BALL_COLLECT_LPWM, 0);
		analogWrite(BALL_COLLECT_RPWM, 0);
		collectorServo.write(COLLECT_SERVO_SPEED);
		if (millis() - timeForward > FORWARD_TIME) {
			collectorMode = MOVE_UP;
		}

	} else if (collectorMode == MOVE_UP) {

		analogWrite(BALL_COLLECT_LPWM, COLLECT_MOTOR_SPEED);
		analogWrite(BALL_COLLECT_RPWM, 0);
		collectorServo.write(0);
		if (collectorLimitUpVal == HIGH) {
			collectorMode = REVERSE;
			timeReverse = millis();
		}

	} else if (collectorMode == MOVE_DOWN) {

		analogWrite(BALL_COLLECT_LPWM, 0);
		analogWrite(BALL_COLLECT_RPWM, -COLLECT_MOTOR_SPEED);
		collectorServo.write(0);
		if (collectorLimitDownVal == HIGH) {
			collectorMode = COLLECT;
			timeForward = millis();
		}

	} else if (collectorMode == REVERSE) {

		analogWrite(BALL_COLLECT_LPWM, 0);
		analogWrite(BALL_COLLECT_RPWM, 0);
		collectorServo.write(-COLLECT_SERVO_SPEED);
		if (millis() - timeReverse > REVERSE_TIME) {
			collectorMode = INACTIVATED;
		}

	} else {
		//Serial.println("wth is going on with collector mode");
	}

}

// Subscribers and publishers
ros::Subscriber<geometry_msgs::Point> waterPointSubscriber("/pilot_suite/water_gun_task/target_center_pose", &watergunCallback);
ros::Subscriber<geometry_msgs::Point> ballPointSubscriber("/pilot_suite/water_gun_task/shooter_pose", &ballshooterCallback);

//---------- Main setup ----------//

void setup() {

	// For debugging
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Initialize PID
	shooterPID.begin(&rpm, &shooterSpeed, &SHOOTER_SPEED, shooterP, shooterI, shooterD);
	shooterPID.setOutputLimits(0, 255);

	// Initialize mechanical components
	waterYawServo.attach(WATER_YAW_SERVO);
	waterYawServo.write(WATER_YAW_ZERO);
	waterPitchServo.attach(WATER_PITCH_SERVO);
	waterPitchServo.write(WATER_PITCH_ZERO);
	collectorServo.attach(BALL_COLLECT_SERVO);
	ballAimServo.attach(BALL_AIM_SERVO);
	hopperServo.attach(HOPPER_SERVO);
	pinMode(BALL_COLLECT_LIMIT_UP, INPUT);
	pinMode(BALL_COLLECT_LIMIT_DOWN, INPUT);
	pinMode(BALL_COLLECT_LPWM, OUTPUT);
	pinMode(BALL_COLLECT_RPWM, OUTPUT);	
	pinMode(PUMP, OUTPUT);
	pinMode(BALL_COLLECT_SERVO, OUTPUT);
	pinMode(SHOOTER_LPWM, OUTPUT);
	pinMode(SHOOTER_RPWM, OUTPUT);
	pinMode(HALL_EFFECT_BACK, INPUT_PULLUP);
	pinMode(HALL_EFFECT_FRONT, INPUT_PULLUP);
	pinMode(CH_A, INPUT_PULLUP);
	pinMode(CH_B, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(CH_A), getEncoderVals, RISING);

	// Initialize ROS
	nh.initNode();
	nh.subscribe(ballPointSubscriber);
	nh.subscribe(waterPointSubscriber);

}

//---------- Main loop ----------//

void loop() {

	// Update values
	update();

	// ROS handling
	nh.spinOnce();

}

//---------- Helper functions ----------//

// Update values
void update() {

	// Change in encoder values
	if (millis() - lastTime > TIME_INTERVAL) {
		lastTime = millis();
		deltaCounter = counter - prevCounter;
		prevCounter = counter;
	}

	// Calculate rpm
	double num = GEAR_RATIO * deltaCounter / PPR;
	rpm = -((num * 1000 * 60) / TIME_INTERVAL);

}
// Turn on the water pump 
void pump() {

	pump_toggle = !pump_toggle;
	digitalWrite(PUMP, pump_toggle);
	if (pump_toggle) {

		pumpTime = millis();
		//Serial.println("pump turned ON"); 

	} else {

		pumpTime = 0;  
		//Serial.println("pump turned OFF"); 

	}

}

// Get RPM
double getShooterRPM() {
	return rpm;
}

// Aim ball shooter with x, y, z
void aimShooter(float x, float y, float z) {

	float theta = atan(x / y) * 180 / PI;
	ballAimServo.write(int(theta));

}

// Aim ball shooter with angle
void aimShooter(float theta) {
	ballAimServo.write(int(theta));
}

// Fire ball shooter with theta
void ballshooterAimShoot(float theta) {

	ballAimServo.write(int(theta));
	//Serial.println("Theta: " + String(theta));
	delay(1000);
	shooterSpeedUp();

}

// Fire ball shooter 
void ballshooterAimShoot(float x, float y, float z) {

	float theta = atan(x / z) * 180 / PI;
	ballshooterAimShoot(theta);

}

// Shoot watergun
void watergunAimShoot(float x, float y, float z) {

	float theta1 = WATER_YAW_ZERO + atan(x / z) * 180 / PI; 
	float theta2 = WATER_PITCH_ZERO + atan(y / z) * 180 / PI; 
	waterYawServo.write(theta1); 
	waterPitchServo.write(theta2); 
	pump();

}

// Fire ball shooter
void shooterSpeedUp() {

	analogWrite(SHOOTER_LPWM, shooterSpeed);
	analogWrite(SHOOTER_RPWM, 0);
	//Serial.println("Shooter speed: " + String(shooterSpeed)); 

}

// Stop ball shooter
void stopShoot() {

	analogWrite(SHOOTER_LPWM, 0);
	analogWrite(SHOOTER_RPWM, 0);
	//Serial.println("Ball shooter stopped!");

}

// Zero water gun
void zeroWater() {

	waterYawServo.write(WATER_YAW_ZERO);
	waterPitchServo.write(WATER_PITCH_ZERO);

}

// Update ball shooter encoder values
void getEncoderVals() {

	if (digitalRead(CH_B) != digitalRead(CH_A)) {
		counter--;
	} else {
		counter++;
	}

}
