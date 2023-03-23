#include <Servo.h>
#include <AFMotor.h>

#define HALL_EFFECT_BACK 12
#define HALL_EFFECT_FRONT 13
#define HOPPER_SERVO 5
#define BALL_COLLECT_LIMIT_UP 1
#define BALL_COLLECT_LIMIT_DOWN 2
#define BALL_COLLECT_LPWM 3
#define BALL_COLLECT_RPWM 4
#define BALL_COLLECT_SERVO 4
#define BALL_AIM_SERVO 7
#define SENSOR_PIN 0
#define RPWM_OUTPUT 5 // Connect to IBT-2 pin 1 (RPWM)
#define LPWM_OUTPUT 6 // Connect to IBT-2 pin 2 (LPWM) 
#define CH_A 20
#define CH_B 21

// Servo setup
Servo collectorServo;
Servo hopperServo;
const int COLLECT_MOTOR_SPEED = 50; // MAX 255
const int COLLECT_SERVO_SPEED = 50; // MAX 360
const int REVERSE_TIME = 5000;

unsigned long timeSinceReverse = 0;

// Ball shooter setup
Servo ballAimServo;

// Encoder counter
volatile long counter = 0;

// Ball shooter hopper movement
bool isHopperMoving = false;
unsigned long timeSinceHopperMove = 0;

void setup() {

  Serial.begin(9600);

  collectorServo.attach(BALL_COLLECT_SERVO);
  hopperServo.attach(HOPPER_SERVO);
	pinMode(BALL_COLLECT_LIMIT_UP, INPUT);
	pinMode(BALL_COLLECT_LIMIT_DOWN, INPUT);
  pinMode(BALL_COLLECT_LPWM, OUTPUT);
  pinMode(BALL_COLLECT_RPWM, OUTPUT);	
  pinMode(BALL_COLLECT_SERVO, OUTPUT);
  pinMode(HALL_EFFECT_BACK, INPUT_PULLUP);
	pinMode(HALL_EFFECT_FRONT, INPUT_PULLUP);
  pinMode(CH_A, INPUT_PULLUP);
  pinMode(CH_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CH_A), getEncoderVals, RISING);

}

enum CollectorModes {

	COLLECT,
	MOVE_DOWN,
	MOVE_UP,
	INACTIVATED,
  REVERSE

};

CollectorModes collectorMode = INACTIVATED;
void loop() {

  int collectorLimitUpVal = digitalRead(BALL_COLLECT_LIMIT_UP);
	int collectorLimitDownVal = digitalRead(BALL_COLLECT_LIMIT_DOWN);
  Serial.println("Counter: " + String(counter));
  Serial.println("Limit up: " + String(collectorLimitUpVal));
	Serial.println("Limit down: " + String(collectorLimitDownVal));

  if (Serial.available() > 0) {

    String data = Serial.readStringUntil('\n');
    if (data == "0") {
      isHopperMoving = true;
      timeSinceHopperMove = millis();
      Serial.println("Hopper moving");
    } else if (data == "1") {
      collectorMode = MOVE_DOWN;
    } else if (data == "2") {
      collectorMode = MOVE_UP;
    } else if (data == "3") {
      ballShoot();
    } else if (data[0] == "s") {
      int x = data.substring(1, 4).toInt();
      int y = data.substring(4, 7).toInt();
      int z = data.substring(7, 10).toInt();
      ballshooterAimShoot(x, y, z);
    }

	}

  // Hall effect sensor values
  int hallEffectBackVal = digitalRead(HALL_EFFECT_BACK);
	int hallEffectFrontVal = digitalRead(HALL_EFFECT_FRONT);

  // Moving the hopper
	if (isHopperMoving) {

		hopperServo.write(0);
		Serial.println(timeSinceHopperMove);

	} else {

		hopperServo.write(60);

	}

  // Stopping the hopper
	if ((hallEffectFrontVal == HIGH || hallEffectBackVal == HIGH) &&
		millis() - timeSinceHopperMove > 500) {

		isHopperMoving = false;
		Serial.println("Hopper stopped");

	}

  // Ball collector modes
  if (collectorMode == INACTIVATED) {
    analogWrite(BALL_COLLECT_LPWM, 0);
    analogWrite(BALL_COLLECT_RPWM, 0);
    collectorServo.write(0);
  } else if (collectorMode == COLLECT) {
    analogWrite(BALL_COLLECT_LPWM, 0);
    analogWrite(BALL_COLLECT_RPWM, 0);
    collectorServo.write(COLLECT_SERVO_SPEED);
  } else if (collectorMode == MOVE_UP) {
    analogWrite(BALL_COLLECT_LPWM, COLLECT_MOTOR_SPEED);
    analogWrite(BALL_COLLECT_RPWM, 0);
    collectorServo.write(0);
    if (collectorLimitUpVal == HIGH) {
      collectorMode = INACTIVATED;
    }
  } else if (collectorMode == MOVE_DOWN) {
    analogWrite(BALL_COLLECT_LPWM, 0);
    analogWrite(BALL_COLLECT_RPWM, -COLLECT_MOTOR_SPEED);
    collectorServo.write(0);
    if (collectorLimitDownVal == HIGH) {
      collectorMode = COLLECT;
    }
  } else if (collectorMode == REVERSE) {
    analogWrite(BALL_COLLECT_LPWM, 0);
    analogWrite(BALL_COLLECT_RPWM, 0);
    collectorServo.write(-COLLECT_SERVO_SPEED);
    if (millis() - timeSinceReverse > REVERSE_TIME) {
      collectorMode = INACTIVATED;
    }
  } else {
    Serial.println("wfh is going on");
  }

}

// Fire ball shooter 
void ballshooterAimShoot(float x, float y, float z) {

	float theta = atan(x/y); 
	ballAimServo.write(int(theta)); 
	delay(1000); 
	ballShoot();

}

// Fire ball shooter
void ballShoot() {

	int sensorValue = 1023;
	// sensor value is in the range 0 to 1023
	// the lower half of it we use for reverse rotation; the upper half for forward rotation
	if (sensorValue < 512){
		// reverse rotation
		int reversePWM = -(sensorValue - 511) / 2;
		analogWrite(LPWM_OUTPUT, 0);
		analogWrite(RPWM_OUTPUT, reversePWM);
	} else {
		// forward rotation
		int forwardPWM = (sensorValue - 512) / 2;
		analogWrite(RPWM_OUTPUT, 0);
		analogWrite(LPWM_OUTPUT, forwardPWM);
	}
	Serial.println("Ball shooter fired!"); 

}

// Update ball shooter encoder values
void getEncoderVals() {

  if (digitalRead(CH_B) != digitalRead(CH_A)) {
    counter--;
  } else {
    counter++;
  }

}
