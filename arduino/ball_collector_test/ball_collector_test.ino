#include <Servo.h>

#define BALL_COLLECT_LIMIT_UP 1
#define BALL_COLLECT_LIMIT_DOWN 2
#define BALL_COLLECT_LPWM 3
#define BALL_COLLECT_RPWM 4
#define BALL_COLLECT_SERVO 5

Servo collectorServo;
const int COLLECT_MOTOR_SPEED = 50; // MAX 255
const int COLLECT_SERVO_SPEED = 50; // MAX 360
const int REVERSE_TIME = 5000;

unsigned long timeSinceReverse = 0;

void setup() {

	Serial.begin(9600);

	collectorServo.attach(BALL_COLLECT_SERVO);
	pinMode(BALL_COLLECT_LIMIT_UP, INPUT);
	pinMode(BALL_COLLECT_LIMIT_DOWN, INPUT);
	pinMode(BALL_COLLECT_LPWM, OUTPUT);
  pinMode(BALL_COLLECT_RPWM, OUTPUT);
	pinMode(BALL_COLLECT_SERVO, OUTPUT);

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

	Serial.println("Limit up: " + String(collectorLimitUpVal));
	Serial.println("Limit down: " + String(collectorLimitDownVal));

	if (Serial.available() > 0) {

    int data = Serial.readStringUntil('\n').toInt();
    if (data == 0) {
      collectorMode = MOVE_DOWN;
    } else if (data == 1) {
      collectorMode = MOVE_UP;
    } else if (data == 2) {
      collectorMode = REVERSE;
      timeSinceReverse = millis();
    }
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
