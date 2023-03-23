#include <Servo.h>

#define BALL_COLLECT_LIMIT_UP 1
#define BALL_COLLECT_LIMIT_DOWN 2
#define BALL_COLLECT_MOTOR 3
#define BALL_COLLECT_SERVO 4

Servo collectorServo;
const int COLLECT_MOTOR_SPEED = 50; // MAX 255
const int COLLECT_SERVO_SPEED = 50; // MAX 360

void setup() {

	Serial.begin(9600);

	collectorServo.attach(BALL_COLLECT_SERVO);
	pinMode(BALL_COLLECT_LIMIT_UP, INPUT);
	pinMode(BALL_COLLECT_LIMIT_DOWN, INPUT);
	pinMode(BALL_COLLECT_MOTOR, OUTPUT);
	pinMode(BALL_COLLECT_SERVO, OUTPUT);

}

enum CollectorModes {

	COLLECT,
	MOVE_DOWN,
	MOVE_UP,
	INACTIVATED

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
    } else if (data == 0) {
      collectorMode = MOVE_UP;
    }
	}

  // Ball collector modes
  if (collectorMode == INACTIVATED) {
    analogWrite(BALL_COLLECT_MOTOR, 0);
    collectorServo.write(0);
  } else if (collectorMode == COLLECT) {
    analogWrite(BALL_COLLECT_MOTOR, 0);
    collectorServo.write(COLLECT_SERVO_SPEED);
  } else if (collectorMode == MOVE_UP) {
    analogWrite(BALL_COLLECT_MOTOR, COLLECT_MOTOR_SPEED);
    collectorServo.write(0);
    if (collectorLimitUpVal == HIGH) {
      collectorMode = INACTIVATED;
    }
  } else {
    analogWrite(BALL_COLLECT_MOTOR, -COLLECT_MOTOR_SPEED);
    collectorServo.write(0);
    if (collectorLimitDownVal == HIGH) {
      collectorMode = COLLECT;
    }
  }
}
