#include <Servo.h>

#define BALL_COLLECT_LIMIT_UP
#define BALL_COLLECT_LIMIT_DOWN
#define BALL_COLLECT_MOTOR
#define BALL_COLLECT_SERVO

Servo collectorServo;

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

}

int rotationMode = 0;
CollectorModes mode = INACTIVATED;
void loop() {

	int collectorLimitUpVal = digitalRead(BALL_COLLECT_LIMIT_UP);
	int collectorLimitDownVal = digitalRead(BALL_COLLECT_LIMIT_DOWN);

	Serial.println("Limit up: " + String(collectorLimitUpVal));
	Serial.println("Limit down: " + String(collectorLimitDownVal));

	if (Serial.available() > 0) {

		analogWrite(BALL_COLLECT_MOTOR, 50); // MAX 255

	}

		

	if (collectorLimitDownVal == HIGH || collectorLimitUpVal == HIGH) {
		analogWrite(BALL_COLLECT_MOTOR, 0)
	}

}
