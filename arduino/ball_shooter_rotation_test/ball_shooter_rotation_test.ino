#include <Servo.h>

#define HALL_EFFECT_BACK 12
#define HALL_EFFECT_FRONT 13
#define HOPPER_SERVO 5

Servo hopperServo;

void setup() {

	Serial.begin(9600);

	hopperServo.attach(HOPPER_SERVO);

	pinMode(HALL_EFFECT_BACK, INPUT_PULLUP);
	pinMode(HALL_EFFECT_FRONT, INPUT_PULLUP);

}

bool isHopperMoving = false;
unsigned long timeSinceHopperMove = 0;
void loop() {

	if (Serial.available() > 0) {

		int data = Serial.readStringUntil('\n').toInt();
		if (data == 0) {

			isHopperMoving = true;
			timeSinceHopperMove = millis();
			Serial.println("Hopper moving");

		} 

	}

	int hallEffectBackVal = digitalRead(HALL_EFFECT_BACK);
	int hallEffectFrontVal = digitalRead(HALL_EFFECT_FRONT);

	if (isHopperMoving) {

		hopperServo.write(0);
		Seria.println(timeSinceHopperMove);

	} else {

		hopperServo.write(60);

	}

	if ((hallEffectFrontVal == HIGH || hallEffectBackVal == HIGH) &&
		millis() - timeSinceHopperMove > 500) {

		isHopperMoving = false;
		Serial.println("Hopper stopped");

	}

}
