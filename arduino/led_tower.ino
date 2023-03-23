#define GREEN 50
#define YELLOW 51
#define RED 52
#define BUZZER 53

void setup() {
	pinMode(GREEN, OUTPUT);
	pinMode(YELLOW, OUTPUT);
	pinMode(RED, OUTPUT);
}

void loop() {
	int input = 0; // This will be the input it receives
	digitalWrite(GREEN, LOW);
	digitalWrite(YELLOW, LOW);
	digitalWrite(RED, LOW);

	switch (input) {
		case 0:
			digitalWrite(GREEN, HIGH);
		case 2:
			digitalWrite(YELLOW, HIGH);
		default:
			digitalWrite(RED, HIGH);
	}
}
