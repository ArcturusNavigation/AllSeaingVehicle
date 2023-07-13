/*
   IBT-2 Motor Control Board driven by Arduino.

   Speed and direction controlled by a potentiometer attached to analog input 0.
   One side pin of the potentiometer (either one) to ground; the other side pin to +5V

   Connection to the IBT-2 board:
   IBT-2 pin 1 (RPWM) to Arduino pin 5(PWM)
   IBT-2 pin 2 (LPWM) to Arduino pin 6(PWM)
   IBT-2 pins 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
   IBT-2 pin 8 (GND) to Arduino GND
   IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected
 */

int RPWM_Output = 10; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 9; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

void setup()
{
	pinMode(RPWM_Output, OUTPUT);
	pinMode(LPWM_Output, OUTPUT);
}

void loop()
{

	// sensor value is in the range 0 to 1023
	// the lower half of it we use for reverse rotation; the upper half for forward rotation
		// reverse rotation
  analogWrite(LPWM_Output, 100);
  analogWrite(RPWM_Output, 0);
  //digitalWrite(LPWM_Output, LOW);
  //digitalWrite(RPWM_Output, HIGH);
}
