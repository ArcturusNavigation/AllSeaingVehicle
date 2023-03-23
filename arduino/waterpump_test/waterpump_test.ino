#include <Servo.h>
#include <AFMotor.h>
Servo servo1;
Servo servo2; 
bool pump_toggle = false;
// servo2 is up,down servo1 is left,right
void setup() {
  // put your setup code here, to run once:
servo1.attach(9); 
servo2.attach(10); 
pinMode(8, OUTPUT);
Serial.begin(9600);
}

void loop() { 
 if (Serial.available()) {
    String inByte = Serial.readStringUntil('\n');  // read user input
    if (inByte == "P"){
      pump_toggle = !pump_toggle;
      digitalWrite(8, pump_toggle);
    }
    else{
      float angles[2]; 
      int index = inByte.indexOf(","); 
      angles[0] = inByte.substring(0,index).toFloat(); 
      angles[1] = inByte.substring(index+1, inByte.length()).toFloat();

      servo1.write(angles[0]); 
      servo2.write(angles[1]); 
    }
    }

  // put your main code here, to run repeatedly:
}
