#include <Servo.h>
#include <AFMotor.h>
Servo servo1;
Servo servo2; 
bool pump_toggle = false;
int water_yaw_servo_pin = 9; 
int water_pitch_servo_pin = 10; 
int ball_flywheel_pin = 5; 
int ball_loader_pin = 6;
int pump_pin =8;
int pump_count = 0; 
int water_pitch_zero = 0; //indicates what angle the water pitch is at when straight
int water_yaw_zero = 45; // will find value when we attach 
float l = 2; //value, in meters, of how far the water gun can shoot in a straight line
/// value was just a guess, subject to change. Also, we might want to think about a variable distance based on the angle&gravity
unsigned long time = 0;  


void setup() {
servo1.attach(water_yaw_servo_pin); // left, right
servo2.attach(water_pitch_servo_pin); // up, down
pinMode(pump_pin, OUTPUT);
pinMode(ball_flywheel_pin, OUTPUT);
pinMode(ball_loader_pin, OUTPUT);
Serial.begin(9600);
}

void loop() { 
 if (Serial.available()) {
    if (time > 180000){
        digitalWrite(8, false);
    }
    String inByte = Serial.readStringUntil('\n');  // read user input
    if (inByte.length() ==1){
          if (inByte == "P"){
            //activate water pump
           pump(); 
          }
          if(inByte == "B"){
            ball_shooter();
          }
          if(inByte == "D"){ // read current pos
          int yaw_pos = servo1.read(); 
          int pitch_pos = servo2.read(); 
          Serial.println("Current Water Gun Angles: ( " + String(yaw_pos) +", " + String(pitch_pos)+ " )"); //print water gun angles
          // print motor speed and relay state
          }
          if(inByte == "R"){ //reset to zeroed values
            servo1.write(water_yaw_zero); 
            servo2.write(water_pitch_zero); 
          }
    }
    else{
      float x; 
      float y; 
      float z; 
      String rest; 
      int index; 
      String mode; 
      index = inByte.indexOf(","); 
      x = inByte.substring(0, index).toFloat(); 
      rest = inByte.substring(index+1, -1);
      index = rest.indexOf(","); 
      y = rest.substring(0, index).toFloat(); 
      rest = rest.substring(index+1, -1);
      index = rest.indexOf(","); 
      z = rest.substring(0, index).toFloat(); 
      rest = rest.substring(index+1, -1); 
      if (rest.length()!= 1){
        Serial.println("Invalid Format");
      }
      else{
        mode = rest; 
        if (mode == "P"){
          float theta1 = asin(z/l); 
          float theta2 = atan(x/y); 
          theta1 = water_yaw_zero + theta1*180/3.14; 
          theta2 = water_pitch_zero + theta2*180/3.14; 
          servo1.write(int(theta1)); 
          servo2.write(int(theta2)); 
        }
        if (mode =="B"){
          //turn to x,y
          ball_shooter(); 
        }
      }
    }
 }
}

void pump(){
  pump_toggle = !pump_toggle;
  digitalWrite(8, pump_toggle);
  if(pump_toggle == false){
    time = 0; 
    Serial.println("pump turned OFF"); 
  }
  else{
    time = millis(); 
    Serial.println("pump turned ON"); 
  }
}

void ball_shooter(){
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
    // check if ball shooter was actually shot
}


