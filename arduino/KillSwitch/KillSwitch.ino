#include <Servo.h>

int pump_relay = 12;
int thruster_relay = 11;
int kill_pin = 10;
int last_reading;

bool pump_switch = false;
bool thruster_switch = true;
Servo yaw;
Servo pitch;
//bool sw = false;
//
//ros::NodeHandle  nh;
//ros::Subscriber fire_sub("watergun/fire", &fire_cb);
//
//void fire_cb(const std_msgs::Bool& fire){
//  
//}
void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
 yaw.attach(9);
 pitch.attach(10);
 pinMode(kill_pin, OUTPUT);
 pinMode(thruster_relay, OUTPUT);
 pinMode(pump_relay, OUTPUT);
 digitalWrite(kill_pin, HIGH);
 digitalWrite(thruster_relay, HIGH);
 digitalWrite(pump_relay, LOW);
 last_reading = digitalRead(kill_pin);
 yaw.write(100);
 pitch.write(160);
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println(digitalRead(kill_pin));
int new_reading = digitalRead(kill_pin);
  if(new_reading & !last_reading){
    Serial.println("Relay closed!");
    digitalWrite(pump_relay, HIGH);
  }
  else if(!new_reading & last_reading){
    Serial.println("Relay open!");
    digitalWrite(pump_relay, LOW);
  }
  last_reading = new_reading;
//  delay(1);
}
