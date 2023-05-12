// good code


#include <Arduino.h>
#include "encoder.h"
#include "drive.h"
#include "wireless.h"
#include "PID.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "ESP32Servo.h"

// wheel radius in meters
#define r 0.06
// distance from back wheel to center in meters
#define b 0.2

float x = 0;
float y = 0;
float heading = 0;
float theta = 0;
float servo_angle = 90;
int timeout_millis = 300;

float last_message_millis = 0;

unsigned long prevLoopTimeMicros = 0; // in microseconds
// how long to wait before updating PID parameters
unsigned long loopDelayMicros = 5000; // in microseconds

double ACCEL_VEL_TRANSITION = (double)loopDelayMicros * 1e-6;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; // trig functions require radians, BNO055 outputs degrees
void readIMU();
void sendIMU();
void readDesiredVel();
void updateRobotPose(float dPhiL, float dPhiR);
void setWheelVel();
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

int servoPin = 13;
Servo myservo;

void setup()
{
    Serial.begin(115200);
    encoderSetup();
    driveSetup();
    wirelessSetup();
    pinMode(servoPin, OUTPUT);

    myservo.setPeriodHertz(50);           // standard 50 hz servo
    myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object

    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    delay(1000);

    bno.setExtCrystalUse(true);
}
void loop()
{
    
    myservo.write(servo_angle);
    if (micros() - prevLoopTimeMicros > loopDelayMicros)
    {

        

        prevLoopTimeMicros = micros();

        readIMU();
        
        readDesiredVel();

        updateVelocity(loopDelayMicros * 1e-6); // update current wheel velocities

        updateRobotPose(dPhiBL, dPhiBR);

        // sends odometry to the remote
        // updateOdometry();

        setWheelVel(); // send new desired wheel velocities
    }
}

void readIMU()
/**
 * Reads position and orientation data from IMU
 *
 * Position: (x, y) [m]
 * Orientation: heading [rad]
 */

{
    sensors_event_t orientationData, linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    // x = x + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    // y = y + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
    heading = orientationData.orientation.x;
}

void sendIMU()
/**
 * Send position, orientation, and wheel encoder data to Jetson
 */
{
    Serial.printf("%.2f, %.2f, %.2f\n", x, y, heading);
}

void readDesiredVel()
{
    /**
     * Reads desired velocity data from Jetson
     */
    if (Serial.available() > 0)
    {
        last_message_millis = millis();
        String data = Serial.readStringUntil('\n');
        int firstCommaIndex = data.indexOf(',');
        int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);

        desiredVelBL = data.substring(0, firstCommaIndex).toFloat();
        desiredVelBR = data.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
        servo_angle = data.substring(secondCommaIndex + 1).toFloat();
        
        sendIMU();
    }
    else
    {
        //if no message for more than timeout_millis, set velocity zero
        if(millis() - last_message_millis > timeout_millis){
            desiredVelBL = 0;
            desiredVelBR = 0;
        }
    }
}

void setWheelVel()
/**
 * Run a PID controller for desired velocities
 */
{
    // calculate error for each motor
    float newErrorFL = desiredVelFL - filtVelFL;
    float newErrorBL = desiredVelBL - filtVelBL;
    float newErrorFR = desiredVelFR - filtVelFR;
    float newErrorBR = desiredVelBR - filtVelBR;

    // get control signal by running PID on all four motors
    voltageFL = runPID(newErrorFL, errorFL, kp, ki, kd, sumErrorFL, maxSumError, loopDelayMicros * 1e-6);
    voltageBL = runPID(newErrorBL, errorBL, kp, ki, kd, sumErrorBL, maxSumError, loopDelayMicros * 1e-6);
    voltageFR = runPID(newErrorFR, errorFR, kp, ki, kd, sumErrorFR, maxSumError, loopDelayMicros * 1e-6);
    voltageBR = runPID(newErrorBR, errorBR, kp, ki, kd, sumErrorBR, maxSumError, loopDelayMicros * 1e-6);

    // only drive the back motors
    driveVolts(0, voltageBL, 0, voltageBR);
}

// updates the robot's path distance variable based on the latest change in angle
void updateRobotPose(float dPhiL, float dPhiR)
{
    float dtheta = r / (2 * b) * (dPhiR - dPhiL);
    // TODO update theta value
    theta = heading*0.01745;
    // TODO use the equations from the handout to calculate the change in x and y
    float dx = r / 2 * (cos(theta) * dPhiR + cos(theta) * dPhiL);
    float dy = r / 2 * (sin(theta) * dPhiR + sin(theta) * dPhiL);
    // TODO update x and y positions
    x += dx;
    y += dy;
}

// stores all the the latest odometry data into the odometry struct
// void updateOdometry()
// {
//     odom_data.millis = millis();
//     odom_data.pathDistance = pathDistance;
//     odom_data.x = x;
//     odom_data.y = y;
//     odom_data.theta = theta;
//     odom_data.velL = filtVelBL;
//     odom_data.velR = filtVelBR;
// }
