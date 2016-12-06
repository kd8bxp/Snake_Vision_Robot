/* 2-Way motor control for the L9110 motor driver adapted from
 * https://www.bananarobotics.com/shop/How-to-use-the-HG7881-(L9110)-Dual-Channel-Motor-Driver-Module
 * 
 * Multiplexer code based on information found
 * http://bildr.org/2011/02/cd74hc4067-arduino/
 * and https://www.sparkfun.com/products/9056
 * 
 * Adapted by LeRoy Miller, 2016 (Copyright)
 * 
 * This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses>
 */

// Seek Heat.

#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614(); 
#define units 1 //1=Fahrenheit,  0=Celius

//Functions:

//rightForward(speed)
//rightBackward(speed)
//leftForward(speed)
//leftBackward(speed)
//stop()
//readRightSensor() - set the mux to read the sensor connected to C0
//readLeftSensor() - set the mux to read the sensor connected to C1
//readSensor() - returns value of current readings


//Setup variables for motors
#define leftMotorPin1   3 //Left IA2   PWM PIN
#define leftMotorPin2   5 //Left IB2 Direction PIN 
#define rightMotorPin1  6 //Right IA1 PWM PIN
#define rightMotorPin2  9 //Right IB1 Direction PIN
#define pwmR  140 //smallest PWM value that will cause motor to spin
#define pwmL  125

//Setup variables for temperature sensors

int rightSensor;
int leftSensor;
int rightAmbient; //ambient temperature of sensors
int leftAmbient;
int rightCalibrate;
int leftCalibrate;
int aL;
int sL;
int aR;
int sR;


//Variables for Mux. (multiplexer)
/* Special cases - 2 Mux boards are used, one is used for SDA, and
 *  the other is used for SCL. There are two thermal sensors, so only
 *  2 of the 16 ports on the mux are needed, meaning, only one
 *  of the mux control lines needs to be changed to read the two sensors
 *  The code would need to be changed, and expanded to add more I2C sensors
 */

#define s0  7
#define s1  8
#define s2  2
#define s3 13

//RED temperatures are higher than ambient, GREEN ambient temperature
#define LED 12  //ready indicator (GREEN)
#define RIGHTREDLED A0 
#define RIGHTGREENLED A1
#define LEFTREDLED A2
#define LEFTGREENLED A3


void setup()   {  
   Serial.begin(9600);
   mlx.begin();
   pinMode(LED, OUTPUT);
   pinMode(s0, OUTPUT);
   pinMode(s1, OUTPUT);
   pinMode(s2, OUTPUT);
   pinMode(s3, OUTPUT);
   pinMode(RIGHTREDLED, OUTPUT);
   pinMode(RIGHTGREENLED, OUTPUT);
   pinMode(LEFTREDLED, OUTPUT);
   pinMode(LEFTGREENLED, OUTPUT);
   pinMode(leftMotorPin1, OUTPUT); 
   pinMode(leftMotorPin2, OUTPUT);  
   pinMode(rightMotorPin1, OUTPUT);
   pinMode(rightMotorPin2, OUTPUT);
   digitalWrite(s0, LOW);
   digitalWrite(s1, LOW);
   digitalWrite(s2, LOW);
   digitalWrite(s3, LOW);
   digitalWrite(RIGHTREDLED, LOW);
   digitalWrite(RIGHTGREENLED, LOW);
   digitalWrite(LEFTREDLED, LOW);
   digitalWrite(LEFTGREENLED, LOW);
  stop();
  delay(5000); //delay before calibration - warning sensors are very senitive point them away from any type of heat source.
  calibrateSensors(); 
 delay(5000);  //another delay after calibration (a buzzer might go good here, warning that the bot is just about ready)
 digitalWrite(LED, HIGH); //When LED comes on ROBOT is Ready to move.
 
}

void loop()                     
{
readRightSensor();
readLeftSensor();

aL = map(leftAmbient, -40, leftAmbient, 0.0, pwmR);
sL = map(leftSensor, leftAmbient, leftSensor, pwmR, 0.0);
aR = map(rightAmbient, -40, rightAmbient, 0.0, pwmL);
sR = map(rightSensor, rightAmbient, rightSensor, pwmL, 0.0);
if (leftAmbient >= leftSensor) {sL = pwmR;}
if (rightAmbient >= rightSensor) {sR = pwmL;}
rightForward(aL - sL);
leftForward(aR - sR);

//delay(50); //test code with small delay (may not be needed)

}

void readRightSensor() {
  digitalWrite(s0, LOW);
  //because S1, S2, S3 are already set LOW we don't need to set again
  rightSensor = readSensor() + rightCalibrate;
  rightAmbient = readAmbient();
  if (rightAmbient >= rightSensor) { 
    digitalWrite(RIGHTGREENLED, HIGH); 
    digitalWrite(RIGHTREDLED, LOW); } else {
        digitalWrite(RIGHTGREENLED, LOW);
        digitalWrite(RIGHTREDLED, HIGH);
    }
  }

void readLeftSensor() {
  digitalWrite(s0, HIGH);
  //because s1, s2, s3 are already set LOW we don't need to set again
  leftSensor = readSensor() + leftCalibrate;
  leftAmbient = readAmbient();
  if (leftAmbient >= leftSensor) { 
    digitalWrite(LEFTGREENLED, HIGH); 
    digitalWrite(LEFTREDLED, LOW); } else {
        digitalWrite(LEFTGREENLED, LOW);
        digitalWrite(LEFTREDLED, HIGH);
    }
}

int readSensor() {
  
  /* Code to read the Thermal sensor goes here 
   *  and should return a value back to the calling function
   */
   
   int temp;
   if (units) {temp = mlx.readObjectTempF();} else {temp = mlx.readObjectTempC();} 
   return temp;
   
}

int readAmbient() {
  
  // Code to read the Thermal sensor Ambient Tempature
  
  int temp;
  if (units) {temp = mlx.readAmbientTempF();} else {temp = mlx.readAmbientTempC();}
  return temp;
  
}

void rightForward(int speedOfRotate) {
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(rightMotorPin1, speedOfRotate);
}

void rightBackward(int speedOfRotate) {
  digitalWrite(rightMotorPin2, HIGH);
  analogWrite(rightMotorPin1, speedOfRotate);
}

void leftForward(int speedOfRotate) {
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(leftMotorPin1, speedOfRotate);
}

void leftBackward(int speedOfRotate) {
  digitalWrite(leftMotorPin2, HIGH);
  analogWrite(leftMotorPin1, speedOfRotate);
}

void stop() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
}

void calibrateSensors() {
  int avg1 = 0;
  int avg2 = 0;
  
  Serial.print("Calibrate Sensors.");
  
  for (int i=0; i<=100; i++) {
    Serial.print(".");
    readRightSensor();
  avg1 = avg1 + rightSensor;
  avg2 = avg2 + rightAmbient;  
  }
  avg1 = avg1 / 100;
  avg2 = avg2 / 100;
  
  rightCalibrate = avg2 - avg1;
  
  avg1 = 0;
  avg2 = 0;
  for (int i=0; i<=100; i++) {
    Serial.print(".");
    readLeftSensor();
    avg1 = avg1 + leftSensor;
    avg2 = avg2 + leftAmbient;
  }
  avg1 = avg1 / 100;
  avg2 = avg2 / 100;
  leftCalibrate = avg2 - avg1;
  Serial.println("");
  Serial.print("Right Calibrate: ");
  Serial.print(rightCalibrate);
  Serial.print(" Left Calibrate: ");
  Serial.println(leftCalibrate);
}

