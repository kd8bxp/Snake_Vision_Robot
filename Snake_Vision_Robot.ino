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


// **** THIS IS FOR TESTING THE SENSORS TO MAKE THE ROBOT SWITCH TO MASTER BRANCH ****

#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614(); //I think I can just use one device name here, but it's something that needs to be tested

#define units 1 //1=Fahrenheit,  0=Celius


//Setup variables for themal sensors

int rightSensor;
int leftSensor;
int rightAmbient; //ambient temperature of sensors
int leftAmbient;
int rightCalibrate;
int leftCalibrate;

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

void setup()   {  
   Serial.begin(9600);
   pinMode(s0, OUTPUT);
   pinMode(s1, OUTPUT);
   pinMode(s2, OUTPUT);
   pinMode(s3, OUTPUT);
   digitalWrite(s0, LOW);
   digitalWrite(s1, LOW);
   digitalWrite(s2, LOW);
   digitalWrite(s3, LOW);
 mlx.begin();
 
delay(1000); //Need delay here for everything to catch up

calibrateSensors();
 
}

void loop()                     
{
  delay(1000);
readRightSensor();
readLeftSensor();


//Compare Ambient tempature to reading tempature if changed do something
Serial.print ("Right Ambient Temp: ");
Serial.print(rightAmbient);
Serial.print(" F Right Temp: ");
Serial.print(rightSensor);
Serial.println(" F");
Serial.print("Left Ambient Temp: ");
Serial.print(leftAmbient);
Serial.print(" F Left Temp: ");
Serial.print(leftSensor);
Serial.println(" F");

if (rightAmbient != rightSensor || leftAmbient != leftSensor) {

    if (rightSensor > leftSensor) { Serial.println("Right High left Motor on"); }
    if (rightSensor < leftSensor) { Serial.println("Left High right Motor on"); }
    if (rightSensor == leftSensor) { Serial.println("Both Equal - Drive Forward"); }
    //probably need to see if both ambients are the same, and stop robot
}
 else {Serial.println("Stop Motors");}
}

void readRightSensor() {
  digitalWrite(s0, LOW);
  //because S1, S2, S3 are already set LOW we don't need to set again
  rightSensor = readSensor() + rightCalibrate;
  rightAmbient = readAmbient();
  }

void readLeftSensor() {
  digitalWrite(s0, HIGH);
  //because s1, s2, s3 are already set LOW we don't need to set again
  leftSensor = readSensor() + leftCalibrate;
  leftAmbient = readAmbient();
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

