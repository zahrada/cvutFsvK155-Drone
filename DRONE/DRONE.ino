/*
  Advanced_I2C.ino
  Brian R Taylor
  brian.taylor@bolderflight.com

  Copyright (c) 2017 Bolder Flight Systems

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
  and associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute,
  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or
  substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h" //include library with gyro/akce/mag
#include "MahonyAHRS.h" // include library to filter data of gyro/akce/mag to roll, pich, yaw
#include "MikrokopterHDB.h"


// speed of bruslesh motor, for testing idela 150 !!!!!!!!!!!!!!!!!!!!
//int speed = 0;

// class for filter data from gyro/akce/mag
Mahony filter;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
float ax, ay, az, gx, gy, gz, mx, my, mz, deltat, roll, pitch, heading;

// Drone brushless motors
Motors drone;

//joystick
#define joy 2


void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.print("IMU initialization unsuccessful");
    Serial.print("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.print(status);
    while (1) {}
  }
  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  pinMode(joy, INPUT_PULLUP);

  filter.begin(50);
}

void loop() {

  int  speed = 0;

  int stavTlac = digitalRead(joy);


  if (stavTlac == LOW)
  {
    speed = 50;

  }
  else
  {
    speed = 00;
  }

speed = 50;

  // read the sensor
  IMU.readSensor();
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  deltat = IMU.getTemperature_C();

  //***************************************************************************************************************
  // Mahonny Filtracion********************************************************************************************
  // Similar to Madgwick scheme but uses proportional and integral filtering on
  // the error between estimated reference vectors and measured ones.
  // short name local variable for readability
  filter.update(gx * (180 / M_PI), gy * (180 / M_PI), gz * (180 / M_PI), ax, ay, az, mx, my, mz);

  //***************************************************************************************************************************
  // Compute direction to North from magnetometr not used *********************************************************************
  float angle_north = atan2(mx, my);
  float magDeclinRad = 0.07; // IN BRNO
  angle_north += magDeclinRad;

  // corection of angle
  if (angle_north < 0)
    angle_north += 2 * PI;
  if (angle_north > 2 * PI)
    angle_north -= 2 * PI;

  float angle_northst = angle_north * 180 / M_PI;

  //***************************************************************************************************************************
  // Compute roll, pitch, yaw from akcelerometr, not work for flying object ***************************************************
  /*
    float pitch = atan2(ax, sqrt(ay * ay + az * az)); // around Y axes
    float roll = atan2(ay, sqrt(ax * ax + az * az)); // around X axes
    float yaw = atan2(az, sqrt(ax * ax + az * az)); // around Z axes

    float pitchst  = pitch * 180 / M_PI;
    float rollst = roll * 180 / M_PI;
    float yawst = yaw * 180 / M_PI;
*/

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print(heading);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");

    

  delay(10);
}



























// Decide when to print
bool readyToPrint() {
  static unsigned long nowMillis;
  static unsigned long thenMillis;

  // If the Processing visualization sketch is sending "s"
  // then send new data each time it wants to redraw
  while (Serial.available()) {
    int val = Serial.read();
    if (val == 's') {
      thenMillis = millis();
      return true;
    }
  }
  // Otherwise, print 8 times per second, for viewing as
  // scrolling numbers in the Arduino Serial Monitor
  nowMillis = millis();
  if (nowMillis - thenMillis > 125) {
    thenMillis = nowMillis;
    return true;
  }
  return false;
}
