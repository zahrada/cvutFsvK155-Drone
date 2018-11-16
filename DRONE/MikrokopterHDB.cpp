
#include "Arduino.h"
#include "MikrokopterHDB.h"

Motors::Motors()
{
  motor0 = 0;
  motor1 = 1;
  motor2 = 2;
  motor3 = 3;
  motor4 = 4;
  motor5 = 5;
}

void Motors::controlMotor(int &motor, int &speed)
{
  // Control one bruslesh motor by speed
  Wire.beginTransmission((TWI_BLCTRL_BASEADDR + (motor << 1)) >> 1);
  Wire.write(speed);
  Wire.endTransmission();
}

void Motors::controlMotors(Motors &motor, int &speed)
{
  // Control every bruslesh motors by speed
  controlMotor(motor.motor0, speed);
  controlMotor(motor.motor1, speed);
  controlMotor(motor.motor2, speed);
  controlMotor(motor.motor3, speed);
  controlMotor(motor.motor4, speed);
  controlMotor(motor.motor5, speed);
}

void Motors::controlSpeedBetween2Motors(int &speed_first, int &speed_second, float angle, int speed_change)
{
  // Calculate how much part of speed to change movement
  int speed_upgrade = speed_change * angle / 60;
  speed_first += speed_upgrade * (60 - angle);
  speed_second += speed_upgrade * angle;
}

void Motors::rotateToNorth(Motors &motor, float &roll, float &pitch, float &heading, int &speed, int speed_change)
{
  // control ESC by IMU, rotate to north by heading to navigate direction between motor1 and motor2
  // heading data is <0,360>
  // if heading is less than 10 and more than 350, nothnig happend, drone is stil ballancing
  // if heading is more than 10 and less than 180, drone turn by CCV
  // if heading is more than 180 and less than 350, drone turn by CV
  int speed_rotate = speed + speed_change;

  if (heading < 180 && heading > 10)
  {
    while (heading < 180 && heading > 10)
    {
      controlMotor(motor.motor0, speed_rotate);
      controlMotor(motor.motor1, speed);
      controlMotor(motor.motor2, speed_rotate);
      controlMotor(motor.motor3, speed);
      controlMotor(motor.motor4, speed_rotate);
      controlMotor(motor.motor5, speed);
    }
  }
  else if (heading > 180 && heading < 350)
  {
    while (heading > 180 && heading < 350)
    {
      controlMotor(motor.motor0, speed);
      controlMotor(motor.motor1, speed_rotate);
      controlMotor(motor.motor2, speed);
      controlMotor(motor.motor3, speed_rotate);
      controlMotor(motor.motor4, speed);
      controlMotor(motor.motor5, speed_rotate);
    }
  }
  else
  {
    controlMotors(motor, speed);
  }
}


void Motors::balanceDrone(Motors &motor, float &roll, float &pitch, float &heading, int &speed, int speed_change)
{
  // Control drone balance by IMU
  // Calculate direction of tilt and regulate speed of bruslesh motors
  // IMU - roll, pitch, heading
  // Drone - motor, speed
  // axes +y (+pitch) is motor 3
  // axes +x (+roll) is between motor 4 and 5
  // calculate direction of tilt (balance_angle)
  // set increase speed on two motor to balance drone

  // change speed for balance
  int speed_first = speed;
  int speed_second = speed;
  int speed_upgrade;

  // compute angle of roll and pitch
  float balanc_angle = atan2(pitch, roll);

  if (balanc_angle < 0)
    balanc_angle += 2 * PI;
  if (balanc_angle > 2 * PI)
    balanc_angle -= 2 * PI;

  if (balanc_angle >= 0 && balanc_angle < 60)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, balanc_angle - 0, speed_change);
    controlMotor(motor.motor0, speed);
    controlMotor(motor.motor1, speed);
    controlMotor(motor.motor2, speed);
    controlMotor(motor.motor3, speed_first);
    controlMotor(motor.motor4, speed_second);
    controlMotor(motor.motor5, speed);
  }
  else if (balanc_angle >= 60 && balanc_angle < 120)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, balanc_angle - 60, speed_change);
    controlMotor(motor.motor0, speed);
    controlMotor(motor.motor1, speed);
    controlMotor(motor.motor2, speed);
    controlMotor(motor.motor3, speed);
    controlMotor(motor.motor4, speed_first);
    controlMotor(motor.motor5, speed_second);
  }
  else if (balanc_angle >= 120 && balanc_angle < 180)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, balanc_angle - 120, speed_change);
    controlMotor(motor.motor0, speed_second);
    controlMotor(motor.motor1, speed);
    controlMotor(motor.motor2, speed);
    controlMotor(motor.motor3, speed);
    controlMotor(motor.motor4, speed);
    controlMotor(motor.motor5, speed_first);
  }
  else if (balanc_angle >= 180 && balanc_angle < 240)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, balanc_angle - 180, speed_change);
    controlMotor(motor.motor0, speed_first);
    controlMotor(motor.motor1, speed_second);
    controlMotor(motor.motor2, speed);
    controlMotor(motor.motor3, speed);
    controlMotor(motor.motor4, speed);
    controlMotor(motor.motor5, speed);
  }
  else if (balanc_angle >= 240 && balanc_angle < 300)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, balanc_angle - 240, speed_change);
    controlMotor(motor.motor0, speed);
    controlMotor(motor.motor1, speed_first);
    controlMotor(motor.motor2, speed_second);
    controlMotor(motor.motor3, speed);
    controlMotor(motor.motor4, speed);
    controlMotor(motor.motor5, speed);
  }
  else if (balanc_angle >= 300 && balanc_angle < 360)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, balanc_angle - 300, speed_change);
    controlMotor(motor.motor0, speed);
    controlMotor(motor.motor1, speed);
    controlMotor(motor.motor2, speed_first);
    controlMotor(motor.motor3, speed_second);
    controlMotor(motor.motor4, speed);
    controlMotor(motor.motor5, speed);
  }
}

void Motors::flyToDirection(Motors &motor, int &speed, int speed_change, float azimuth)
{
  // Drone flying direction by azimuth
  // azimuth data is <0,360>
  // if azimuth is between 330 and 30, speed increase on motor  4 and 5
  // if azimuth is between 30 and 90, speed increase on motor  5 and 0
  // if azimuth is between 90 and 150, speed increase on motor  0 and 1
  // if azimuth is between 150 and 210, speed increase on motor  1 and 2
  // if azimuth is between 210 and 270, speed increase on motor  2 and 3
  // if azimuth is between 270 and 330, speed increase on motor  3 and 4

  int speed_first = speed;
  int speed_second = speed;

  if (azimuth >= 0 && azimuth < 30 || azimuth >= 330 && azimuth < 360)
  {
    if (azimuth >= 330 && azimuth < 360)
    {
      azimuth -= 330;
    }
    else
    {
      azimuth += 30;
    }

    controlSpeedBetween2Motors(speed_first, speed_second, azimuth, speed_change);
    controlMotor(motor.motor0, speed);
    controlMotor(motor.motor1, speed);
    controlMotor(motor.motor2, speed);
    controlMotor(motor.motor3, speed);
    controlMotor(motor.motor4, speed_first);
    controlMotor(motor.motor5, speed_second);
  }
  else if (azimuth >= 30 && azimuth < 90)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, azimuth - 30, speed_change);
    controlMotor(motor.motor0, speed_second);
    controlMotor(motor.motor1, speed);
    controlMotor(motor.motor2, speed);
    controlMotor(motor.motor3, speed);
    controlMotor(motor.motor4, speed);
    controlMotor(motor.motor5, speed_first);
  }
  else if (azimuth >= 90 && azimuth < 150)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, azimuth - 90, speed_change);
    controlMotor(motor.motor0, speed_first);
    controlMotor(motor.motor1, speed_second);
    controlMotor(motor.motor2, speed);
    controlMotor(motor.motor3, speed);
    controlMotor(motor.motor4, speed);
    controlMotor(motor.motor5, speed);
  }
  else if (azimuth >= 150 && azimuth < 210)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, azimuth - 150, speed_change);
    controlMotor(motor.motor0, speed);
    controlMotor(motor.motor1, speed_first);
    controlMotor(motor.motor2, speed_second);
    controlMotor(motor.motor3, speed);
    controlMotor(motor.motor4, speed);
    controlMotor(motor.motor5, speed);
  }
  else if (azimuth >= 210 && azimuth < 270)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, azimuth - 210, speed_change);
    controlMotor(motor.motor0, speed);
    controlMotor(motor.motor1, speed_first);
    controlMotor(motor.motor2, speed_second);
    controlMotor(motor.motor3, speed);
    controlMotor(motor.motor4, speed);
    controlMotor(motor.motor5, speed);
  }
  else if (azimuth >= 270 && azimuth < 330)
  {
    controlSpeedBetween2Motors(speed_first, speed_second, azimuth - 270, speed_change);
    controlMotor(motor.motor0, speed);
    controlMotor(motor.motor1, speed);
    controlMotor(motor.motor2, speed);
    controlMotor(motor.motor3, speed_first);
    controlMotor(motor.motor4, speed_second);
    controlMotor(motor.motor5, speed);
  }
}

void Motors::flyUp(Motors &motor, int &speed, int speed_change, float high)
{
  speed +=speed_change;
  controlMotors(motor, speed);

}
