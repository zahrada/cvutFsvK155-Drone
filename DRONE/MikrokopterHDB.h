//Library for comunication with MikroKopter Hexa Distribution Board V2.0
// C - SCL(A5)
// D - SDA(A4)
// + - VIN 12V
// - - GROUND

#ifndef MikrokopterHDB_h
#define MikrokopterHDB_h

#include "Arduino.h"
#include "Wire.h"    // I2C library

#define TWI_BLCTRL_BASEADDR 0x52 // define ESC and bruslesh motor

class Motors {
  private:
  int motor0, motor1, motor2, motor3, motor4, motor5;

  public:
    Motors();
    int speed = 0;  // speed of bruslesh motor, for testing ideal 150 !!!!!!!!!!!!!!!!!!!!
    void controlMotor(int &motor, int &speed);
    void controlMotors(Motors &motor, int &speed);
    void controlSpeedBetween2Motors(int &speed_first, int &speed_second, float angle, int speed_change);
    void rotateToNorth(Motors &motor, float &roll, float &pitch, float &heading, int &speed, int speed_change);
    void balanceDrone(Motors &motor, float &roll, float &pitch, float &heading, int &speed, int speed_change);
    void flyToDirection(Motors &motor, int &speed, int speed_change, float azimuth);
    void flyUp(Motors &motor, int &speed, int speed_change, float high);

};

#endif // MikrokopterHDB_h
