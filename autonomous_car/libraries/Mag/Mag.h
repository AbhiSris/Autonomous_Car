/*
 * Mag.h - Library for receiving magnometer inputs
 * Header file for the Magnometer functions, has all the variables
 */

#ifndef Mag_h
#define Mag_h

#include <Wire.h>
#include "Arduino.h"

//I2C Arduino Library
#define address 0x1E //0011110b, I2C 7bit address of HMC5883

class Magnometer {
  public:
    void mag_setup(/*address, register mode 2, continuous measurement mode*/);
    void start_comm(/*Register 3*/);
    void mag_read_write(/*Some ish*/);
    void setAngle(double gp);
    double getAngle();
  private:
  	int mag_x;
  	int mag_y;
  	int mag_z;
  	double _angle;
};

#endif