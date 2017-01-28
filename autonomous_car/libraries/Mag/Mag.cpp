/*
 * Mag.cpp - Library for magnometer readings
 * Class file for the magnometer
 */

#include "Arduino.h"
#include "Wire.h"
#include "Mag.h"

//I2C Arduino Library
#define address 0x1E //0011110b, I2C 7bit address of HMC5883

void Magnometer::mag_setup() {
  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void Magnometer::start_comm() {
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
}

void Magnometer::mag_read_write() {
  int x,y,z; //triple axis data
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }

  mag_x = x;   
  mag_y = y;
  mag_z = z;
  
  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.print(z);
  Serial.print("    ");
  
  delay(250);
}

void Magnometer::setAngle(double gp) {
  double angle;

  double x_n = mag_x*gp;
  double y_n = mag_y*gp;
  double z_n = mag_z*gp;

  if(y_n == 0) {
    if(x_n < 0) {
      angle = 180.0;
    }
    else {
      angle = 0.0;
    }
  }
  else if(y_n > 0) {
    angle = 90 - atan2(x_n,y_n)*180/PI;
  }
  else {
    if(x_n < 0) {
      angle = 90 + atan2(-x_n,y_n)*180/PI;
    }
    else {
      angle = 270 + 180 - atan2(x_n,y_n)*180/PI;
    }
  }
  _angle = angle;
  // return angle;
}

double Magnometer::getAngle() {
  return _angle;
}

