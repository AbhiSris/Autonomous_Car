/*
 * Ultrasonic.h- Library for sending Ultrasonic sensor outputs
 * HEADER FILE
 */

#ifndef Ultrasonic_h
#define Ultrasonic_h

#include "Arduino.h"

class Ultrasonic {
  public:
  	Ultrasonic(int echo, int init);	
  	void initPinModes();
    void calc_distance();
    int get_dist();
    int print_distance();
  private:
    int _echo;
    int _init;
    int _dist;
};

#endif