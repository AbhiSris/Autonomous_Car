/*
 * Ultrasonic.cpp - Library for sending Ultrasonic sensor outputs
 */

#include "Arduino.h"
#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int echo, int init) {
  _echo = echo;
  _init = init;
}

void Ultrasonic::initPinModes() {
  pinMode(_init, OUTPUT);
  pinMode(_echo, INPUT);
}

void Ultrasonic::calc_distance() {
  digitalWrite(_init, HIGH);
  delayMicroseconds(10);
  digitalWrite(_init, LOW);
  unsigned long pulseTime = pulseIn(_echo, HIGH);
  int distance = (pulseTime/2) / 29.1;
  _dist = distance;
  // return distance;
}

int Ultrasonic::get_dist() {
  return _dist;
}

int Ultrasonic::print_distance() {
  if (_dist >= 200 || _dist <= 0) {
    Serial.println("Out of Range");
  }
  else {
    Serial.print(_dist);
    Serial.println(" cm");
  }
}