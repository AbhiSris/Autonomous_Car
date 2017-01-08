#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Define constants for motors
#define INIT_FRONT_SPEED 75
#define INIT_BACK_SPEED 30
#define INCREASE_FRONT_MOTOR 10
#define US_UPPER_LIMIT 200
#define US_LOWER_LIMIT 0

// Motor definitions
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *frontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backMotor = AFMS.getMotor(3);

int mOn = 8;
int mOff = 9;

int start_button = 0;

//Sonar 1
int echoPinL =2;
int trigPinL =3;
int distanceL =0;

//Sonar 2
int echoPinC =4;
int trigPinC =5;
int distanceC =0;

//Sonar 3
int echoPinR =6;
int trigPinR =7;
int distanceR =0;

int distVals[3] = {0,0,0};

void setup() {
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz

  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinC, OUTPUT);
  pinMode(echoPinC, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(mOn, INPUT);
  pinMode(mOff, INPUT);

  frontMotor->setSpeed(INIT_FRONT_SPEED);
  backMotor->setSpeed(INIT_BACK_SPEED);

  // Loop won't start without clicking the start button
  start_button = digitalRead(mOn);
  while (!start_button) {
    start_button = digitalRead(mOn);
  }
  motor_forward(backMotor);
  

}

void loop() {
  
    getAllDistances();

    if (distVals[1] < 50) {
      motor_off(backMotor);
    }
    
    start_button = digitalRead(mOn);
    if (start_button) {
      motor_forward(backMotor);
    }
}

/* 
 *  ULTRASONIC CONTROL CODE
 */
int getDistance (int trigPin, int echoPin){
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW); 
  unsigned long pulseTime = pulseIn(echoPin, HIGH); 
  int distance = (pulseTime/2) / 29.1;
  return distance;
}

void getAllDistances () {
  distVals[0] = getDistance(trigPinL, echoPinL);
  distVals[1] = getDistance(trigPinC, echoPinC);
  distVals[2] = getDistance(trigPinR, echoPinR);

  Serial.print("L: ");
  printDistance(distVals[0]);
  Serial.print("C: ");
  printDistance(distVals[1]);
  Serial.print("R: ");
  printDistance(distVals[2]);
  Serial.println(" ");
  delay(150);
}

void printDistance(int dist){
  if (dist >= US_UPPER_LIMIT || dist <= US_LOWER_LIMIT ) {
    Serial.println(" Out of range");
  }
  else {
    Serial.print(dist);
    Serial.println(" cm");
  }
}

/*
 *  MOTOR CONTROL CODE 
 */
void motor_forward(Adafruit_DCMotor *inputMotor){
  inputMotor->run(FORWARD);
  delay(10);
} 
void motor_reverse(Adafruit_DCMotor *inputMotor){
  inputMotor->run(BACKWARD);
  delay(10);
}
void motor_off(Adafruit_DCMotor *inputMotor){
  inputMotor->run(RELEASE);
  delay(10);
}



