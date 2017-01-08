#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// Define constants for motors
#define INIT_FRONT_SPEED 75
#define INIT_BACK_SPEED 30
#define INCREASE_FRONT_MOTOR 10
#define US_UPPER_LIMIT 200
#define US_LOWER_LIMIT 0
#define SERVO_INIT 75

// Motor definitions
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *frontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backMotor = AFMS.getMotor(3);
Servo myservo;

int pos = SERVO_INIT;

int mOn = 8;
int mOff = 12;

int start_button = 0;
int left_button = 0;
int right_button = 0;

//Sonar 1
int echoPinL =2;
int trigPinL =3;
int distanceL =0;

//Sonar 2
int echoPinR =4;
int trigPinR =5;
int distanceC =0;

int distVals[2] = {0,0};

void setup() {
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz

  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(mOn, INPUT);
  pinMode(mOff, INPUT);

  frontMotor->setSpeed(INIT_FRONT_SPEED);
  backMotor->setSpeed(INIT_BACK_SPEED);

  myservo.attach(9); 

  // Loop won't start without clicking the start button
  start_button = digitalRead(mOn);
  while (!start_button) {
    start_button = digitalRead(mOn);
  }
  //motor_forward(backMotor);
  myservo.write(pos);
  

}

/*
 * LOOP FOR CODE
 */

void loop() {
  
//    getAllDistances();
//
//    if (distVals[1] < 50) {
//      motor_off(backMotor);
//    }
    
//    start_button = digitalRead(mOn);
//    if (start_button) {
//      motor_forward(backMotor);
//    }

    left_button = digitalRead(mOn);
    if (left_button) {
      pos-=1;
      myservo.write(pos);
//      Serial.println(pos);
      delay(15);
    }
    right_button = digitalRead(mOff);
    if (right_button) {
      pos+=1;
      myservo.write(pos);
//      Serial.println(pos);
      delay(15);
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
  distVals[1] = getDistance(trigPinR, echoPinR);

  Serial.print("L: ");
  printDistance(distVals[0]);
  Serial.print("R: ");
  printDistance(distVals[1]);
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



