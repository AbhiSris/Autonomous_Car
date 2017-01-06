#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(3);

int mOn = 8;
int mOff = 9;

int bs1 = 0;
int bs2 = 0;

//Sonar 1
int echoPin1 =2;
int initPin1 =3;
int distance1 =0;

//Sonar 2
int echoPin2 =4;
int initPin2 =5;
int distance2 =0;

//Sonar 3
int echoPin3 =6;
int initPin3 =7;
int distance3 =0;

void setup() {

  pinMode(initPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(initPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(initPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz

  myMotor1->setSpeed(75);
  myMotor2->setSpeed(75);

  pinMode(mOn, INPUT);
  pinMode(mOff, INPUT);

}

void loop() {
  bs1 = digitalRead(mOn);
  bs2 = digitalRead(mOff);

//  Serial.print("L: ");
//  distance1 = getDistance(initPin1, echoPin1);
//  printDistance(distance1);
//  delay(150);
//    
//  Serial.print("C: ");  
//  distance2 = getDistance(initPin2, echoPin2);
//  printDistance(distance2);
//  delay(150);
//  
//   Serial.print("R: ");  
//  distance3 = getDistance(initPin3, echoPin3);
//  printDistance(distance3);
//  delay(150);
//
//  Serial.println(" ");

  

  if (bs1){
    motorF();
  }
  else if (bs2){
    motorOff();
  }

  //delay(500);

}

int getDistance (int initPin, int echoPin){

  digitalWrite(initPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(initPin, LOW); 
  unsigned long pulseTime = pulseIn(echoPin, HIGH); 
  int distance = (pulseTime/2) / 29.1;
  return distance;

}

void printDistance(int dist){
  if (dist >= 200 || dist <= 0 ){
    Serial.println(" Out of range");
  }
  else{
    Serial.print(dist);
    Serial.println(" cm");
  }
}

void motorF(){
  myMotor2->run(FORWARD);
  Serial.println("case 1");
  delay(10);
} 
void motorR(){
  myMotor2->run(BACKWARD);
  delay(10);
}
void motorOff(){
  myMotor2->run(RELEASE);
  delay(10);
}



