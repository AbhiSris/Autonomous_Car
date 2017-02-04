#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <math.h>
#include <Mag.h>

// Define constants for motors
#define INIT_FRONT_SPEED 150
#define INIT_BACK_SPEED 30
#define INCREASE_FRONT_MOTOR 10
#define US_UPPER_LIMIT 200
#define US_LOWER_LIMIT 0
#define SERVO_INIT 75
#define OBSTACLE_DIST 20
#define TURNING_ANGLE 45

// Motor definitions
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *frontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backMotor = AFMS.getMotor(3);
Servo myservo;

/*
   CODE FOR AUTONOMOUS
*/

bool turningFlag = false;
bool turningStepperFlag = false;
bool obstacleFlag = false;
bool leftFlag = false;
bool rightFlag = false;
volatile int startBearing;
volatile int currBearing;

int pos = SERVO_INIT;

int stopButton = 6;

int buttonState = 0;

//Sonar 1
int echoPinL = 2;
int trigPinL = 3;
int distanceL = 0;

//Sonar 2
int echoPinR = 4;
int trigPinR = 5;
int distanceC = 0;

// Mag sensor value
double gp = 1.3;

int distVals[2] = {0, 0};

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

Magnometer mag;

int state = 0;

void setup() {
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz

  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  frontMotor->setSpeed(INIT_FRONT_SPEED);
  backMotor->setSpeed(INIT_BACK_SPEED);

  myservo.attach(9);

//  // Loop won't start without clicking the start button
//  start_button = digitalRead(mOn);
//  while (!start_button) {
//    start_button = digitalRead(mOn);
//  }
  //motor_forward(backMotor);
  myservo.write(pos);

  mag.mag_setup();

  backMotor->run(FORWARD);

  pinMode(stopButton, INPUT);

}

/*
   LOOP FOR CODE
*/

void loop() {
  /*
     LOGIC FOR AUTONOMOUS
  */

  buttonState = digitalRead(stopButton);

  

  getAccelReadings();

    Serial.println(state);
    getAllDistances();

  if (stopButton == HIGH) {
    frontMotor->run(RELEASE);
    backMotor->run(RELEASE);
  }
  

  // State 1: both sensors see something, turn right for now
  if (distVals[0] < OBSTACLE_DIST && distVals[1] < OBSTACLE_DIST && !leftFlag && !rightFlag && !turningFlag) {
    state = 1;
    Serial.println(state);
    getAccelReadings();
    startBearing = mag.getAngle();
    frontMotor->run(RELEASE);
    turningFlag = true;
    rightFlag = true;
    obstacleFlag = true;
  }

  // State 2: left sensor sees something, turn right
  else if (distVals[0] < OBSTACLE_DIST  && !leftFlag && !rightFlag && !turningFlag) {
    state = 2;
    Serial.println(state);
    getAccelReadings();
    startBearing = mag.getAngle();
    frontMotor->run(FORWARD);
    turningFlag = true;
    rightFlag = true;
    obstacleFlag = true;
  }

  // State 3: right sensor sees something, turn left
  else if (distVals[1] < OBSTACLE_DIST  && !leftFlag && !rightFlag && !turningFlag) {
    state = 3;
    Serial.println(state);
    getAccelReadings();
    startBearing = mag.getAngle();
    frontMotor->run(BACKWARD);
    turningFlag = true;
    leftFlag = true;
    obstacleFlag = true;
  }

  // State 4: while turning, stop at 45 deg
  if (turningFlag && obstacleFlag) {
    state = 4;
    Serial.println(state);
    getAccelReadings();
    currBearing = mag.getAngle();
    while ( abs(startBearing - currBearing) < TURNING_ANGLE) {
      getAccelReadings();
      Serial.println("");
      currBearing = mag.getAngle();
      Serial.print("start: ");
      Serial.print(startBearing);
      Serial.print("curr: ");
      Serial.println(currBearing);
    }
    frontMotor->run(RELEASE);
    turningFlag = false;
    turningStepperFlag = true;
  }

  // State 5: Turn stepper to face the obstacle
  if (turningStepperFlag) {
    state = 5;
    Serial.println(state);
    if (startBearing > currBearing) {
      while (startBearing > currBearing) {
        pos += 1;
        myservo.write(pos);
        Serial.println(pos);
        delay(15);
        getAccelReadings();
        Serial.println("");
        currBearing = mag.getAngle();
        Serial.print("start: ");
        Serial.print(startBearing);
        Serial.print("curr: ");
        Serial.println(currBearing);
      }
    }
    else {
      while (startBearing < currBearing) {
        pos -= 1;
        myservo.write(pos);
        Serial.println(pos);
        delay(15);
        getAccelReadings();
        Serial.println("");
        currBearing = mag.getAngle();
        Serial.print("start: ");
        Serial.print(startBearing);
        Serial.print("curr: ");
        Serial.println(currBearing);
      }
    }
    turningStepperFlag == false;
  }

  // State 7: both sensors see something, turn right for now
  if (distVals[0] > OBSTACLE_DIST && distVals[1] > OBSTACLE_DIST && leftFlag && !turningFlag && obstacleFlag) {
    state = 7;
    Serial.println(state);
    getAccelReadings();
    startBearing = mag.getAngle();
    frontMotor->run(FORWARD);
    turningFlag = true;
    leftFlag = false;
    rightFlag = true;
    obstacleFlag = false;
  }

  // State 8: both sensors see something, turn right for now
  if (distVals[0] > OBSTACLE_DIST && distVals[1] > OBSTACLE_DIST && rightFlag && !turningFlag && obstacleFlag) {
    state = 8;
    Serial.println(state);
    getAccelReadings();
    startBearing = mag.getAngle();
    frontMotor->run(BACKWARD);
    turningFlag = true;
    rightFlag = false;
    leftFlag = true;
    obstacleFlag = false;
  }

  // State 9: while turning, stop at 45 deg
  if (turningFlag && !obstacleFlag) {
    state = 9;
    Serial.println(state);
    getAccelReadings();
    currBearing = mag.getAngle();
    while ( abs(startBearing - currBearing) < TURNING_ANGLE) {
      getAccelReadings();
      Serial.println("");
      currBearing = mag.getAngle();
      Serial.print("start: ");
      Serial.print(startBearing);
      Serial.print("curr: ");
      Serial.println(currBearing);
    }
    frontMotor->run(RELEASE);
    turningFlag = false;
    turningStepperFlag = true;
    leftFlag = false;
    rightFlag = false;
  }

}

/*
   ULTRASONIC CONTROL CODE
*/
int getDistance (int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long pulseTime = pulseIn(echoPin, HIGH);
  int distance = (pulseTime / 2) / 29.1;
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

void printDistance(int dist) {
//  if (dist >= US_UPPER_LIMIT || dist <= US_LOWER_LIMIT ) {
//    Serial.println(" Out of range");
//  }
//  else {
    Serial.print(dist);
    Serial.println(" cm");
//  }
}

/*
   MOTOR CONTROL CODE
*/
void motor_forward(Adafruit_DCMotor *inputMotor) {
  inputMotor->run(FORWARD);
  delay(10);
}
void motor_reverse(Adafruit_DCMotor *inputMotor) {
  inputMotor->run(BACKWARD);
  delay(10);
}
void motor_off(Adafruit_DCMotor *inputMotor) {
  inputMotor->run(RELEASE);
  delay(10);
}

/*
   ACCEL CONTROL CODE
*/

void getAccelReadings() {
  mag.start_comm();
  mag.mag_read_write();


  Serial.print("Angle is ");
  mag.setAngle(gp);
  Serial.println(mag.getAngle());
}



