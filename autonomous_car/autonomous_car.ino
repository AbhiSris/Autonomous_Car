#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <math.h>
#include <Mag.h>

// Define constants for motors
#define INIT_FRONT_SPEED 240
#define INIT_BACK_SPEED 40
#define TURN_SPEED 50
#define INCREASE_FRONT_MOTOR 10
#define US_UPPER_LIMIT 200
#define US_LOWER_LIMIT 0
#define SERVO_INIT 70
#define OBSTACLE_DIST 25
#define TURNING_ANGLE 45
#define ACCEL_TUNING 5

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
bool turningStepperOrigFlag = false;
bool obstacleFlag = false;
bool leftFlag = false;
bool rightFlag = false;
volatile int startBearing;
volatile int currBearing;
volatile int tuning_num = 0;

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

//  backMotor->run(FORWARD);

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
    backMotor->run(RELEASE);
    getAccelReadings();
    startBearing = mag.getAngle();
    frontMotor->run(FORWARD);
    turningFlag = true;
    rightFlag = true;
    obstacleFlag = true;
  }

  // State 2: left sensor sees something, turn right
  else if (distVals[0] < OBSTACLE_DIST  && !leftFlag && !rightFlag && !turningFlag) {
    state = 2;
    Serial.println(state);
    backMotor->run(RELEASE);
    getAccelReadings();
    startBearing = mag.getAngle();
    frontMotor->run(BACKWARD);
    turningFlag = true;
    rightFlag = true;
    obstacleFlag = true;
  }

  // State 3: right sensor sees something, turn left
  else if (distVals[1] < OBSTACLE_DIST  && !leftFlag && !rightFlag && !turningFlag) {
    state = 3;
    Serial.println(state);
    backMotor->run(RELEASE);
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
    while (tuning_num < ACCEL_TUNING) {
      getAccelReadings();
      startBearing = mag.getAngle();
      tuning_num = tuning_num + 1;
    }
    tuning_num = 0;
    backMotor->setSpeed(TURN_SPEED);
    backMotor->run(FORWARD);
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
    backMotor->run(RELEASE);
    backMotor->setSpeed(INIT_BACK_SPEED);
    frontMotor->run(RELEASE);
    turningFlag = false;
    turningStepperFlag = true;
  }

  // State 5: Turn stepper to face the obstacle
  if (turningStepperFlag && !turningStepperOrigFlag) {
    state = 5;
    Serial.println(state);
    if (startBearing > currBearing && turningStepperFlag) {
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
      turningStepperFlag == false;
    }
    else if (startBearing < currBearing && turningStepperFlag){
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
      turningStepperFlag == false;
    }
    backMotor->run(FORWARD);
  }

  // State 7: both sensors see something, turn right for now
  if (distVals[0] > OBSTACLE_DIST && distVals[1] > OBSTACLE_DIST && leftFlag && !turningFlag && obstacleFlag) {
    state = 7;
    Serial.println(state);
    backMotor->run(RELEASE);
    getAccelReadings();
    currBearing = mag.getAngle();
//    frontMotor->run(FORWARD);
//    turningFlag = true;
    leftFlag = false;
    rightFlag = true;
    turningStepperFlag = true;
    turningStepperOrigFlag = true;
  }

  // State 8: both sensors see something, turn right for now
  if (distVals[0] > OBSTACLE_DIST && distVals[1] > OBSTACLE_DIST && rightFlag && !turningFlag && obstacleFlag) {
    state = 8;
    Serial.println(state);
    backMotor->run(RELEASE);
    getAccelReadings();
    currBearing = mag.getAngle();
//    frontMotor->run(BACKWARD);
//    turningFlag = true;
    leftFlag = true;
    rightFlag = false;
    turningStepperFlag = true;
    turningStepperOrigFlag = true;
  }

  if (turningStepperFlag && turningStepperOrigFlag) {
    state = 10;
    Serial.println(state);
//    if (pos < SERVO_INIT && turningStepperOrigFlag) {
//      while (pos < SERVO_INIT) {
//        
//        pos += 1;
//        myservo.write(pos);
//        Serial.println(pos);
//        delay(15);
//        getAccelReadings();
//        Serial.println("");
//        currBearing = mag.getAngle();
//        Serial.print("start: ");
//        Serial.print(startBearing);
//        Serial.print("curr: ");
//        Serial.println(currBearing);
//      }
//      turningStepperFlag == false;
//      turningStepperOrigFlag == false;
//    }
//    else if (pos > SERVO_INIT && turningStepperOrigFlag){
//      while (pos > SERVO_INIT) {
//        pos -= 1;
//        myservo.write(pos);
//        Serial.println(pos);
//        delay(15);
//        getAccelReadings();
//        Serial.println("");
//        currBearing = mag.getAngle();
//        Serial.print("start: ");
//        Serial.print(startBearing);
//        Serial.print("curr: ");
//        Serial.println(currBearing);
//      }

      if ((abs(pos - SERVO_INIT) > 20) && turningStepperOrigFlag){
        pos = SERVO_INIT;
        myservo.write(pos);
        delay(500);
      }
        
      turningStepperFlag == false;
      turningStepperOrigFlag == false;
      turningFlag = true;
      obstacleFlag = false;
  }

  // State 9: while turning, stop at 45 deg
  if (turningFlag && !obstacleFlag) {
    state = 9;
    Serial.println(state);
//    while (tuning_num < ACCEL_TUNING) {
//      getAccelReadings();
//      startBearing = mag.getAngle();
//      tuning_num = tuning_num + 1;
//    }
    tuning_num = 0;
    if (leftFlag) {
      frontMotor->run(FORWARD);
    }
    else if (rightFlag) {
      frontMotor->run(BACKWARD);
    }
    backMotor->setSpeed(TURN_SPEED);
    backMotor->run(FORWARD);
    getAccelReadings();
    currBearing = mag.getAngle();

    if (startBearing - currBearing > 0 && turningFlag) {
      while (startBearing - currBearing > 0) {
        getAccelReadings();
        Serial.println("");
        currBearing = mag.getAngle();
        Serial.print("start: ");
        Serial.print(startBearing);
        Serial.print("curr: ");
        Serial.println(currBearing);
      }
      turningFlag = false;
    }
    else if (startBearing - currBearing < 0 && turningFlag) {
      while (startBearing - currBearing < 0) {
        getAccelReadings();
        Serial.println("");
        currBearing = mag.getAngle();
        Serial.print("start: ");
        Serial.print(startBearing);
        Serial.print("curr: ");
        Serial.println(currBearing);
      }
      turningFlag = false;
    }
    
//    while ( abs(startBearing - currBearing) > 1) {
//      getAccelReadings();
//      Serial.println("");
//      currBearing = mag.getAngle();
//      Serial.print("start: ");
//      Serial.print(startBearing);
//      Serial.print("curr: ");
//      Serial.println(currBearing);
//    }
    backMotor->run(RELEASE);
    frontMotor->run(RELEASE);
    backMotor->setSpeed(INIT_BACK_SPEED);
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
  mag.setAngle(gp);

  if (mag.getCount() > 5) {

    if ((mag.getPrevAngle() - mag.getAngle()) > 250) {
      mag.incrementTurns();
    }
    else if ((mag.getAngle()) - mag.getPrevAngle() > 250) {
      mag.decrementTurns();
    }
    else {
      mag.setPrevAngle(mag.getAngle());
    }

    mag.setCount();
  }
  
  mag.incrementCount();



  Serial.print("Prev Angle is ");
  Serial.print(mag.getPrevAngle());
  Serial.print("  Curr Angle is ");
  Serial.println(mag.getAngle());
  
}



