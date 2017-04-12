#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <Mag.h>
#include <ServoTimer2.h>  // the servo library
#include <Adafruit_GPS.h> //Load the GPS Library. Make sure you have installed the library form the adafruit site above
#include <SoftwareSerial.h> //Load the Software Serial Library. This library in effect gives the arduino additional serial ports

#define rollPin  9

// Define constants for motors
#define INIT_FRONT_SPEED 240
#define INIT_BACK_SPEED 40
#define TURN_SPEED 50
#define INCREASE_FRONT_MOTOR 10
#define US_UPPER_LIMIT 200
#define US_LOWER_LIMIT 0
#define OBSTACLE_DIST 25
#define TURNING_ANGLE 45
#define ACCEL_TUNING 5
#define SERVO_MAX 1760
#define SERVO_INIT 1260
#define SERVO_MIN 760
#define GPS_CHECK 20

// Motor definitions
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *frontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backMotor = AFMS.getMotor(3);
ServoTimer2 servoRoll;

/*
   CODE FOR AUTONOMOUS
*/

bool turningFlag = false;
bool turningStepperFlag = false;
bool turningStepperOrigFlag = false;
bool obstacleFlag = false;
bool maneuverFlag = false;
bool leftFlag = false;
bool rightFlag = false;
volatile int startBearing;
volatile int currBearing;
volatile int tuning_num = 0;

int pos = SERVO_INIT;

int stopButton = 6;

float interval = 3000;
float currentMillis = 0;
float previousMillis = 0;

int buttonState = 0;

//Sonar 1
int echoPinL = 4;
int trigPinL = 5;
int distanceL = 0;

//Sonar 2
int echoPinR = 6;
int trigPinR = 7;
int distanceC = 0;

// Mag sensor value
double gp = 1.3;

int distVals[2] = {1000, 1000};

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

Magnometer mag;

int state = 0;
int GPS_counter = 0;

/*
 * GPS Declarations
 */

SoftwareSerial mySerial(3, 2); //Initialize SoftwareSerial, and tell it you will be connecting through pins 2 and 3
Adafruit_GPS GPS(&mySerial); //Create GPS object
 
String NMEA1;  //We will use this variable to hold our first NMEA sentence
String NMEA2;  //We will use this variable to hold our second NMEA sentence
char c;       //Used to read the characters spewing from the GPS module
int desiredLat = 0;
int desiredLon = 0;

void setup() {
  servoRoll.attach(rollPin);
  
  Serial.begin(9600);
  GPS.begin(9600);       //Turn GPS on at baud rate of 9600
  GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  delay(1000);  //Pause
  
  AFMS.begin();  // create with the default frequency 1.6KHz

  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  frontMotor->setSpeed(INIT_FRONT_SPEED);
  backMotor->setSpeed(INIT_BACK_SPEED);

  //  // Loop won't start without clicking the start button
  //  start_button = digitalRead(mOn);
  //  while (!start_button) {
  //    start_button = digitalRead(mOn);
  //  }
  //motor_forward(backMotor);

  servoRoll.write(pos);

  mag.mag_setup();

//  backMotor->run(FORWARD);

  pinMode(stopButton, INPUT);
  maneuverFlag = false;

}

/*
   LOOP FOR CODE
*/

void loop() { 

  getAccelReadings();
  state = 0;
  Serial.println(state);

  unsigned long currentMillis = millis(); //currentMillis is the time that the arduino has been running in milliseconds

  if(currentMillis - previousMillis>interval){  //should be a 3 second delay without the delay function
    previousMillis = currentMillis; 
    getAllDistances(); //run the ping sensor 
    } 
  else {
      readGPS(); 
    }

  if (distVals[0] < OBSTACLE_DIST || distVals[1] < OBSTACLE_DIST) {
    maneuverFlag = true;
  }
  if (GPS.fix) {
    Serial.println("");
    Serial.println(GPS.latitude, 4); 
    Serial.println(GPS.longitude, 4); 
    Serial.println("");
  }

//  while (maneuverFlag) {
//    maneuverCode();
//  }
}
/*
 * DIRECTION CODE
 */

void initDirection() {
  readGPS(); 
  firstLat = GPS.latitude;
  firstLong = GPS.longitude;

  volatile unsigned long prevDirTimer = millis();
  volatile unsigned long currDirTimer = millis();
  while (currDirTimer - prevDirTimer < 3000) {
    currDirTimer = millis();
  }

  readGPS();
  secondLat = GPS.latitude;
  secondLong = GPS.longitude;

  
}

/*
 * MANEUVER CODE
 */

void maneuverCode() {
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
      leftFlag = false;
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
      leftFlag = false;
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
      rightFlag = false;
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
      if (leftFlag && turningStepperFlag) {
          pos = SERVO_MAX;
          servoRoll.write(pos);
          Serial.println(pos);
          turningStepperFlag = false;
          delay(500);
      }
      else if (rightFlag && turningStepperFlag){
          pos = SERVO_MIN;
          servoRoll.write(pos);
          Serial.println(pos);
          turningStepperFlag = false;
          delay(500);
      }
      backMotor->run(FORWARD);
    }
  
    // State 6: both sensors see something, turn right for now
    if (distVals[0] > OBSTACLE_DIST && distVals[1] > OBSTACLE_DIST && leftFlag && !turningFlag && obstacleFlag) {
      state = 6;
      Serial.println(state);
      backMotor->run(RELEASE);
      getAccelReadings();
      currBearing = mag.getAngle();
      leftFlag = false;
      rightFlag = true;
      turningStepperFlag = true;
      turningStepperOrigFlag = true;
    }
  
    // State 7: both sensors see something, turn right for now
    if (distVals[0] > OBSTACLE_DIST && distVals[1] > OBSTACLE_DIST && rightFlag && !turningFlag && obstacleFlag) {
      state = 8;
      Serial.println(state);
      backMotor->run(RELEASE);
      getAccelReadings();
      currBearing = mag.getAngle();
      leftFlag = true;
      rightFlag = false;
      turningStepperFlag = true;
      turningStepperOrigFlag = true;
    }
  
    if (turningStepperFlag && turningStepperOrigFlag) {
      state = 10;
      Serial.println(state);
      if ( pos != SERVO_INIT /*(abs(pos - SERVO_INIT) > 20) */&& turningStepperOrigFlag){
        pos = SERVO_INIT;
        servoRoll.write(pos);
        delay(500);
      }
      turningStepperFlag == false;
      turningStepperOrigFlag == false;
      turningFlag = true;
      obstacleFlag = false;
    }
  
    // State 8: while turning, stop at 45 deg
    if (turningFlag && !obstacleFlag) {
      state = 9;
      Serial.println(state);
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
      backMotor->run(RELEASE);
      frontMotor->run(RELEASE);
      backMotor->setSpeed(INIT_BACK_SPEED);
      leftFlag = false;
      rightFlag = false;
    }
    maneuverFlag = false;
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

/*
 * GPS CODE
 */

 void readGPS() {  //This function will read and remember two NMEA sentences from GPS
  clearGPS();    //Serial port probably has old or corrupt data, so begin by clearing it all out
  while(!GPS.newNMEAreceived()) { //Keep reading characters in this loop until a good NMEA sentence is received
    c=GPS.read(); //read a character from the GPS
  }
  GPS.parse(GPS.lastNMEA());  //Once you get a good NMEA, parse it
  Serial.println(GPS.satellites);
  NMEA1=GPS.lastNMEA();      //Once parsed, save NMEA sentence into NMEA1

  Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  
//  while(!GPS.newNMEAreceived()) {  //Go out and get the second NMEA sentence, should be different type than the first one read above.
//    c=GPS.read();
//  }
//  GPS.parse(GPS.lastNMEA());
//  NMEA2=GPS.lastNMEA();
//  Serial.println(NMEA1);
//  Serial.println(NMEA2);
  Serial.println("");
}

void clearGPS() {  //Since between GPS reads, we still have data streaming in, we need to clear the old data by reading a few sentences, and discarding these
  while(!GPS.newNMEAreceived()) {
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  while(!GPS.newNMEAreceived()) {
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
}


