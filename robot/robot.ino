#include <Servo.h>

//Arduino PWM Speed Control：
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

int currentSpeed = 0;

//Servo declarations
Servo myservo;  // create servo object to control a servo
int ServoPin = 9;

//temperature sensor and HC-SR05 declarations
const int trigPin = 13;
const int echoPin = 12;
const int leftSensor = A2;
const int rightSensor = A3;

const int tempSensor = A1;

float duration;
float distance;
float readTemp;
float temperature;
float soundSpeed;

// Ranging distance for HC-SR05
const int maxRange = 500;
const int minRange = 2;
const float distThreshold = 30;
const int minDist = -5;

float dist;
float leftDist;
float rightDist;

/* copy the next four line */
/*******for straight line*******/
const int tolerableDiff = 100;
const float speedAdjustPercentage = 0.93;
const int correctionTime = 500;
unsigned long leftTime, rightTime;

/**********for turning**********/
float maxSpeed;

void setup()
{
  pinMode(10, OUTPUT);

  //setup for wheels
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  //setup for servo
  myservo.attach(ServoPin);  // attaches the servo on pin 9 to the servo object

  //setup for distance sensor
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);  // write to trigger input for Sensor
  pinMode(echoPin, INPUT);  // read from echo output from Sensor
  digitalWrite(10, HIGH);
  delay(3000);
  
  /*  Copy from here till the end of setup(). When test it, hold the robot in air unitil the calibration is 
   *  done. Notice that when it is done, both magnets will be aligned with its corresponding hall sensor, and
   *  there will be a 3 second delay for us to put it on the ground.
   */
  calibrate(leftSensor, rightSensor);
  leftTime = millis();
  rightTime = millis();
  distance = getDistance();
  maxSpeed = getMaxSpeed(leftSensor);
  currentSpeed = 255;
  
}

void loop()
{
  //accelerate(255, 4);
  distance = getDistance();
  //delay(200);
  //Serial.print("Distance: ");
  //Serial.println(distance);
  Serial.print(leftTime);
  Serial.print("    ");
  Serial.print(rightTime);
  Serial.print("    ");
  Serial.println(abs(leftTime - rightTime));
  

  /* temporarily comment out the slowdown function, improvements are needed and described below if   
   * straightline moving works. 
   * 
   * copy the rest of the code.
   * 
   * Also, copy the functions getMaxSpeed(), calibrate(), straightLineCorrection().
   */
   
  /*if (distance < distThreshold) {
    slowDown(currentSpeed, distance);

    /*
     * this whole part need to be re-written. It basically do the following few thing in order.
     * 1. slowdown (already done)
     * 2. sweep (it should be working)
     * 3. turn (need to work on it, specifically to find a way to control the angle of turning.
     * 4. accelerate to max speed.
     * 
     */
    /*sweep(1000, 15);
    Serial.print("Left Distance: ");
    Serial.println(leftDist);
    Serial.print("Right Distance: ");
    Serial.println(rightDist);
    delay(500);

    if (leftDist < distThreshold && rightDist < distThreshold) {
      //  Serial.print("180");
      turn(false, 1000, 235);
      delay(250);
      turn(false, 1000, 235);
    }
    else if (leftDist > rightDist) {
      turn(false, 1000, 235);
    }
    else {
      turn(true, 1000, 235);
    }
  }

  else {*/
    if (analogRead(leftSensor) < 50)
      leftTime = millis();
    if (analogRead(rightSensor) < 50)
      rightTime = millis();
    if (leftTime > rightTime) {
      if (leftTime - rightTime < 500)
        straightLineCorrection(leftTime, rightTime);
    }
    else {
      if (rightTime - leftTime < 500)
        straightLineCorrection(leftTime, rightTime);
    }
    
      
  //}
}



//FUNCTIONS****************************************************************

/*
   Function: sweep - Rotates the servo motor so it turns 90 degrees to the left and then to the right.
   parameter: wait_time - the amount of time that the servo motor pauses when it faces 90 degrees to the left and right.
   parameter: turn_speed - used to control how fast the servo motors position changes.
*/
void sweep(int wait_time, int turn_speed) {
  int dist1, dist2;
  int pos;
  for (pos = 90; pos <= 180; pos += 1) {        // goes from 90 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);                         // tell servo to go to position in variable 'pos'
    delay(turn_speed);                         // waits turn_speed ms for the servo to reach the position
  }
  delay(wait_time / 2);                          //pauses for wait_time ms at position 180
  leftDist = getLowest(10);
  //  leftDist = getDistance();

  // Serial.print("left: ");
  // Serial.print(leftDist);
  delay(wait_time / 2);

  for (pos = 180; pos >= 0; pos -= 1) {         // goes from 180 degrees to 0 degrees
    myservo.write(pos);                          // tell servo to go to position in variable 'pos'
    delay(turn_speed);                           // waits turn_speed ms for the servo to reach the position
  }
  delay(wait_time / 2);                         //pauses for wait_time ms at position 0
  rightDist = getLowest(10);
  //rightDist = getDistance();

  // Serial.print("right: ");
  //Serial.println(rightDist);
  delay(wait_time / 2);

  for (pos = 0; pos <= 90; pos += 1) {         // goes from 0 degrees to 90 degrees (back to the start position)
    // in steps of 1 degree
    myservo.write(pos);                       // tell servo to go to position in variable 'pos'
    delay(turn_speed);                       // waits turn_speed ms for the servo to reach the position
  }
}

/*
   Function - moveWheels - so far just for testing purposes
*/
void turn(boolean right, int time, int speed) {
  digitalWrite(M1, right);
  digitalWrite(M2, !right);
  analogWrite(E1, speed);   //PWM Speed Control
  analogWrite(E2, speed);   //PWM Speed Control
  delay(time);
  analogWrite(E1, 0);
  analogWrite(E2, 0);
  delay(time);
}

/*
   Function: getDistance - returns the distance measured  by the HC-SR05 and prints it onto the serial monitor
*/
float getDistance(void) {
  // set up HC-SR04 by applying a 10 microsecond pulse to it
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // reads the current temperature in degree Celsius
  readTemp = analogRead(tempSensor);
  temperature = (5.0 * readTemp * 100.0) / 1024;
  // calculate the corresponding speed of sound in m/s
  soundSpeed = 331.5 + (0.6 * temperature);
  // convert speed of sound from m/s to us/cm
  soundSpeed = 29 * soundSpeed / 343; //?

  // reads the duration of the pulse when echoPin is high
  duration = pulseIn(echoPin, HIGH);
  distance = duration / (2 * 29); // improve measurement precision (change soundspeed to 29)

  return distance;
}


void accelerate(int finalSpeed, int speedChange) {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  if (finalSpeed > currentSpeed) {
    for (currentSpeed;  currentSpeed < finalSpeed; currentSpeed += speedChange) {
      //Serial.println(currentSpeed);
      analogWrite(E1, currentSpeed);   //PWM Speed Control
      analogWrite(E2, currentSpeed);   //PWM Speed Control
      delay(1);
    }
    analogWrite(E1, finalSpeed);   //PWM Speed Control
    analogWrite(E2, finalSpeed);   //PWM Speed Control
  } else if (finalSpeed < currentSpeed) {
    for (currentSpeed;  currentSpeed > finalSpeed; currentSpeed -= speedChange) {
      //Serial.println(currentSpeed);
      analogWrite(E1, currentSpeed);   //PWM Speed Control
      analogWrite(E2, currentSpeed);   //PWM Speed Control
      delay(1);
    }
    analogWrite(E1, finalSpeed);   //PWM Speed Control
    analogWrite(E2, finalSpeed);   //PWM Speed Control
    //delay(1);
  }
}

int getAverage(int count) {
  int sum = 0;
  for (int i = 0; i < count; i++) {
    if (sum > 30000)
      return (int) (sum / count);
    sum += getDistance();
  }

  return (int) (sum / count);
}

int getLowest(int count) {
  int current;
  int lowest = getDistance();
  for (int i = 0; i < count; i++) {
    current = getDistance();
    if ( current < lowest ) {
      lowest = current;
    }
  }
  return lowest;
}

void slowDown (int initialSpeed, float initialDistance) {
  int currentSpeed = initialSpeed;
  int currentDistance = initialDistance;

  while (currentSpeed > 100) {
    currentSpeed = initialSpeed / (initialDistance - minDist) * (currentDistance - minDist);
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, currentSpeed);   //PWM Speed Control
    analogWrite(E2, currentSpeed);   //PWM Speed Control
    delay(30);
    currentDistance = getLowest(10);
  }
    analogWrite(E1, 0);   //PWM Speed Control
    analogWrite(E2, 0);   //PWM Speed Control
  
}

float getMaxSpeed(int sensorPin) {
  int wheelSpeed;
  unsigned long startTime, endTime;

  analogWrite(E1, 255);
  analogWrite(E2, 255);
  while (analogRead(sensorPin) < 100) {}
  startTime = millis();
  while (analogRead(sensorPin) > 100) {}
  endTime = millis();
  
  wheelSpeed = 0.2 / (float)(endTime - startTime) * 1000.0;
  return wheelSpeed;
}


void calibrate (int leftSensor, int rightSensor) {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  
  analogWrite(E1, 120); 
  while (analogRead(leftSensor) > 50) {}
  analogWrite(E1, 0);
  
  analogWrite(E2, 120);
  while (analogRead(rightSensor) > 50) {}
  analogWrite(E2, 0);

  delay(3000);
}

void straightLineCorrection (unsigned long leftTime, unsigned long rightTime) {
  if (abs(leftTime - rightTime) > tolerableDiff) {
    if (leftTime > rightTime) {
      analogWrite(E2, (int) (currentSpeed * speedAdjustPercentage));
      analogWrite(E1, currentSpeed);
      delay(correctionTime);
      analogWrite(E2, currentSpeed);
    }
    else {
      analogWrite(E1, (int) (currentSpeed * speedAdjustPercentage));
      analogWrite(E2, currentSpeed);
      delay(correctionTime);
      analogWrite(E1, currentSpeed);
    }
  }
}


