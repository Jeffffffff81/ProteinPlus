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

const int tempSensor = A2;

float duration;
float distance;
float readTemp;
float temperature;
float soundSpeed;

// Ranging distance for HC-SR05
const int maxRange = 500;
const int minRange = 2;
const float distThreshold = 30;

float dist;
float leftDist;
float rightDist;

void setup()
{
  Serial.begin(9600);
  pinMode(10, OUTPUT);

  //setup for wheels
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  //setup for servo
  myservo.attach(ServoPin);

  //setup for distance sensor

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(10, HIGH);
  distance = getDistance(); //bug fix/hack for distance = .1 initially
}

void loop()
{
  accelerate(255, 4);
  distance = getDistance();
  avoidWall();
  delay(800);
  
  int dist = turnAndCheck(60, 30, 2, 1);
  if (dist < 20) {
    Serial.println("turn right!");
  }

  accelerate(255, 4);
  distance = getDistance();
  avoidWall();
  delay(800);

  dist = turnAndCheck(-60, 30, 2, 1);
  if (dist < 20) {
    Serial.println("turn left!");
  }
}

//**************HIGHLEVELFUNCTIONS******************//
/*
 * Checks to see if its going to crash into a wall and take corrective action
 */
void avoidWall(){
    if (distance < distThreshold) {
    accelerate(0, 8);
    sweep(1000, 15);

    if (leftDist < distThreshold && rightDist < distThreshold) {
      turn(false, 1000, 235);
      turn(false, 1000, 235);
    }
    else if (leftDist > rightDist) {
      turn(false, 1000, 235);
    }
    else {
      turn(true, 1000, 235);
    }
  }
}


/*
   Function: sweep - Rotates the servo motor so it turns 90 degrees to the left and then to the right.

   parameter: wait_time - the amount of time that the servo motor pauses when it faces 90 degrees to the left and right.

   parameter: turn_speed - used to control how fast the servo motors position changes.
*/
void sweep(int wait_time, int turn_speed) {
  
  leftDist = turnAndCheck(90, wait_time, turn_speed, 10);

  rightDist = turnAndCheck(-90, wait_time, turn_speed, 10);

}


//CHENs problem
void slowDown(int standOff){
  //currSpeed;
  while(getDistance() > standOff) {
    accelerate(getDistance(), 1);
  }
}

//**************LOWLEVELFUNCTIONS******************//
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
      analogWrite(E1, currentSpeed);   
      analogWrite(E2, currentSpeed);   
      delay(1);
    }
    analogWrite(E1, finalSpeed);   
    analogWrite(E2, finalSpeed);   
  } else if (finalSpeed < currentSpeed) {
    for (currentSpeed;  currentSpeed > finalSpeed; currentSpeed -= speedChange) {
  
      analogWrite(E1, currentSpeed);
      analogWrite(E2, currentSpeed);   
      delay(1);
    }
    analogWrite(E1, finalSpeed);
    analogWrite(E2, finalSpeed);
  }
}

/*
 * Turns servo to dir degrees (0 is straight ahead), then returns the distance,
 */
int turnAndCheck(int dir, int wait_time, int turn_speed, int count) {
  int pos;
  //If turning left
  for (pos = 90; pos <= dir+90; pos += 1) {
    myservo.write(pos);
    delay(turn_speed);
  }
  //If turning right
  for (pos = 90; pos >= dir+90; pos -= 1) {
    myservo.write(pos);
    delay(turn_speed);
  }

  delay(wait_time / 2);
  int dist = getLowest(count);
  delay(wait_time / 2);
  
  //Return from left
  for (pos = dir+90; pos >= 90; pos -= 1) {
    myservo.write(pos);
    delay(turn_speed);
  }
  //Return from right
  for (pos = dir+90; pos <= 90; pos += 1) {
    myservo.write(pos);
    delay(turn_speed);
  }
  return dist;
}

/*
 * Reads distance multiple times and returns
 * the lowest
 */
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
