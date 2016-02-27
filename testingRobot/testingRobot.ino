#include <Servo.h>

//PINS:
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

Servo myservo;
int ServoPin = 9;

const int trigPin = 13;
const int echoPin = 12;
const int tempSensor = A2;

//GLOBAL VARIABLES:
int distance;
int currentSpeed = 0;
int servoPos;

void setup()
{
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  myservo.attach(ServoPin);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  distance = getDistance(); //bug fix/hack for distance = .1 initially
}

void loop()
{
  int delta;
  changeSpeed(255, 4);
  avoidWall(30);
  delay(800);

  delta = turnAndCheckDelta(70, 10, 3, 4);
  slowLeftWheel(delta, 500);

  changeSpeed(255, 4);
  avoidWall(30);
  delay(800);

  delta = turnAndCheckDelta(-70, 10, 3, 4);
  slowRightWheel(delta, 500);
}

//**************HIGHLEVELFUNCTIONS******************//
/*
   Checks to see if its going to crash into a wall and take corrective action
*/
void avoidWall(int distThreshold) {
  distance = getDistance();
  if (distance < distThreshold) {
    changeSpeed(0, 8);
    int leftDist = turnAndCheck(90, 1000, 15, 10);
    int rightDist = turnAndCheck(-90, 1000, 15, 10);

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


//CHENs problem
void slowDown(int standOff) {
  //currSpeed;
  while (getDistance() > standOff) {
    changeSpeed(getDistance(), 1);
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
  float duration;
  float readTemp;
  float temperature;
  float soundSpeed;
  float thisDistance;

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
  thisDistance = duration / (2 * 29); // improve measurement precision (change soundspeed to 29)

  return thisDistance;
}

void changeSpeed(int finalSpeed, int acceleration) {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  if (finalSpeed > currentSpeed) {
    for (currentSpeed;  currentSpeed < finalSpeed; currentSpeed += acceleration) {
      analogWrite(E1, currentSpeed);
      analogWrite(E2, currentSpeed);
      delay(1);
    }
    analogWrite(E1, finalSpeed);
    analogWrite(E2, finalSpeed);
  } else if (finalSpeed < currentSpeed) {
    for (currentSpeed;  currentSpeed > finalSpeed; currentSpeed -= acceleration) {

      analogWrite(E1, currentSpeed);
      analogWrite(E2, currentSpeed);
      delay(1);
    }
    analogWrite(E1, finalSpeed);
    analogWrite(E2, finalSpeed);
  }
}

/*
   Turns servo to dir degrees (0 is straight ahead), then returns change in distance,
*/
int turnAndCheckDelta(int dir, int wait_time, int turn_speed, int count) {
  int pos;
  //If turning left
  for (pos = 90; pos <= dir + 90; pos += 1) {
    myservo.write(pos);
    delay(1 / turn_speed);
  }
  //If turning right
  for (pos = 90; pos >= dir + 90; pos -= 1) {
    myservo.write(pos);
    delay(1 / turn_speed);
  }

  int initialDist = getLowest(count);
  delay(wait_time);
  int finalDist = getLowest(count);

  //Return from left
  for (pos = dir + 90; pos >= 90; pos -= 1) {
    myservo.write(pos);
    delay(1 / turn_speed);
  }
  //Return from right
  for (pos = dir + 90; pos <= 90; pos += 1) {
    myservo.write(pos);
    delay(1 / turn_speed);
  }
  //delta should not depend on wait time. so divide by wait time:
  return (initialDist - finalDist)/wait_time;
}

/*
   Turns and checks distance
*/
int turnAndCheck(int dir, int wait_time, int turn_speed, int count) {
  int pos;
  //If turning left
  for (pos = 90; pos <= dir + 90; pos += 1) {
    myservo.write(pos);
    delay(1 / turn_speed);
  }
  //If turning right
  for (pos = 90; pos >= dir + 90; pos -= 1) {
    myservo.write(pos);
    delay(1 / turn_speed);
  }

  delay(wait_time / 2);
  int dist = getLowest(count);
  delay(wait_time / 2);

  //Return from left
  for (pos = dir + 90; pos >= 90; pos -= 1) {
    myservo.write(pos);
    delay(1 / turn_speed);
  }
  //Return from right
  for (pos = dir + 90; pos <= 90; pos += 1) {
    myservo.write(pos);
    delay(1 / turn_speed);
  }
  return dist;
}

/*
   Reads distance multiple times and returns
   the lowest
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

/*
   Slow whell by percentage between 0 and 1, for certain length of time.
*/
void slowLeftWheel(double percent, int time) {
  if (percent > 1 || percent < 1) {
    return;
  }
  digitalWrite(E1, currentSpeed - (percent * currentSpeed));
  delay(time);
  digitalWrite(E1, currentSpeed);
}

void slowRightWheel(double percent, int time) {
  if (percent > 1 || percent < 1) {
    return;
  }
  digitalWrite(E2, currentSpeed - (percent * currentSpeed));
  delay(time);
  digitalWrite(E2, currentSpeed);
}


