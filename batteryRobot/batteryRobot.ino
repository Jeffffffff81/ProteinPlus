#include <Servo.h>

//PINS:
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

Servo myservo;
const int ServoPin = 9;

const int trigPin = 13;
const int echoPin = 12;
const int tempPin = A5;
const int switchPin = 2;
const int OpticalPowerPin = 3;

const int leftHallPin = A2;
const int rightHallPin = A3;

//CONSTANTS:
const int MAX_DISTANCE = 1000;

//GLOBAL VARIABLES:
float distance;
int currentSpeed = 0;
int servoPos = 0;
/*******for straight line*******/
const int tolerableDiff = 50;
const float speedAdjustPercentage = 0.96;
const int correctionTime = 500;
unsigned long leftTime, rightTime;

void setup()
{
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(OpticalPowerPin, HIGH);

  myservo.attach(ServoPin);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  distance = getLowestDist(10); //bug fix/hack for distance = .1 initially
  calibrate();
}

void loop() {
  if (digitalRead(switchPin)) {
    digitalWrite(OpticalPowerPin, LOW);

    correctWheels();

    changeSpeed(255, 4);

    distance = getLowestDist(4);
    if (distance < 40) {
      avoidWall(40);

    }
  }

  else {
    digitalWrite(OpticalPowerPin, HIGH);
    blackLine(150);
  }

}
//**************HALLEFFECT*************************//
void correctWheels() {
  if (analogRead(leftHallPin) < 50)
    leftTime = millis();
  if (analogRead(rightHallPin) < 50) {
    rightTime = millis();
  }


  if (leftTime > rightTime) {
    if (leftTime - rightTime < 500)
      straightLineCorrection(leftTime, rightTime);
  }
  else {
    if (rightTime - leftTime < 500)
      straightLineCorrection(leftTime, rightTime);
  }
}

void straightLineCorrection (unsigned long leftTime, unsigned long rightTime) {
  if (abs(leftTime - rightTime) > tolerableDiff) {
    if (leftTime > rightTime) {
      analogWrite(E2, (int) (currentSpeed * speedAdjustPercentage));
      analogWrite(E1, currentSpeed);
    }
    else {
      analogWrite(E1, (int) (currentSpeed * speedAdjustPercentage));
      analogWrite(E2, currentSpeed);
    }
  }
}


void calibrate () {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  analogWrite(E1, 60);
  while (analogRead(leftHallPin) > 50) {}
  analogWrite(E1, 0);

  analogWrite(E2, 60);
  while (analogRead(rightHallPin) > 50) {}
  analogWrite(E2, 0);

  delay(2000);
}

//**************HIGHLEVELFUNCTIONS******************//
void blackLine(int sensorThreshold) {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  int LsensorValue;
  int MsensorValue;
  int RsensorValue;

  // read the input on analog pin 0:
  LsensorValue = analogRead(A4);
  delay(1);
  MsensorValue = analogRead(A1);
  delay(1);
  RsensorValue = analogRead(A0);
  delay(1);

  if (LsensorValue <= sensorThreshold && RsensorValue <= sensorThreshold ) {
    analogWrite(E1, 120);
    analogWrite(E2, 120);
  }

  // Case Steer Left - as long as left sensor detects black line
  else if (LsensorValue > sensorThreshold) {

    LsensorValue = analogRead(A4);
    analogWrite(E1, 20);
    analogWrite(E2, 100);
  }


  // Case Steer Right - as long as right sensor detects black line
  else if (RsensorValue > sensorThreshold) {

    RsensorValue = analogRead(A0);
    analogWrite(E1, 100);
    analogWrite(E2, 20);
  }

}


/*
   Checks to see if its going to crash into a wall and take corrective action
*/
void avoidWall(int distThreshold) {
  slowDown(5);
  //changeSpeed(0, 3);

  turnServo(90);
  delay(200);
  int leftDist = getLowestDist(10);
  delay(200);

  turnServo(-90);
  delay(200);
  int rightDist = getLowestDist(10);
  delay(200);
  turnServo(0);

  if (leftDist < distThreshold && rightDist < distThreshold) {
    turn(true, 250, 235);
    turn(true, 250, 235);
  }
  else if (leftDist > rightDist) {
    turn(true, 250, 235);
  }
  else {
    turn(false, 250, 235);
  }

}


//**************LOWLEVELFUNCTIONS******************//
/*
   Function - moveWheels - so far just for testing purposes
*/
void turn(boolean left, int time, int speed) {
  digitalWrite(M1, !left);
  digitalWrite(M2, left);
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
  float temperature;
  float thisDistance;

  temperature = (5.0 * analogRead(tempPin) * 100.0) / 1024;

  // send pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //receive pulse. Third argument is timout in microseconds
  duration = pulseIn(echoPin, HIGH, 100000);
  if (duration == 0) {
    return MAX_DISTANCE;
  }
  thisDistance  = (331.5 + (0.6 * temperature)) * duration / 2 * 100 / 1000000;

  return thisDistance;
}

void changeSpeed(int finalSpeed, int acceleration) {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  if (finalSpeed > currentSpeed) {
    for (currentSpeed;  currentSpeed < finalSpeed; currentSpeed += acceleration) {
      analogWrite(E1, currentSpeed);
      analogWrite(E2, currentSpeed);
      delay(20);
    }
    analogWrite(E1, finalSpeed);
    analogWrite(E2, finalSpeed);
  } else if (finalSpeed < currentSpeed) {
    for (currentSpeed;  currentSpeed > finalSpeed; currentSpeed -= acceleration) {

      analogWrite(E1, currentSpeed);
      analogWrite(E2, currentSpeed);
      delay(20);
    }
    analogWrite(E1, finalSpeed);
    analogWrite(E2, finalSpeed);
  }
}

/*
   Reads distance multiple times and returns
   the lowest
*/
float getLowestDist(int count) {
  float current;
  float lowest = getDistance();
  for (int i = 0; i < count; i++) {
    current = getDistance();
    if ( current < lowest ) {
      lowest = current;
    }
  }
  return lowest;
}

void turnServo(int dir) {
  if (dir > 90 || dir < -90) {
    return;
  }

  //if turning left:
  while (servoPos <= dir + 90) {
    myservo.write(servoPos);
    servoPos++;
    delay(2);
  }
  //If turning right
  while (servoPos >= dir + 90) {
    myservo.write(servoPos);
    servoPos--;
    delay(2);
  }
}

void slowDown (float minDist) {
  int chenSpeed = currentSpeed;
  int initialSpeed = currentSpeed;
  int initialDistance = getLowestDist(5);
  int currentDistance = initialDistance;

  while (currentSpeed > 100) {
    chenSpeed = initialSpeed / (initialDistance - minDist) * (currentDistance - minDist);
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, chenSpeed);   //PWM Speed Control
    analogWrite(E2, chenSpeed);   //PWM Speed Control
    delay(30);
    currentDistance = getLowestDist(10);
  }
  analogWrite(E1, 0);   //PWM Speed Control
  analogWrite(E2, 0);   //PWM Speed Control
  currentSpeed = 0;
}

/*
   Slow whell by percentage between 0 and 1, for certain length of time.
*/
void slowLeftWheel(float percent, int time) {
  if (percent > 1 || percent < 1) {
    return;
  }
  analogWrite(E1, (int)(percent * (float)currentSpeed));
  delay(time);
  analogWrite(E1, currentSpeed);
}

void slowRightWheel(float percent, int time) {
  if (percent > 1 || percent < 1) {
    return;
  }
  analogWrite(E2, (int)(percent * (float)currentSpeed));
  delay(time);
  analogWrite(E2, currentSpeed);
}


