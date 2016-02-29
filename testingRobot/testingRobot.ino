#include <Servo.h>
#include <NewPing.h>

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

//CONSTANTS:
int MAX_DISTANCE = 1000;

//GLOBAL VARIABLES:
float distance;
int currentSpeed = 0;
int servoPos = 0;
NewPing sonar(trigPin, echoPin, MAX_DISTANCE);

void setup()
{
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  myservo.attach(ServoPin);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  distance = getDistance(); //bug fix/hack for distance = .1 initially
}

void loop() {
  while (true) {
    Serial.println(getDistance());
    delay(100);
  }
  
  for (int i = 0; i < 4; i ++) {
    changeSpeed(0, 4);
    distance = getLowestDist(10);
    if (distance < 30) {
      avoidWall(30);
    }
    delay(250);
  }

  Serial.print(getDistance());

  turnServo(70);
  delay(300);
  distance = getLowestDist(5);
  delay(300);
  Serial.print("  left ");
  Serial.println(distance);
  turnServo(0);
  if (distance < 30) {
    Serial.println("turn right?");

  }
}

//**************HIGHLEVELFUNCTIONS******************//
/*
   Checks to see if its going to crash into a wall and take corrective action
*/
void avoidWall(int distThreshold) {
  //slowDown(0); //problem here?

  changeSpeed(0, 1);
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
    turn(true, 1000, 235);
    turn(true, 1000, 235);
  }
  else if (leftDist > rightDist) {
    turn(true, 1000, 235);
  }
  else {
    turn(false, 1000, 235);
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
  

  //return sonar.ping_cm();

  
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
      delay(10);
    }
    analogWrite(E1, finalSpeed);
    analogWrite(E2, finalSpeed);
  } else if (finalSpeed < currentSpeed) {
    for (currentSpeed;  currentSpeed > finalSpeed; currentSpeed -= acceleration) {

      analogWrite(E1, currentSpeed);
      analogWrite(E2, currentSpeed);
      delay(10);
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


