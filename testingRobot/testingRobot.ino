#include <Servo.h>

//Arduino PWM Speed Control：
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

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
int maxRange = 500;
int minRange = 2;



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
}

void loop()
{
  digitalWrite(10, HIGH); 
  sweep(1000, 15);
  delay(2000); 
  moveWheels();
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
  delay(wait_time/2);                            //pauses for wait_time ms at position 180
  dist1 = getDistance(); 

  for (pos = 180; pos >= 0; pos -= 1) {         // goes from 180 degrees to 0 degrees
    myservo.write(pos);                          // tell servo to go to position in variable 'pos'
    delay(turn_speed);                           // waits turn_speed ms for the servo to reach the position
  }
  delay(wait_time/2);                           //pauses for wait_time ms at position 0
  dist2 = getDistance(); 
  delay(wait_time/2);
  
  for (pos = 0; pos <= 90; pos += 1) {         // goes from 0 degrees to 90 degrees (back to the start position)
    // in steps of 1 degree
    myservo.write(pos);                       // tell servo to go to position in variable 'pos'
    delay(turn_speed);                       // waits turn_speed ms for the servo to reach the position
  }
}

/*
   Function - moveWheels - so far just for testing purposes
*/
void moveWheels(void) {
  int value;
  for (value = 0 ; value <= 255; value += 5)
  {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, value);   //PWM Speed Control
    analogWrite(E2, value);   //PWM Speed Control
    delay(30);
  }
   for (value = 255 ; value >= 0; value -= 5)
  {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, value);   //PWM Speed Control
    analogWrite(E2, value);   //PWM Speed Control
    delay(30);
  }
}

/*
 * Function: getDistance - returns the distance measured  by the HC-SR05 and prints it onto the serial monitor
 */
float getDistance(void){
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
  distance = duration / (2 * soundSpeed); // improve measurement precision

  if (distance <= maxRange && distance >= minRange) {
    // print the result onto serial monitor
      Serial.println(distance);
      delay(500);        // delay for 500 ms to allow the echo fully fades before next reading
      return distance; 
  }
  else {
    Serial.println("ERROR: Object not detected.");
    delay(500);           // delay for 500 ms to allow the echo fully fades before next reading
    return NULL; 
  }
  
  
}

