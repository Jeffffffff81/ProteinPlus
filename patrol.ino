#include <IRremote.h>
//#include <Servo.h>


/*  Infrared Remote Controller & IR Receiver demo program
  copy right John Yu
  connect VS1838B to  D10 see http://osoyoo.com/?p=144
*/
const int receiver_pin = 3;        // set 3 as receiver input signal pin
IRrecv irrecv(receiver_pin);
decode_results signals;

// PINS:
const int L_MOTOR_SPEED_PIN = 5;      // E1
const int L_MOTOR_DIRECTION_PIN = 4;  // M1
const int R_MOTOR_DIRECTION_PIN = 7;  // M2
const int R_MOTOR_SPEED_PIN = 6;      // E2
const int LED_PIN = 3;                // blue and red led

//const int E1 = 5;
//const int M1 = 4;
//const int E2 = 6;
//const int M2 = 7;

//Servo myservo;
//const int ServoPin = 9;
//
//const int trigPin = 13;
//const int echoPin = 12;
//
//const int tempPin = A5;
//
//const int switchPin = 2;



// GOLBAL VARIABLES
const int MAX_DISTANCE = 1000;
const float RIGHT_WHEEL_CORRECT = 1;

const int L_MOTOR = 1;
const int R_MOTOR = 2;
const int FORWARD = HIGH;
const int BACKWARD = LOW;
const int LEFT = 1;
const int RIGHT = 2;

const int TURN_SPEED = 215;
const int MIN_SPEED = 60;
const int MAX_SPEED = 255;
const int ROTATION_FACTOR = 5; // at TURN_SPEED, each degree rotation needs ROTATION_FACTOR milliseconds




void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn(); // enable input from IR receiver


  pinMode(L_MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(R_MOTOR_DIRECTION_PIN, OUTPUT);



}

void loop() {
  //  if (irrecv.decode(&signals)) {
  //    Serial.println(signals.value, HEX);
  //    irrecv.resume(); // get the next signal
  //  }

  patrol (120, 1);


}


void patrol(int patrol_speed, int loop_num) {
  setMoveSpeed (patrol_speed, FORWARD);
  int i;
  for (i = 0; i < loop_num; i++) {
    moveForward (patrol_speed, 5000);
    turn(90, LEFT);
    moveForward (patrol_speed, 5000);
    turn(90, LEFT);
    moveForward (patrol_speed, 5000);
    turn(90, RIGHT);
    moveForward (patrol_speed, 5000);
    turn(90, RIGHT);
  }
}



void moveForward (int volt_speed, int duration) {
  unsigned long start_time, end_time;
  int i;

  setMoveSpeed(volt_speed, FORWARD);
  delay(duration);
  setMoveSpeed(0, FORWARD);
  delay(2000);


  //  start_time = millis();
  //
  //  for (i=0; i<2*duration; i++) {
  //    end_time = millis();
  //    if ((unsigned long)end_time-start_time >= duration){
  //      setMoveSpeed(0, FORWARD);
  //      delay(2000);
  //    }
  //  }

}



void turn(int rotation, int direction) {
  // stop the robot before turning
  setMotorSpeed(L_MOTOR, 0, FORWARD);
  setMotorSpeed(R_MOTOR, 0, BACKWARD);
  delay(20);

  if (direction == LEFT) {
    setMotorSpeed(L_MOTOR, MIN_SPEED, FORWARD);
    setMotorSpeed(R_MOTOR, TURN_SPEED, FORWARD);
  }
  if (direction == RIGHT) {
    setMotorSpeed(L_MOTOR, TURN_SPEED, FORWARD);
    setMotorSpeed(R_MOTOR, MIN_SPEED, FORWARD);
  }
  delay(rotation * (ROTATION_FACTOR));
}



void stop() {
  setMotorSpeed(L_MOTOR, 0, FORWARD);
  setMotorSpeed(R_MOTOR, 0, BACKWARD);
}

/*
 * Set the speed of the robot in movement
 * speed: the spped in volts (0-255) in which the robot moves at
 * direction: the direction in which the robot will move (forward or backward)
 */

void setMoveSpeed (int speed, int direction) {
  setMotorSpeed(L_MOTOR, speed, direction);
  setMotorSpeed(R_MOTOR, (int) ((float) (speed * RIGHT_WHEEL_CORRECT)), direction);
}

/*
 * Set the speed of a given motor in a given direction
 * motor: the motor that will be controlled (left motor or right motor)
 * speed: the speed in volts (0-255) that will be sent to motor
 * direction: the direction in which the motor will spin (forward or backward)
 */
void setMotorSpeed (int motor, int speed, int direction) {
  int dir_pin, speed_pin;
  // decide which motor (left or right) to control
  if (motor == L_MOTOR) {
    dir_pin = L_MOTOR_DIRECTION_PIN;
    speed_pin = L_MOTOR_SPEED_PIN;
  }
  else {
    dir_pin = R_MOTOR_DIRECTION_PIN;
    speed_pin = R_MOTOR_SPEED_PIN;
  }
  digitalWrite(dir_pin, direction);       // set the direction of motor
  analogWrite(speed_pin, speed);          // set the speed of motor
}





//void move(results.value) {
//  if () {                       // right arrow
//    analogWrite(E1, 0);
//    analogWrite(E2, 255);
//  }
//
//  else if () {                  // left arrow
//    while (results.value == 0xFFFFFF) {
//      analogWrite(E1, 255);
//      analogWrite(E2, 0);
//    }
//  }
//
//  analogWrite(E1, 255);   //PWM Speed Control
//  analogWrite(E2, 255);   //PWM Speed Control
//}


//int RECEIVE_PIN = 3;
//IRrecv irreceiver(RECEIVE_PIN);
//decode_results results;
//
//void setup()
//{
//  Serial.begin(9600);
//  irreceiver.enableIRIn(); // Start the receiver
//}
//
//void loop() {
//  if (irreceiver.decode(&results)) {
//    Serial.println(results.value, HEX);
//  }
//}






