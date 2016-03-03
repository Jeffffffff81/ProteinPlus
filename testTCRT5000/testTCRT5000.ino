
/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */

//variables for storing TCRT5000 sensor values
int middleSensor; 
int rightSensor; 
int leftSensor; 

int E1 = 5; 
int E2 = 6;
int M1 = 4; 
int M2 = 7; 

int LsensorValue; 
int RsensorValue; 
int MsensorValue; 
 
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  pinMode(M1, OUTPUT); 
  pinMode(M2, OUTPUT); 

  digitalWrite(M1, HIGH); 
  digitalWrite(M2, HIGH); 
}
 
// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  middleSensor = analogRead(A0);
  delay(50);
  rightSensor = analogRead(A1);
  delay(50); 
  leftSensor = analogRead(A4); 
  delay(50);
  
  // print out the value you read:
  Serial.print("middle: "); 
  Serial.print(middleSensor);

  Serial.print(" right: "); 
  Serial.print(rightSensor); 

  Serial.print(" left: ");
  Serial.println(leftSensor); 
  delay(50);        // delay in between reads for stability used to be 1 


  while(LsensorValue > 60) {
    LsensorValue = analogRead(A4); 
    analogWrite(E1, 50); 
    analogWrite(E2, 255); 
  }

  while(RsensorValue > 60) {
    RsensorValue = analogRead(A0);
    analogWrite(E1, 255); 
    analogWrite(E2, 50); 
  }
}
