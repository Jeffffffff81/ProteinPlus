#include <IRremote.h>

/*  Infrared Remote Controller & IR Receiver demo program
  copy right John Yu
  connect VS1838B to  D10 see http://osoyoo.com/?p=144
*/
int input_pin = 3; //set 3 as input signal pin
IRrecv irrecv(input_pin);
decode_results signals;
void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // enable input from IR receiver
}
void loop() {
if (irrecv.decode(&signals)) {
    Serial.println(signals.value, HEX);
    irrecv.resume(); // get the next signal
  }
}




















 
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





//void move(results.value){
//  if(){ //right arrow
//    analogWrite(E1, 0);
//    analogWrite(E2,255);
//  } else if(){ // left arrow
//    while(results.value==0xFFFFFF){
//      analogWrite(E1, 255);
//      analogWrite(E2, 0); }
//  } m
//  
//  analogWrite(E1, 255);   //PWM Speed Control
//  analogWrite(E2, 255);   //PWM Speed Control
//}
