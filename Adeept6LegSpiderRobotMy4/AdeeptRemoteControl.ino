/***********************************************************
File name:  AdeeptRemoteControl.ino
Description:  
The joystick U1 moves to the left, the robot rotates to the left; 
the joystick U1 moves to the right, the robot rotates to the right; 
the joystick U1 moves forward, the robot moves forward; 
the joystick U1 moves backward, the robot moves backwards mobile.
The rocker U2 moves to th e left and the robot pans to the left; 
the rocker U2 translates to the right and the robot pans to the right.
Button A: The robot enters the remote motion mode, LED2 is on, LED3 is off;
Button B: The robot enters the automatic avoidance obstacle mode, LED2 goes out, LED3 lights up;
Button C: The robot enters search for light source mode, LED2 is on, LED3 is on;
Button D: Control the buzzer to sound.

Website: www.adeept.com
E-mail: support@adeept.com
Author: Tom
Date: 2018/06/12 
***********************************************************/
#include <SPI.h>
#include "RF24.h"
RF24 radio(9, 10);            // define the object to control NRF24L01
byte addresses[5] = "5";  // define communication address which should correspond to remote control
int data[10];                  // define array used to save the communication data
int mode = 0;
const int pot6Pin = 7;        // define R6
const int pot5Pin = 6;        // define R1
const int led1Pin = 6;        // define pin for LED1 which is close to NRF24L01 and used to indicate the state of NRF24L01 
const int led2Pin = 7;        // define pin for LED2 which is the mode is displayed in the Six foot robots remote control mode  
const int led3Pin = 8;        // define pin for LED3 which is the mode is displayed in the Six foot robots auto mode 
const int APin = 2;           // define pin for D2
const int BPin = 3;           // define pin for D3
const int CPin = 4;           // define pin for D4
const int DPin = 5;           // define pin for D5

const int u1XPin = 0;      // define pin for direction X of joystick U1
const int u1YPin = 1;      // define pin for direction Y of joystick U1
const int u2XPin = 2;      // define pin for direction X of joystick U2
const int u2YPin = 3;      // define pin for direction Y of joystick U2

void setup() {
  
  radio.begin();                      // initialize RF24
  radio.setRetries(0, 15);            // set retries times
  radio.setPALevel(RF24_PA_LOW);      // set power
  radio.openWritingPipe(addresses);   // open delivery channel
  radio.stopListening();              // stop monitoring
  pinMode(led1Pin, OUTPUT);           // set led1Pin to output mode
  pinMode(led2Pin, OUTPUT);           // set led2Pin to output mode
  pinMode(led3Pin, OUTPUT);           // set led3Pin to output mode
  pinMode(APin, INPUT_PULLUP);        // set APin to output mode
  pinMode(BPin, INPUT_PULLUP);        // set BPin to output mode
  pinMode(CPin, INPUT_PULLUP);        // set CPin to output mode
  pinMode(DPin, INPUT_PULLUP);        // set DPin to output mode  
}

void loop() {
  // put the values of rocker, switch and potentiometer into the array
  data[0] = analogRead(u1XPin);
  data[1] = analogRead(u1YPin);
  if(digitalRead(APin)==LOW){  //Switch the working mode
    delay(10);
    if(digitalRead(APin)==LOW){data[2] = 0;}
  }
  if( digitalRead(BPin)==LOW){ //Switch the working mode
    delay(10);
    if(digitalRead(BPin)==LOW){data[2] = 1;}
  }
  if(digitalRead(CPin)==LOW){//Switch the working mode
    delay(10);
    if(digitalRead(CPin)==LOW){data[2] = 2;}
  }
  data[5] = digitalRead(DPin);
  data[6] = analogRead(pot5Pin);
  data[7] = analogRead(u2YPin);
  data[8] = analogRead(u2XPin);
  data[9] = 1234;
  // send array data. If the sending succeeds, open signal LED
  if (radio.write( data, sizeof(data) ))
  {digitalWrite(led1Pin,HIGH);
  // delay for a period of time, then turn off the signal LED for next sending
  }else{digitalWrite(led1Pin,LOW);}
  delay(50);
  if(data[2]==0){//LED display status
    digitalWrite(led2Pin,HIGH);
    digitalWrite(led3Pin,LOW);
  }
  if(data[2]==1){//LED display status
      digitalWrite(led2Pin,LOW);
      digitalWrite(led3Pin,HIGH);
  }
  if(data[2]==2){//LED display status
      digitalWrite(led2Pin,HIGH);
      digitalWrite(led3Pin,HIGH);
  }
}
