/***********************************************************
File name:  Adeept6LegSpiderRobot.ino
Description: 
Function 1: Remote motion mode, forward, backward, left turn, 
            right turn, pan left and pan right. This feature 
            supports Adeept Remote Control Shield Board remote 
            control, panning left and right panning function 
            does not support Android APP remote control;
Function 2: Find the light source mode. The robot detects the 
            light source in the left and right direction and 
            follows the light source to move left and right. 
            This feature supports Adeept Remote Control Shield 
            Board remote control and Android APP remote control;
Function 3: Automatically avoid obstacle mode. This feature 
            supports Adeept Remote Control Shield Board remote 
            control and Android APP remote control; 
Website: www.adeept.com
E-mail: support@adeept.com
Author: Tom
Date: 2018/06/12
***********************************************************/
#include <Adeept_Distance.h>
#include <Adeept_PWMPCA9685.h>
#include <SPI.h>
#include <RF24.h>
Adeept_PWMPCA9685 pwm0 = Adeept_PWMPCA9685(0x40);                //1+A5 A4 A3 A2 A1 A0+RW, RW is Read and Write
Adeept_PWMPCA9685 pwm1 = Adeept_PWMPCA9685(0x41);                //1+A5 A4 A3 A2 A1 A0+RW, RW is Read and Write
Adeept_Distance Dist;
//WayOfCommunication=1:use Bluetooth communication.    
//WayOfCommunication=0:remote control communication
int WayOfCommunication=0;

int distance;
int movinSpeed=200;//Control the speed of movement by controlling the delay time
RF24 radio(9, 10);                // define the object to control NRF24L01
byte addresses[5] = "5";      // define communication address which should correspond to remote control
int data[10]={512, 512, 1, 0, 1, 1, 512, 512, 512,1};  // define array used to save the communication data
int mode = 0,UltrasoundMode=0;

//The coordinate calculation parameters(c,d,e,f) of one leg of the robot, 
//we provide a fixed value of the mechanical leg, can not be changed
float c = 34.5;
float d = 12.5;
float e = 38;
float f = 57;

int a1=0,b1=5,g1=0;//1 leg offset adjustment;(a1:PWM26 offset);(b1:PWM25 offset);(g1:PWM24 offset);
int a2=0,b2=-7,g2=10;//2 leg offset adjustment;(a2:PWM5 offset);(b2:PWM6 offset);(g2:PWM7 offset);
int a3=0,b3=2,g3=-4;//3 leg offset adjustment;(a3:PWM22 offset);(b3:PWM21 offset);(g3:PWM20 offset);
int a4=11,b4=2,g4=-5;//4 leg offset adjustment;(a4:PWM9 offset);(b4:PWM10 offset);(g4:PWM11 offset);
int a5=0,b5=0,g5=-5;//5 leg offset adjustment;(a5:PWM18 offset);(b5:PWM17 offset);(g5:PWM16 offset);
int a6=0,b6=-3,g6=4;//6 leg offset adjustment;(a6:PWM13 offset);(b6:PWM14 offset);(g6:PWM15 offset);

float alpha=90, beta=90, gamma=90,x=0,y=50.5,z=-22.5;//Default value of the default six legs of the robot
float alpha1=90, beta1=90, gamma1=90;//The initial value of the robot's first leg standing
float alpha2=90, beta2=90, gamma2=90;//The initial value of the robot's second leg standing
float alpha3=90, beta3=90, gamma3=90;//The initial value of the robot's third leg standing
float alpha4=90, beta4=90, gamma4=90;//The initial value of the robot's fourth leg standing
float alpha5=90, beta5=90, gamma5=90;//The initial value of the fifth leg of the robot
float alpha6=90, beta6=90, gamma6=90;//The initial value of the robot's sixth leg standing

int buzzerPin = 7;  // define pin for buzzer
int analogPinRight = 0;//Right photoresist module input interface
int analogPinLeft = 1;//Left photoresistor module input interface
int valRight = 0;           // variable to store the value read
int valLeft = 0;           // variable to store the value read
int cnt=0;

int legpos;
void setup(){
  radio.begin();                      // initialize RF24
  radio.setRetries(0, 15);            // set retries times
  radio.setPALevel(RF24_PA_LOW);      // set power
  radio.openReadingPipe(1, addresses);// open delivery channel
  radio.startListening();             // start monitoring
  delay(100);
  pwm0.begin();
  pwm0.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm1.begin();
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  Serial.begin(9600);
  Dist.begin(2,3);//begin(Echo,Trig)
  delay(200); Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, -22.5); Leg6Position(0, 50.5, -22.5); 
              Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, -22.5); Leg5Position(0, 50.5, -22.5); MoveLegPosition();
  pwm0.setPWM(3, 0, angle(90));
}
void loop()
{   
   if(WayOfCommunication==1){ //WayOfCommunication=1:use Bluetooth communication. 
      BluetoothControl();
      switch(mode){
        case 1: UltrasoundDetectionMode(); break;
        case 2: FollowLightMode(); break;
        default: 
             pwm0.setPWM(2, 0, 0);
             pwm0.setPWM(1, 0, 4095);
             pwm0.setPWM(0, 0, 4095);
             if(data[1]==900&&data[0]==512){
                ForwardStart();
                legpos=0;
                while(data[1]==900){
                  BluetoothControl();
                  if(legpos==0){
                    Forward();
                    legpos=1;
                  }
                  else{
                    Forward2();
                    legpos=0;
                  }
                }
                data[1]=512;
                if(legpos==0){
                    ForwardStop();
                  }
                  else{
                    ForwardStop2();
                  } 
             }
             if(data[1]==200&&data[0]==512){
                BackwardStart();
                legpos=0;
                while(data[1]==200){
                  BluetoothControl();
                  if(legpos==0){
                    Backward();
                    legpos=1;
                  }
                  else{
                    Backward2();
                    legpos=0;
                  }
                }
                data[1]=512;
                if(legpos==0){
                    BackwardStop();
                  }
                  else{
                    BackwardStop2();
                  } 
              }
             if(data[1]==512&&data[0]==900){TurnLeft();}
             if(data[1]==512&&data[0]==200){TurnRight();}
             break;
      }
   }
   if(WayOfCommunication==0){ //WayOfCommunication=0:use remote control communication  
    receiveData();
     switch(mode){
     case 1: UltrasoundDetectionMode(); break;
     case 2: FollowLightMode(); break;
     default: 
             pwm0.setPWM(2, 0, 0);
             pwm0.setPWM(1, 0, 4095);
             pwm0.setPWM(0, 0, 4095);
             if(data[1]>=700&&data[0]>=400&&data[0]<=700){
                BackwardStart();
                legpos=0;
                while(data[1]>=700){
                  receiveData();
                  if(legpos==0){
                    Backward();
                    legpos=1;
                  }
                  else{
                    Backward2();
                    legpos=0;
                  }
                }
                data[1]=512;
                if(legpos==0){
                    BackwardStop();
                  }
                  else{
                    BackwardStop2();
                  } 
              }
             if(data[1]<=400&&data[0]>=400&&data[0]<=700){
                ForwardStart();
                legpos=0;
                while(data[1]<=400){
                  receiveData();
                  if(legpos==0){
                    Forward();
                    legpos=1;
                  }
                  else{
                    Forward2();
                    legpos=0;
                  }
                }
                data[1]=512;
                if(legpos==0){
                    ForwardStop();
                  }
                  else{
                    ForwardStop2();
                  } 
              }
             if(data[0]>=700&&data[1]>=400&&data[1]<=700){
              TurnLeft();
             }
             if(data[0]<=400&&data[1]>=400&&data[1]<=700){
              TurnRight();
             }
             if(data[8]>=700){
                StartHorizontalMovement();
                while(data[8]>=700){receiveData();HorizontallyRight();}
                data[8]=512;
                StopHorizontalMovement();
              }
             if(data[8]<=400){
                StartHorizontalMovement();
                while(data[8]<=400){receiveData();HorizontallyLeft();}
                data[8]=512;
                StopHorizontalMovement();
              }
      break;
     }
  }
}
void BluetoothControl(){
  //Bluetooth control
  if(Serial.available() > 0){//Receive serial(Bluetooth) data   
       switch(Serial.read()){//Save the serial(Bluetooth) data received 
          case 'a': data[1]=900;data[0]=512;break;//go ahead
          case 'b': data[1]=512;data[0]=200;break;//turn right
          case 'c': data[1]=512;data[0]=900;break;//turn left
          case 'd': data[1]=200;data[0]=512;break;//backwards
          case 'e': mode = 0; data[8]=512;data[1]=512;data[0]=512;break;
          case 'f': mode = 1; break;
          case 'g': mode = 2; break;
          case 'h': tone(buzzerPin, 2000);break;
          case 'i': noTone(buzzerPin);break;
       }
    } 
}
void UltrasoundDetectionMode(){//mode=1
    float   distanceLeft=0, distanceRight=0; 
    pwm0.setPWM(3, 0, angle(90));
    pwm0.setPWM(2, 0, 4095);
    pwm0.setPWM(1, 0, 4095);
    pwm0.setPWM(0, 0, 0);
    ForwardStart();
    legpos=0;
    while(mode==1){
      distance = Dist.getDistanceCentimeter();
      if(distance<=20){
      if(legpos==0){
        ForwardStop();
      }
      else{
        ForwardStop2();
      }
      pwm0.setPWM(3, 0, angle(180));
      delay(1000);
      do{
          distanceLeft = Dist.getDistanceCentimeter();}while(distanceLeft<0);
      pwm0.setPWM(3, 0, angle(0));
      delay(1000);
      do{
          distanceRight = Dist.getDistanceCentimeter();}while(distanceRight<0);
      pwm0.setPWM(3, 0, angle(90));
      delay(100);
      if(distanceLeft>30&&distanceLeft>distanceRight){
          TurnLeft(); TurnLeft(); TurnLeft();  
        }else if(distanceRight>=30&&distanceRight>=distanceLeft){
          TurnRight(); TurnRight(); TurnRight();
        }else
        {TurnRight(); TurnRight(); TurnRight();TurnRight(); TurnRight(); TurnRight();}
      ForwardStart();
      }
      if(legpos==0){
        Forward();
        legpos=1;
      }
      else{
        Forward2();
        legpos=0;
      }
      if(WayOfCommunication==0){receiveData();}else{BluetoothControl();}
    }
    if(legpos==0){
      ForwardStop();
    }
    else{
      ForwardStop2();
    } 
}
void FollowLightMode(){
    pwm0.setPWM(1, 0, 0);
    pwm0.setPWM(2, 0, 4095);
    pwm0.setPWM(0, 0, 4095);
    if((analogRead(analogPinRight)-analogRead(analogPinLeft))>=50){
        StartHorizontalMovement();
        while((analogRead(analogPinRight)-analogRead(analogPinLeft))>=50&&mode==2){HorizontallyRight();if(WayOfCommunication==0){receiveData();}else{BluetoothControl();}}
        StopHorizontalMovement();
    }
    if((analogRead(analogPinLeft)-analogRead(analogPinRight))>=50){
        StartHorizontalMovement();
        while((analogRead(analogPinLeft)-analogRead(analogPinRight))>=50&&mode==2){HorizontallyLeft();if(WayOfCommunication==0){receiveData();}else{BluetoothControl();}}    
        StopHorizontalMovement();
   } 
}
void StartHorizontalMovement(){   
    delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, -22.5); Leg6Position(0, 50.5, -22.5);Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, -22.5); Leg5Position(0, 50.5, -22.5); MoveLegPosition();
    delay(movinSpeed); Leg1Position(32.46, 38.69, 0); Leg4Position(0, 50.5, 0); Leg5Position(-32.46, 38.69, 0); MoveLegPosition();
    delay(movinSpeed); Leg1Position(27.5, 30, 0); Leg4Position(0, 60.5, 0); Leg5Position(-27.5, 30, 0);MoveLegPosition(); 
    delay(movinSpeed); Leg1Position(27.5, 30, -22.5); Leg4Position(0, 60.5, -22.5); Leg5Position(-27.5, 30, -22.5);MoveLegPosition(); 
    delay(movinSpeed); Leg2Position(32.46, 38.69, 0); Leg3Position(0, 50.5, 0); Leg6Position(-32.46, 38.69, 0); MoveLegPosition();
}
void StopHorizontalMovement(){
    delay(movinSpeed); Leg2Position(0, 50.5, 0); Leg3Position(0, 50.5, 0); Leg6Position(0, 50.5, 0); MoveLegPosition();
    delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, -22.5); Leg6Position(0, 50.5, -22.5); MoveLegPosition();
    delay(movinSpeed); Leg1Position(27.5, 30, 0); Leg4Position(0, 60.5, 0); Leg5Position(-27.5, 30, 0); MoveLegPosition();
    delay(movinSpeed); Leg1Position(0, 50.5, 0); Leg4Position(0, 50.5, 0); Leg5Position(0, 50.5, 0); MoveLegPosition();
    delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, -22.5); Leg5Position(0, 50.5, -22.5); MoveLegPosition();
}
void HorizontallyLeft(){//Move horizontally to the left
    delay(movinSpeed); Leg2Position(27.5, 30, 0); Leg3Position(0, 60.5, 0); Leg6Position(-27.5, 30, 0); MoveLegPosition();
    delay(movinSpeed); Leg1Position(37.5, 47.3, -22.5); Leg4Position(0, 40.5, -22.5); Leg5Position(-37.5, 47.3, -22.5); Leg2Position(37.5, 47.3, 0); Leg3Position(0, 40.5, 0); Leg6Position(-37.5, 47.3, 0); MoveLegPosition();
    delay(movinSpeed); Leg2Position(37.5, 47.3, -22.5); Leg3Position(0, 40.5, -22.5); Leg6Position(-37.5, 47.3, -22.5); MoveLegPosition();
    delay(movinSpeed); Leg1Position(37.5, 47.3, 0); Leg4Position(0, 40.5, 0); Leg5Position(-37.5, 47.3, 0); MoveLegPosition(); 
    delay(movinSpeed); Leg1Position(27.5, 30, 0); Leg4Position(0, 60.5, 0); Leg5Position(-27.5, 30, 0); Leg2Position(27.5, 30, -22.5); Leg3Position(0, 60.5, -22.5); Leg6Position(-27.5, 30, -22.5); MoveLegPosition();
    delay(movinSpeed); Leg1Position(27.5, 30, -22.5); Leg4Position(0, 60.5, -22.5); Leg5Position(-27.5, 30, -22.5); MoveLegPosition();  
}
void HorizontallyRight(){//Move horizontally to the right
  delay(movinSpeed); Leg2Position(27.5, 30, 0); Leg3Position(0, 60.5, 0); Leg6Position(-27.5, 30, 0); MoveLegPosition();
  delay(movinSpeed); Leg2Position(27.5, 30, -22.5); Leg3Position(0, 60.5, -22.5); Leg6Position(-27.5, 30, -22.5);  MoveLegPosition();
  delay(movinSpeed); Leg1Position(27.5, 30, 0); Leg4Position(0, 60.5, 0); Leg5Position(-27.5, 30, 0);MoveLegPosition(); 
  delay(movinSpeed); Leg1Position(37.5, 47.3, 0); Leg4Position(0, 40.5, 0); Leg5Position(-37.5, 47.3, 0); Leg2Position(37.5, 47.3, -22.5); Leg3Position(0, 40.5, -22.5); Leg6Position(-37.5, 47.3, -22.5); MoveLegPosition();
  delay(movinSpeed); Leg1Position(37.5, 47.3, -22.5); Leg4Position(0, 40.5, -22.5); Leg5Position(-37.5, 47.3, -22.5); MoveLegPosition(); 
  delay(movinSpeed); Leg2Position(37.5, 47.3, 0); Leg3Position(0, 40.5, 0); Leg6Position(-37.5, 47.3, 0); Leg1Position(27.5, 30, -22.5); Leg4Position(0, 60.5, -22.5); Leg5Position(-27.5, 30, -22.5); MoveLegPosition();
}

void TurnRight(){
   delay(movinSpeed); Leg1Position(0, 50.5, 0); Leg4Position(0, 50.5, 0);  MoveLegPosition();
   delay(movinSpeed); Leg1Position(32.46, 38.69, 0); Leg4Position(-17.27, 47.45, 0);  MoveLegPosition();
   delay(movinSpeed); Leg1Position(32.46, 38.69, -22.5); Leg4Position(-17.27, 47.45, -22.5);  MoveLegPosition();
   delay(movinSpeed); Leg2Position(0, 50.5, 0); Leg3Position(0, 50.5, 0); Leg6Position(0, 50.5, 0); MoveLegPosition(); 
   delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(17.27, 47.45, -22.5); Leg5Position(-32.46, 38.69, -22.5); Leg3Position(17.27, 47.45, 0); Leg6Position(-32.46, 38.69, 0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(17.27, 47.45, -22.5); Leg6Position(-32.46, 38.69, -22.5); MoveLegPosition();
   delay(movinSpeed); Leg1Position(0, 50.5, 0); Leg4Position(17.27, 47.45, 0); Leg5Position(-32.46, 38.69, 0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(32.46, 38.69, -22.5); Leg3Position(-17.27, 47.45, -22.5); Leg6Position(0, 50.5, -22.5); Leg1Position(0, 50.5, 0); Leg4Position(0, 50.5, 0); Leg5Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, -22.5); Leg5Position(0, 50.5, -22.5); MoveLegPosition();
   delay(movinSpeed); Leg2Position(32.46, 38.69, 0); Leg3Position(-17.27, 47.45, 0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(0, 50.5, 0); Leg3Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, -22.5); MoveLegPosition();
}


void TurnLeft(){
   delay(movinSpeed); Leg2Position(0, 50.5, 0); Leg3Position(0, 50.5, 0);  MoveLegPosition();
   delay(movinSpeed); Leg2Position(32.46, 38.69, 0); Leg3Position(-17.27, 47.45, 0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(32.46, 38.69, -22.5); Leg3Position(-17.27, 47.45, -22.5);  MoveLegPosition();
   delay(movinSpeed); Leg1Position(0, 50.5, 0); Leg4Position(0, 50.5, 0); Leg5Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(17.27, 47.45, -22.5); Leg6Position(-32.46, 38.69, -22.5); Leg4Position(17.27, 47.45, 0); Leg5Position(-32.46, 38.69, 0); MoveLegPosition();
   delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(17.27, 47.45, -22.5); Leg5Position(-32.46, 38.69, -22.5); MoveLegPosition(); 
   delay(movinSpeed); Leg2Position(0, 50.5, 0); Leg3Position(17.27, 47.45, 0); Leg6Position(-32.46, 38.69, 0); MoveLegPosition(); 
   delay(movinSpeed); Leg1Position(32.46, 38.69, -22.5); Leg4Position(-17.27, 47.45, -22.5); Leg5Position(0, 50.5, -22.5); Leg2Position(0, 50.5, 0); Leg3Position(0, 50.5, 0); Leg6Position(0, 50.5, 0); MoveLegPosition(); 
   delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, -22.5); Leg6Position(0, 50.5, -22.5); MoveLegPosition();
   delay(movinSpeed); Leg1Position(32.46, 38.69, 0); Leg4Position(-17.27, 47.45, 0); MoveLegPosition();
   delay(movinSpeed); Leg1Position(0, 50.5, 0); Leg4Position(0, 50.5, 0); MoveLegPosition();  
   delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, -22.5); MoveLegPosition();              
}




void BackwardStart(){
    delay(movinSpeed);Leg3Position(0, 50.5, 0);Leg4Position(0, 50.5, 0); MoveLegPosition();
    delay(movinSpeed);Leg3Position(17.27, 47.45, 0);Leg4Position(-17.27, 47.45, 0);MoveLegPosition();
    delay(movinSpeed);Leg3Position(17.27, 47.45, -22.5);Leg4Position(-17.27, 47.45, -22.5);MoveLegPosition();
    delay(movinSpeed);Leg1Position(0, 50.5, 0);Leg6Position(0, 50.5, 0);MoveLegPosition();
    delay(movinSpeed);Leg6Position(-32.46, 38.69, 0);MoveLegPosition();
}
void Backward(){   
   delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(-17.27, 47.45, -22.5); Leg6Position(-32.46, 38.69, -22.5); MoveLegPosition();
   delay(movinSpeed); Leg1Position(32.46, 38.69, 0); Leg4Position(17.27, 47.45, 0); Leg5Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(32.46, 38.69, -22.5); Leg3Position(17.27, 47.45, -22.5); Leg6Position(0, 50.5, -22.5); 
                      Leg1Position(0, 50.5, 0); Leg4Position(-17.27, 47.45, 0); Leg5Position(-32.46, 38.69, 0); MoveLegPosition();
}
void Backward2(){
   delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(-17.27, 47.45, -22.5); Leg5Position(-32.46, 38.69, -22.5); MoveLegPosition();
   delay(movinSpeed); Leg2Position(32.46, 38.69, 0); Leg3Position(17.27, 47.45,0); Leg6Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(0, 50.5, 0); Leg3Position(-17.27, 47.45, 0); Leg6Position(-32.46, 38.69, 0); 
                      Leg1Position(32.46, 38.69, -22.5); Leg4Position(17.27, 47.45, -22.5); Leg5Position(0, 50.5, -22.5); MoveLegPosition();
}
void BackwardStop(){
   delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, 0); Leg6Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg3Position(0, 50.5, -22.5); Leg6Position(0, 50.5, -22.5); MoveLegPosition();
   delay(movinSpeed); Leg1Position(32.46, 38.69, 0); Leg4Position(17.27, 47.45,0); MoveLegPosition();
   delay(movinSpeed); Leg1Position(0, 50.5, 0); Leg4Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, -22.5); MoveLegPosition();
}
void BackwardStop2(){
   delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, 0); Leg5Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg4Position(0, 50.5, -22.5); Leg5Position(0, 50.5, -22.5); MoveLegPosition();
   delay(movinSpeed); Leg2Position(32.46, 38.69, 0); Leg3Position(17.27, 47.45,0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(0, 50.5, 0); Leg3Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, -22.5); MoveLegPosition();
}



void ForwardStart(){
    delay(movinSpeed);Leg3Position(0, 50.5, 0);Leg4Position(0, 50.5, 0); MoveLegPosition();
    delay(movinSpeed);Leg3Position(17.27, 47.45, 0);Leg4Position(-17.27, 47.45, 0);MoveLegPosition();
    delay(movinSpeed);Leg3Position(17.27, 47.45, -22.5);Leg4Position(-17.27, 47.45, -22.5);MoveLegPosition();
    delay(movinSpeed);Leg2Position(0, 50.5, 0);Leg5Position(0, 50.5, 0);MoveLegPosition();
    delay(movinSpeed);Leg5Position(-32.46, 38.69, 0);MoveLegPosition();
    delay(movinSpeed);Leg5Position(-32.46, 38.69, -22.5);MoveLegPosition();
}
void Forward(){
   delay(movinSpeed); Leg2Position(32.46, 38.69, 0); Leg3Position(17.27, 47.45, 0); Leg6Position(0, 50.5, 0); Leg1Position(0, 50.5, -22.5); Leg4Position(-17.27, 47.45, -22.5); Leg5Position(-32.46, 38.69, -22.5); MoveLegPosition();
   delay(movinSpeed); Leg2Position(32.46, 38.69, -22.5); Leg3Position(17.27, 47.45, -22.5); Leg6Position(0, 50.5, -22.5); MoveLegPosition();  
   delay(movinSpeed); Leg1Position(0, 50.5, 0); Leg4Position(-17.27, 47.45, 0); Leg5Position(-32.46, 38.69, 0); MoveLegPosition();
   
}
void ForwardStop(){
    delay(movinSpeed); Leg3Position(0, 50.5, 0); Leg6Position(0, 50.5, 0); MoveLegPosition();
    delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, -22.5); Leg6Position(0, 50.5, -22.5); MoveLegPosition();
    delay(movinSpeed); Leg1Position(0, 50.5, 0); Leg4Position(0, 50.5, 0); MoveLegPosition();
    delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, -22.5); MoveLegPosition();
}
void Forward2(){
   delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(-17.27, 47.45, -22.5); Leg6Position(-32.46, 38.69, -22.5); Leg1Position(32.46, 38.69, 0); Leg4Position(17.27, 47.45, 0); Leg5Position(0, 50.5, 0); MoveLegPosition();
   delay(movinSpeed); Leg1Position(32.46, 38.69, -22.5); Leg4Position(17.27, 47.45, -22.5); Leg5Position(0, 50.5, -22.5); MoveLegPosition();
   delay(movinSpeed); Leg2Position(0, 50.5, 0); Leg3Position(-17.27, 47.45, 0); Leg6Position(-32.46, 38.69, 0); MoveLegPosition();
}
void ForwardStop2(){
    delay(movinSpeed); Leg4Position(0, 50.5, 0); Leg5Position(0, 50.5, 0); MoveLegPosition();
    delay(movinSpeed); Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, -22.5); Leg5Position(0, 50.5, -22.5); MoveLegPosition();
    delay(movinSpeed); Leg2Position(0, 50.5, 0); Leg3Position(0, 50.5, 0); MoveLegPosition();
    delay(movinSpeed); Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, -22.5); MoveLegPosition();
}


void Leg1Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha1=alpha+a1; beta1=beta+b1; gamma1=gamma+g1;
}
void Leg2Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha2=alpha+a2; beta2=beta+b2; gamma2=gamma+g2;
}
void Leg3Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha3=alpha+a3; beta3=beta+b3; gamma3=gamma+g3;
}
void Leg4Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha4=alpha+a4; beta4=beta+b4; gamma4=gamma+g4;
}
void Leg5Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha5=alpha+a5; beta5=beta+b5; gamma5=gamma+g5;
}
void Leg6Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha6=alpha+a6; beta6=beta+b6; gamma6=gamma+g6;
}
void MoveLegPosition(){
  pwm0.setPWM(13, 0, oppAngle(alpha1)); pwm0.setPWM(14, 0, oppAngle(beta1)); pwm0.setPWM(15, 0, oppAngle(gamma1));
  pwm1.setPWM(2, 0, angle(alpha2)); pwm1.setPWM(1, 0, angle(beta2)); pwm1.setPWM(0, 0, angle(gamma2)); 
  pwm0.setPWM(9, 0, oppAngle(alpha3)); pwm0.setPWM(10, 0, oppAngle(beta3)); pwm0.setPWM(11, 0, oppAngle(gamma3));
  pwm1.setPWM(6, 0, angle(alpha4)); pwm1.setPWM(5, 0, angle(beta4)); pwm1.setPWM(4, 0, angle(gamma4)); 
  pwm0.setPWM(5, 0, oppAngle(alpha5)); pwm0.setPWM(6, 0, oppAngle(beta5)); pwm0.setPWM(7, 0, oppAngle(gamma5));
  pwm1.setPWM(10, 0, angle(alpha6)); pwm1.setPWM(9, 0, angle(beta6)); pwm1.setPWM(8, 0, angle(gamma6)); 
}
void CalculateAngle(float x, float y, float z, float &alpha, float &beta, float &gamma){
  // calculate u-v angle
  float u, v;
  u = sqrt(pow(x, 2) + pow(y, 2));
  v = z;
  beta = PI / 2 - acos((pow(e, 2) + (pow(u - d, 2) + pow(v - c, 2)) - pow(f, 2)) / (2 * e * sqrt(pow(u - d, 2) + pow(v - c, 2)))) - atan2(v - c, u - d);
  gamma = acos((pow(e, 2) + pow(f, 2) - (pow(u - d, 2) + pow(v - c, 2))) / (2 * e * f));
  // calculate x-y-z angle
  alpha = atan2(y, x);
  // transform radian to angle
  alpha = alpha * 180 / PI;
  beta = beta * 180 / PI;
  gamma = gamma * 180 / PI;
}
void CalculatePoint(float alpha, float beta, float gamma, volatile float &x, volatile float &y, volatile float &z){
  // transform angle to radian
  alpha = alpha * PI / 180;
  beta = beta  * PI / 180;
  gamma = gamma * PI / 180;
  // calculate u-v coordinate
  float u, v;
  u = d + e * sin(beta) + f * sin(gamma - beta);
  v = c + e * cos(beta) - f * cos(gamma - beta);
  // calculate x-y-z coordinate
  x = u * cos(alpha);
  y = u * sin(alpha);
  z = v;
}
void receiveData(){
   if ( radio.available()) {             // if receive the data
    while (radio.available()) {          // read all the data
      radio.read( data, sizeof(data) );  // read data
    }
      mode = data[2];
    // control the buzzer
    if (!data[5]){tone(buzzerPin, 2000);}else{noTone(buzzerPin);}
   }
}
int angle(int angle){//Angle conversion
  if(angle>=180){angle=180;}
  if(angle<=0){angle=0;}
  return map(angle,0,180,190,570);//570-190 
}
int oppAngle(int oppAngle){// The opposite direction angle conversion
  if(oppAngle>=180){oppAngle=180;}                  
  if(oppAngle<=0){oppAngle=0;}
 return map(oppAngle,0,180,570,190);//190-570
}
