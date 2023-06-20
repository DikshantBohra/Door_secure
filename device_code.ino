#include <Servo.h>
#include "LowPower.h"

// PIN DECLARTION

#define MOS 10

#define IR 2

#define US_RX 6
#define US_TX 5

#define PIR 3

#define ACT 4

#define ALM 9

#define MODE A3


Servo SR_LIFT;
Servo SR_ROT;
Servo SR_UD;

bool threat= false;
bool alert= false;
bool neut=false;
bool safe=true;
bool pir=false;
bool intr=true;
bool US_al=false;


int timex = 0;



int timeint=10000;
int Downtime= 0;


long unsigned int lowIn;
long unsigned int pause = 5000;
boolean lockLow = true;
boolean takeLowTime;



long duration;
int distance;
int check;
int mode_input=0;
//int mode=1;
bool mode=false;
bool sleeps;
bool wait=true;
bool lift=true;
int count=0;
int pirState;




void PIRSensor();
void SR_FIND();

void detect(){  
  US_al=true;
  }
void off_safe()
  {
   if(check==5){
   Serial.println("SECURE SIGNAL DETECTED");
   sleeps=true;
   check=0;
   }
   Serial.print(check=check+1);
  }

void wakeUp1(){
//  Serial.println("ALERT");
  intr = true;
  sleeps=false;
  lift=false;
  timex=0;
//  Serial.println("HOME SECURITY MODE");
  US_al=false;
  digitalWrite(MOS,LOW);
  }
 
void wakeUp2(){
//  Serial.println("SAFE BOX MODE");
  digitalWrite(MOS,LOW);
  sleeps=false;
  lift=false;
  threat=true;
   attachInterrupt(digitalPinToInterrupt(IR), off_safe, FALLING);
  }
   
void delay10(){
   for(int i=0;i<5;i++)
   {
    Serial.print(".");
    delay(1000);
    }
  }
   
void sleep(){
   detachInterrupt(digitalPinToInterrupt(PIR));
   detachInterrupt(digitalPinToInterrupt(IR));
   Serial.print("\n\nACTIVATING SAFE BOX");
   delay10();
   SR_LIFT.write(0);
//   Serial.println("ACTIVATED");
   check=0;
   alert=false;
   wait=true;
   delay(2000);
   sleeps=false;
   count=0;
   lift=true;
   digitalWrite(MOS,HIGH);
   check_mode();
   LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void check_mode(){
  mode_input=analogRead(MODE);
  if(mode_input==0){
    Serial.println("\n\nACTIVATED: HOME SECURITY MODE\n");
    attachInterrupt(digitalPinToInterrupt(IR), wakeUp1, CHANGE);
    detachInterrupt(digitalPinToInterrupt(PIR));
    mode=true;
    }
  else{
    Serial.println("\n\nACTIVATED: SAFE BOX MODE\n");
    attachInterrupt(digitalPinToInterrupt(PIR), wakeUp2, CHANGE);
    detachInterrupt(digitalPinToInterrupt(IR));
    mode=false;    
    }
  delay(1000);
  }
   

void setup() {
  Serial.begin(9600);
  pinMode(MOS, OUTPUT);
  pinMode(US_RX, INPUT);
  pinMode(US_TX, OUTPUT);
  pinMode(ACT, OUTPUT);
  pinMode(ALM, OUTPUT);
 
  SR_LIFT.attach(12);
  SR_UD.attach(11);
  SR_ROT.attach(13);

  SR_LIFT.write(20);
  SR_UD.write(0);
  SR_ROT.write(0);
  pinMode(IR, INPUT);
  pinMode(PIR,INPUT_PULLUP);
  Serial.println("___________________MOBILIZED SAFE__________________");
  sleep();
 
//  attachInterrupt(digitalPinToInterrupt(IR), wakeUp, FALLING);
 
   
}
void loop() {
  if(wait==true){
//  delay(2000);/
  digitalWrite(MOS,LOW);
  wait=false;
  }
  if(mode==true){
    Serial.println("Security mode");
    detachInterrupt(digitalPinToInterrupt(IR));
     Home_Security();
     }
  if(mode==false){
    detachInterrupt(digitalPinToInterrupt(PIR));
//  attachInterrupt(digitalPinToInterrupt(IR), off_safe, CHANGE);/
    Safe_Box();
    }
 if (sleeps==true)
   {SR_LIFT.write(0);
    sleep();
    }
  delay(1000);
}


//------------------------------PERSONAL_SAFE_BOX------------------

void Safe_Box(){
//  Serial.println("IN safe mode function");/

    PIRSensor();
    if (pir==true){
      Serial.println(" MOMENT DETECTED .....NEUTRALIZING ");
      SR_LIFT.write(80);
      pir=false;
//      delay(2000);/
//      attachInterrupt(digitalPinToInterrupt(IR), off_safe, FALLING);
      for(int i=0;i<2;i++){
       
//        SR_LIFT.write(80);
        delay(300);
        SR_FIND();
        if(threat==false){
//          SR_LIFT.write(0);/
          delay(300);
          break;
          sleeps=true;
          }
        }
    }
   if(count==20){
      SR_LIFT.write(0);
      delay(300);
      Serial.println(" Sleeping......NO MOTION DETECTED");
      delay(1000);
      sleep();
      }
 
 }

//------------------------------HOME_SECURITY------------------

void Home_Security(){
 
   if(intr==true){
     digitalWrite(MOS,LOW);
     attachInterrupt(digitalPinToInterrupt(IR), detect, CHANGE);
      if(US_al==true){
        detachInterrupt(digitalPinToInterrupt(IR));
        Serial.println("DETECTED");
        ALERT();
        intr=false;
      }
     timex=timex+1;
   }
   if (timex>10){
    sleeps=true;
    }
   if(alert==true && threat==true){
     SR_LIFT.write(80);
     delay(300);
     SR_FIND();
     timex=timex+2;
    }
}



//------------------------------OPEN BOX FOR TERMINATION------------------

//void SR_BOX(){
//  if(alert==true){
//    SR_LIFT.write(70);
//    }
//   else{
//    SR_LIFT.write(0);
//    }
//   }
//------------------------------ATTACK------------------


void attack(){
  digitalWrite(ACT,HIGH);
  delay(500);
  digitalWrite(ACT,LOW);
}


//------------------------------ALARM------------------
void alarm(){
  if(alert== true){
    tone(ALM,200);
    }
   else{
    noTone(ALM);
    }
  }
//------------------------------------------ALERT--------------


void ALERT(){
    alert=true;
    threat=true;
    timex=0;
    intr=false;

 }
void SR_FIND(){
  for(int i=10;i<=140;i=i+10){  
   SR_ROT.write(i);
  //  SR_ROT.write(i);
//   delay(100);/
   distance = calculateDistance();
           
        if(distance<10){
               i=i-10;
               Serial.println(distance);
               SR_UD.write(100);
               tone(ALM,200);
                delay(1000);
                attack();
                SR_UD.write(0);
                noTone(ALM);
                Serial.println("Cleared");
                sleeps=true;
                threat=false;
               
             
            }
    }
  // Repeats the previous lines from 140 to 30 degrees
  for(int i=140;i>=10;i=i-10){  
        SR_ROT.write(i);
      //  SR_ROT.write(i);
//        delay(100);/
        distance = calculateDistance();
        if(distance<10){
             i=i+10;    
            Serial.println(distance);
            SR_UD.write(100);
            tone(ALM,200);
            delay(1000);
            attack();
            SR_UD.write(0);
            noTone(ALM);
            threat=false;
            sleeps=true;
            Serial.println("Cleared");
          }
        }

}


int calculateDistance(){
  while (1){
    digitalWrite(US_TX, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TX, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TX, LOW);
    duration = pulseIn(US_RX, HIGH);
    distance= duration*0.034/2;
    if (distance > 1){
      //Serial.println("Exiting While");
//      Serial.print(distance);
      return distance;  
    }
    else{
      //Serial.println("Error");
    }
    }
}

void PIRSensor()
 {
  Serial.println("Reading PIR");
  Serial.println(digitalRead(PIR));
//  delay(500);
//  Serial.println(val);
  if (digitalRead(PIR) == HIGH) {            // check if the input is HIGH
      // turn LED ON
    if (pirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
      pir=true;
    }
  } else {
//    digitalWrite(ledPin, LOW); // turn LED OFF
    if (pirState == HIGH){
      // we have just turned of
      Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
  count=count+1;















 
//   if(digitalRead(PIR) == 1) {
//      if(lockLow) {
//           Downtime = millis();
//           pir = true;
//           lockLow = false;
//           Serial.println("Motion detected.");
//           delay(50);
//      }
//      takeLowTime = true;
//   }
//   if(digitalRead(PIR) == 0) {
//      if(takeLowTime){
//         lowIn = millis();
//         takeLowTime = false;
//      }
//      if(!lockLow && millis() - lowIn > pause) {
//         pir=false;
//         lockLow = true;
//         Serial.println("Motion ended.");
//         delay(50);
//      }
//   }
//   delay(1000);
//   count=count+1;

}
//void srlift(){
//  if(lift==false){
//  for (int i=0;i<=80;i++){
//    SR_LIFT.write(i);
//    lift=true;
//    }
//  }
 
///  }
