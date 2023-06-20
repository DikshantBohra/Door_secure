//FINAL DOOR CODE





#include <SoftwareSerial.h>
//#include "Adafruit_FONA.h" // used for GSM module/
#include "LowPower.h"

#include <SPI.h>
#include <RFID.h>


//#define RFID_MISO 12
//#define RFID_MOSI 11
//#define RFID_SCK 13

#define SS_PIN 10
#define RST_PIN 9

#define Vib_sen 2  //Input for Vibration Sensor

#define PIR A2

//#define POW_chk 9

#define IR_tx 8

//#define GSM_RX 4
//#define GSM_TX 5
//#define GSM_RST 3
//#define GSM_DTR 6

#define POW_MODFET A0

#define AlM 7

//#define ON_BUT 8

#define ESP 8

RFID rfid(SS_PIN, RST_PIN);
String Card="51 38 135 75";
String rfidCard;

//--------------------------------------------------
bool intr = false;
bool vib = false;
bool pir = false;
//bool rfid = false;/
bool charging = false;
bool alert = false;
bool SMS_SENT=false;
bool call= false;
bool secure=false;
bool ini=false;
bool wait=false;

int power=0;

//------------------------PIR Sensor Variable-------

long unsigned int lowIn;
long unsigned int pause = 5000;
boolean lockLow = true;
boolean takeLowTime;


//--------------------------Vibration Sensor Variable--------
int vib_data=0;
int vib_sens=7; //should be adjusted according to the environment


//--------------------------GSM_MODULE------------------------


char replybuffer[255];
//SoftwareSerial mySerial(GSM_TX,GSM_RX);
char incomingByte;
String inputString;

void Vib_sens();
void RFID_READ();
void GSM_RCV_MSG();
void PIRSensor();
void IR_COM();
void GSM_MSG_SEND();
void GSM_CALL();


void wakeUp()
{
//  Serial.println("Triggered");
  intr = true;
  digitalWrite(POW_MODFET,LOW);
  wait=true;
 
}
void delay10(){
   for(int i=0;i<5;i++)
   {
    Serial.print(".");
    delay(1000);
    }}
void sleep(){
  delay10();
  attachInterrupt(digitalPinToInterrupt(Vib_sen), wakeUp, FALLING);
  digitalWrite(POW_MODFET,HIGH);
  digitalWrite(ESP,HIGH);
  ini=false;
  alert=false;
  Serial.println("\n\n\n     ACTIVATED");
  delay(1000);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
   }

void setup() {
   pinMode(Vib_sen, INPUT_PULLUP);    
   
    pinMode(PIR, INPUT);
//    pinMode(Vib_sen, INPUT);
//    pinMode(POW_chk,INPUT);/
    pinMode(IR_tx,OUTPUT);
    pinMode(AlM,OUTPUT);
//    pinMode(ON_BUT,INPUT);
    pinMode(POW_MODFET,OUTPUT);

    pinMode(ESP,OUTPUT);
   
    SPI.begin();
    rfid.init();
//    initiallize();/
    Serial.begin(9600);
//    mySerial.begin(9600);
    digitalWrite(POW_MODFET,HIGH);
    Serial.println("_______________DOOR SECURE________________\n\n");
    Serial.print("ACTIVATING THE DOOR SECURE");
    sleep();

}

void loop() {

if(wait==true){
  Serial.println("\n\n TRIGGERED \n");
//  delay(2000);
  wait=false;
  }
if(intr == true && secure==false){
  Vib_sens();
  if(vib==true){
    digitalWrite(POW_MODFET,LOW);
    detachInterrupt(digitalPinToInterrupt(Vib_sen));
    PIRSensor();
    if(pir==true){
      Serial.println("\n\nHIGH ON ALERT\n");
     digitalWrite(ESP,LOW);
      alert=true;
      intr=false;
      pir=false;
      vib=false;
      }
    }
}
 delay(500);
// GSM_RCV_MSG();
 RFID_READ();
 
if (alert==true && secure==false){
  Serial.println("\nDANGER\n");
  IR_COM();
//  tone(7,200);
  if(SMS_SENT== false){
//    GSM_MSG_SEND();
    }
   
  if(call== false){
//    GSM_CALL();
    }
 
  }

if(secure== true){
  secure= false;
  noTone(AlM);
  Serial.println("\n CAUTION:- TURN OFF Device Before Reactivating\n");
  delay(1000);
  Serial.print("\nREACTIVATING IN 8 SECOND");
  delay(3000);
//delay10();
  sleep();
  }


//  digitalWrite(POW_MODFET,LOW);
//  GSM_CALL();
 
//  Vib_sens();
//  if(vib){
//    RFID_READ();
//     delay(5000);
//     }
   

}





void PIRSensor() {
//   Serial.println("Reading PIR");/
//   Serial.println(digitalRead(PIR)+"\n");/
   
   if(digitalRead(PIR) == 1) {
      if(lockLow) {
         pir = true;
         lockLow = false;
         Serial.println("Motion detected.");
         delay(50);
      }
      takeLowTime = true;
   }
   if(digitalRead(PIR) == 0) {
      if(takeLowTime){
         lowIn = millis();
         takeLowTime = false;
      }
      if(!lockLow && millis() - lowIn > pause) {
         pir=false;
         lockLow = true;
         Serial.println("Motion ended.");
         delay(50);
      }
   }
}


//------------------------Vibration Sensor---------------
void Vib_sens(){
  vib_data = analogRead(Vib_sen);
//  Serial.println(vib_data);/
  if(vib_data>vib_sens){
//    Serial.print("Vibration Detected");
    vib=true;      
  }
  else {
//    Serial.print(" NO Vibration Detected");/
//    vib=false;
  }
 
}

//------------------------------Power Check--------------------------
//void Power(){
//  power = digitalRead(POW_chk);
//  if(power==1){
//    charging=true;      
//  }
//  else{
//    charging=false;
//  }
//  
//}

//------------------------------Ultrasonic-----BUZZER ----------------------------
void IR_COM(){
   if(alert==true){
     Serial.println("\nALERTING FELLOW DEVICE\n");
    for(int i=0;i<10;i++){
      digitalWrite(IR_tx,HIGH);
//    tone(AlM,2000);
//      delay(500);/
      delay(100);
      digitalWrite(IR_tx,LOW);
      delay(100);
    }
    }
    else{
    digitalWrite(IR_tx,LOW);
    noTone(AlM);
    }
}

//---------------------------GSM MODULE COMMUNICATION----------------------------

//void GSM_RCV_MSG(){
//  initiallize();
//  Serial.println("Trying initialization");
//  if(mySerial.available()){
//    mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
//    mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
//    mySerial.println("AT+CNMI=1,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
//    mySerial.println("AT+CMGL=\"REC UNREAD\"");
//    delay(100);
//      
//      while(mySerial.available())
//       {
//        incomingByte = mySerial.read();
//        inputString += incomingByte;
//       }
//   inputString.toUpperCase(); // Uppercase the Received Message
//   if (inputString.indexOf("STOP*") > -1){
//     alert = false;
//     secure=true;
//     Serial.println("Secured");
//    }
//   }
//  
//      
//  }
//
//void GSM_CALL(){
//   initiallize();
//   if(mySerial.available()){
//        mySerial.println("AT"); //Once the handshake test is successful, i t will back to OK
//        mySerial.println("ATD+ +919456830171;"); //  change ZZ with country code and xxxxxxxxxxx with phone number to dial
//        delay(10000); // wait for 20 seconds...
//        mySerial.println("ATH"); //hang up    
//        call=true;
//   }
//  
//  }
//void GSM_MSG_SEND(){
//  initiallize();
//  if(mySerial.available()){
//      mySerial.println("AT");
//      mySerial.println("AT+CMGF=1");
//      mySerial.println("AT+CMGS=\"+917668995572\"");
//      mySerial.print("ALert ALert ALert");
//      mySerial.write(26);
//      SMS_SENT=true;  
//    }
//  }
 
//void initiallize(){
//  if(ini==false){
////   Serial.println("Trying initialization");
//   if(mySerial.available()){
//   for(int x=0;x<2;x++){
//    Serial.println("Initializing...");
//    delay(1000);
//  
//    mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
//    mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
//    mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
//    mySerial.println("AT+CREG?"); //Check whether it has registered in the network
//    ini=true;
//
//   }
//   exit;
//  }  
// }
//}

//---------------------------RFID MODULE----------------------------

void RFID_READ(){
  digitalWrite(A0,LOW);
//   Serial.print("Scanning");
 for(int i=0;i<5;i++){
  delay(100);
//   Serial.print(".");
  if (rfid.isCard()) {
    Serial.print("Card :");
    if (rfid.readCardSerial()) {
      rfidCard = String(rfid.serNum[0]) + " " + String(rfid.serNum[1]) + " " + String(rfid.serNum[2]) + " " + String(rfid.serNum[3]);
      Serial.println(rfidCard);
      if (rfidCard == "51 38 135 75") {
        Serial.println("Card Detected \n\n\n        ... SECURED...\n\n");
        delay(1000);
        secure=true;
          tone(7,200);
          delay(1000);
          noTone(7);
          delay(1000);
          tone(7,200);
          delay(1000);
          noTone(7);
          break;
      } else {
        Serial.println("Unknown Card \n....ALERT....");
//        secure=true;
//        tone(7,100);
//        delay(2000);
//        noTone(7);
      }
    }
    rfid.halt();
  }
}


}
