#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

/**        
  Connection
  Arduino    VoiceRecognitionModule
   2   ------->     TX
   3   ------->     RX
*/
VR myVR(2,3);    // 2:RX 3:TX, you can choose your favourite pins.

uint8_t records[7]; // save record
uint8_t buf[64];

int led = 13;

#define Forward   (0)
#define Backward  (1) 
#define Right     (2)
#define Left      (3) 
#define Stop      (4) 


void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      //Serial.write(buf[i]);
    }
    else{
//      Serial.print("[");
//      Serial.print(buf[i], HEX);
//      Serial.print("]");
    }
  }
}

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf  -->  VR module return value when voice is recognized.
             buf[0]  -->  Group mode(FF: None Group, 0x8n: User, 0x0n:System
             buf[1]  -->  number of record which is recognized. 
             buf[2]  -->  Recognizer index(position) value of the recognized record.
             buf[3]  -->  Signature length
             buf[4]~buf[n] --> Signature
*/
void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
 }
  Serial.println("\r\n");
}

void setup()
{
  /** initialize */
  myVR.begin(9600);
  
  Serial.begin(9600);
  
  pinMode(led, OUTPUT);
    
  if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
  }else{
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while(1);
  }
  
  if(myVR.load((uint8_t)Forward) >= 0){
   // Serial.println("onRecord loaded");
  }
  
  if(myVR.load((uint8_t)Backward) >= 0){
    //Serial.println("offRecord loaded");
  }
  
  if(myVR.load((uint8_t)Right) >= 0){
    //Serial.println("onRecord loaded");
  }
  
  if(myVR.load((uint8_t)Left) >= 0){
    //Serial.println("offRecord loaded");
  }
  
  if(myVR.load((uint8_t)Stop) >= 0){
    //Serial.println("onRecord loaded");
  }
  
}

void loop()
{
  int ret;
  ret = myVR.recognize(buf, 50);
  if(ret>0){
    switch(buf[1]){
      case Forward:
        /** turn on LED */
        digitalWrite(led, HIGH);
        Serial.println("MOVE");
        break;
      case Backward:
        /** turn off LED*/
        digitalWrite(led, LOW);
        Serial.println("BACKWARD");
        break;
        case Right:
        /** turn on LED */
        digitalWrite(led, HIGH);
        Serial.println("RIGHT");
        break;
      case Left:
        /** turn off LED*/
        digitalWrite(led, LOW);
        Serial.println("LEFT");
        break;
        case Stop:
        /** turn on LED */
        digitalWrite(led, HIGH);
        Serial.println("STOP");
        break;
      default:
        //Serial.println("Record function undefined");
        break;
    }
    /** voice recognized */
   // printVR(buf);
  }
}
