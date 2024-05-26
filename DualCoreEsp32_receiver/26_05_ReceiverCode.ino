#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
int x = 0,y=0;
char s ;
#include "BluetoothSerial.h"
#include <Arduino.h>
BluetoothSerial serialBT;

//Bluetooth signal Store in this variable
char btSignal;
int check;

#define RXp2 16
#define TXp2 17

#define trigN 12
#define echoN 14

#define trigS 27
#define echoS 26
int R_Speed = 150;
int L_Speed = 185;
//declare channel for pwm Output
#define R 0
#define L 1
int enA = 5;
int enB = 18;

//motor controlling pin
int IN1 = 23;
int IN2 = 22;
int IN3 = 21;
int IN4 = 19;

uint8_t recv_data[6];

TaskHandle_t Core0Task;
TaskHandle_t Core1Task;
int a =0;
int dirc = -1;

// Task function for Core 0
uint8_t calculate_checksum(uint8_t *data) {
  uint8_t checksum = 0;
  checksum |= 0b11000000 & data[1];
  checksum |= 0b00110000 & data[2];
  checksum |= 0b00001100 & data[3];
  checksum |= 0b00000011 & data[4];
  return checksum;
}
void core0_task(void *pvParameters) {
    while (1) {
        //Serial.println("Core 0 is running");
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 1 second
          
      if (serialBT.available()) {
        serialBT.readBytes(recv_data, 6);
        if (recv_data[0] != 'T') {
          Serial.print("Receive error!");
          return;
         // stop();
          digitalWrite(2,0);
        }
        if (recv_data[5] != calculate_checksum(recv_data)) {
          Serial.print("Decode error!");
          return;
//          stop();
          digitalWrite(2,0);
        }
        Serial.printf("Direction: %d\n", recv_data[1]);
        digitalWrite(2,1);
      }
      dirc = recv_data[1];
    }
}


void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Serial2.begin(9600,SERIAL_8N1,RXp2,TXp2);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(trigN, OUTPUT);
    pinMode(echoN, INPUT);
    pinMode(trigS, OUTPUT);
    pinMode(echoS, INPUT);
    pinMode(2, OUTPUT);
    // Setup PWM channels
    ledcSetup(R, 20000, 8);  // Channel 0 for Motor A, 5 kHz frequency, 8-bit resolution
    ledcAttachPin(enA, R);
    ledcSetup(L, 20000, 8);  // Channel 0 for Motor A, 5 kHz frequency, 8-bit resolution
    ledcAttachPin(enB, L);
  
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(15, INPUT_PULLUP);

    serialBT.begin("WheelChair");
  Serial.println("The device started, now you can pair it with bluetooth!");
  
 
    // Create task for core 0
    xTaskCreatePinnedToCore(
        core0_task,   /* Task function */
        "Core0Task",  /* Task name */
        10000,        /* Stack size */
        NULL,         /* Task parameter */
        1,            /* Priority */
        &Core0Task,   /* Task handle */
        0             /* Core ID */
    );

    // Create task for core 1
    xTaskCreatePinnedToCore(
        core1_task,   /* Task function */
        "Core1Task",  /* Task name */
        10000,        /* Stack size */
        NULL,         /* Task parameter */  
        1,            /* Priority */
        &Core1Task,   /* Task handle */
        1             /* Core ID */
    );


}
String val="";
char str;

void core1_task(void *pvParameters) {
  
    while (1) {
      
        //Serial.println(dirc);
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay 
        if(digitalRead(15)==0){
          val[0]='S';
          if(check==0) {
            stop();dirc = 0;
            delay(2000);
           }
          check = 1;
        //spd = recv_data[2];

    if(dirc == 1){
      //moveForward(spd);
      voiceForward();
      Forward();
      s = 'F';
      int a=1;
      int frontH = ultrasonicS(); 
      while(frontH<30){
        if(a==1 && frontH<20){
          voiceBackward();delay(30);
          a=0;  
        }
        frontH = ultrasonicS();
        stop();
        Serial.println("StopS");
        if(dirc!=1) break;
      }
      
    }
    if(dirc == 2){
      //moveBackward(spd);
      //voiceBackward();
      Backward();
      s = 'B';
      int b=1;
      int backH = ultrasonicN();
      while(backH<30){
        if(b==1 && backH<20){
          voiceForward();delay(30);
          b=0;  
        }
        backH = ultrasonicN();
        stop();
        Serial.println("StopS");
        if(dirc!=2) break;
      }
      
    }
    if(dirc == 3){
      //moveRight(spd);
      //voiceRight();
      Right();
      s = 'R';
    }
    if(dirc == 4){
      //moveLeft(spd);
      //voiceLeft();
      Left();
      s = 'L';
    }
    if(dirc == 0){
      //stopMotors();
      //stop();
//      if(s=='F'){
//        voiceBackward();delay(30);
//      }
//      if(s=='B'){
//        voiceForward();delay(30);
//      }
//      if(s=='L'){
//        voiceLeft();delay(30);
//      }
//      if(s=='R'){
//        voiceRight();delay(30);
//      }
      stop();
    }
    if(dirc == -1){
      stop();
    }
    //delay(100); 
    //val[0]='S';
  }
  else{
      dirc = 0;
      if(Serial2.available()>0){
        val = Serial2.readString();
        Serial.println(val);  
    }
    
    if(val[0]=='M'){
        Serial.println("Forward");
        //voiceForward();
        voiceForward();
        y=0;
        int front = ultrasonicS();
        
        if(front < 30){
            Serial.println("something front");
            if(front < 20 && y==0){
                Backward();delay(40);
                y=1;
            }
            val[0] = 'S';
        }
        str = 'm';
    }
    if(val[0]=='B'){
        Serial.println("Backward");
        //voiceForward();
        voiceBackward();
        y=0;
        int back = ultrasonicN();
        
        if(back < 30){
            Serial.println("something back");
            if(back < 20 && y==0){
                Forward();delay(40);
                y=1;
            }
            val[0] = 'S';
        }
        str = 'b';
    }


//    
//    if(val[0]=='B'){
//        int count = 0;x=0;str = 'b';
//        Serial.println("BackwardOUT");
//        while(count<100 && val[0]!='S'){
//            Serial.println("Backward");
//            //voiceBackward();
//            voiceBackward();
//            int back = ultrasonicN();
//            if(back < 30){
//                Serial.println("something back");
//                if(back < 20 && x==0){
//                    Backward();delay(40);
//                    x=1;
//                }
//                val[0] = 'S';
//            }
//            val[0] == 'S';  
//            count++;
//        }
//        
//    }
    if(val[0]=='L'){
        Serial.println("Left");
        //voiceLeft();
        Left();
        delay(600);
//        if(str == 'm') 
//            val[0]='M';
//        else
            val[0]='S';
    }
    if(val[0]=='R'){
        Serial.println("Right");
        //voiceRight();
        Right();
        delay(600);
//        if(str == 'm') 
//            val[0]='M';
//        else
            val[0]='S';
    }
    if(val[0]=='S'){
        Serial.println("Stop");
        if(str == 'm'){
          Backward();delay(40);
        }
        else if(str == 'b'){
          Forward();delay(40);
        }
        stop();
        str = 's';
        x=0;
    }
  }
    }
}



void loop() {
  // put your main code here, to run repeatedly:
    delay(1000);
}
int ultrasonicS() {
    long duration, distance; 
    digitalWrite(trigS, LOW);
    delayMicroseconds(2);
    
    digitalWrite(trigS, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigS, LOW);
    
    duration = pulseIn(echoS, HIGH);
    
    distance = duration * 0.034 / 2;
    
    // Print the distance to the Serial Monitor
    Serial.print("Distance1: ");
    Serial.print(distance);
    Serial.println(" cm");
  
    return distance;
}

int ultrasonicN() {
  long duration, distance; 
  digitalWrite(trigN, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigN, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigN, LOW);
  
   duration = pulseIn(echoN, HIGH);
  
   distance = duration * 0.034 / 2;
  
  // Print the distance to the Serial Monitor
  Serial.print("Distance2: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Delay before the next reading
  //delay(100); // 1 second
  return distance;
}
void  voiceBackward() {
  ledcWrite(R, 167);
  ledcWrite(L, 160);

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}


void voiceForward() {
  ledcWrite(R, 160);
  ledcWrite(L, 167);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void voiceLeft(){
  ledcWrite(R, 0);
  ledcWrite(L, L_Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void voiceRight(){
  ledcWrite(R, R_Speed);
  ledcWrite(L, 0);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}  

void stop() {
    
    ledcWrite(R, R_Speed);
    ledcWrite(L, L_Speed);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}



void Left() {
  ledcWrite(R, 180);
  ledcWrite(L, 180);

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  }

void Right() {
  ledcWrite(R, 180);
  ledcWrite(L, 180);

  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void  Backward() {
  ledcWrite(R, L_Speed);
  ledcWrite(L, R_Speed);

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}

void Forward() {
  ledcWrite(R, R_Speed);
  ledcWrite(L, L_Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
