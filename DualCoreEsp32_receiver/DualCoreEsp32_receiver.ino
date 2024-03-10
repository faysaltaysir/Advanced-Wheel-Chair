
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include "BluetoothSerial.h"
#include <Arduino.h>
#define RXp2 16
#define TXp2 17
BluetoothSerial serialBT;

//Bluetooth signal Store in this variable
char btSignal;
int check;
//initial Speed
int Speed = 230;
char s ;
//declare channel for pwm Output
#define R 0
#define L 1
#define trigN 12
#define echoN 14

#define trigS 27
#define echoS 26

//PWM Pin for Controlling the speed
int enA = 5;
int enB = 18;

//motor controlling pin
int IN1 = 23;
int IN2 = 22;
int IN3 = 19;
int IN4 = 21;
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

// Task function for Core 1
void core1_task(void *pvParameters) {
  
    while (1) {
        //Serial.println(dirc);
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay 
        if(digitalRead(15)==0){
          if(check==0) {
            stop();dirc = 0;
            delay(2000);
           }
          check = 1;
        //spd = recv_data[2];

    if(dirc == 1){
      //moveForward(spd);
      forward();
      s = 'F';
      int a=1,b=1;
      while(ultrasonicS()<30){
        if(a==1){
          backward();delay(30);
          a=0;  
        }
        
        stop();
        Serial.println("StopS");
        if(dirc!=1) break;
      }
      while(ultrasonicN()>40){
        if(b==1){
          backward();delay(30);
          b=0;  
        }
        stop();
        Serial.println("StopN");
        if(dirc!=1) break;
      }
    }
    if(dirc == 2){
      //moveBackward(spd);
      backward();
      s = 'B';
    }
    if(dirc == 3){
      //moveRight(spd);
      right();
      s = 'R';
    }
    if(dirc == 4){
      //moveLeft(spd);
      left();
      s = 'L';
    }
    if(dirc == 0){
      //stopMotors();
      //stop();
      if(s=='F'){
        backward();delay(30);
      }
      if(s=='B'){
        forward();delay(30);
      }
      if(s=='L'){
        left();delay(30);
      }
      if(s=='R'){
        right();delay(30);
      }
      stop();
    }
    if(dirc == -1){
      stop();
    }
    //delay(100); 
    //val[0]='S';
  }
  else{
    
    stop();delay(500);
    //Serial.println("voice");
    digitalWrite(2,0);
    if(check==1) stop();
    check = 0;
    String val = Serial2.readString();
    //Serial.println(val);
    int c=1,d=1;
    if(val[0] == 'M'){
      //moveForward(spd);
      forward_Voice();
      //delay(2000);
      stop();
      //Serial.print(val);
    }
    else if(val[0] == 'B'){
      //moveBackward(spd);
      backward();
      delay(1000);
      stop();
    }
    else if(val[0] == 'R'){
      //moveRight(spd);
      right();
      delay(1000);
      stop();
    }
    else if(val[0] == 'L'){
      //moveLeft(spd);
      left();
      delay(1000);
      stop();
    }
    else if(val[0] == 'S'){
      //stopMotors();
      stop();
    }
    recv_data[1] = 0;
    digitalWrite(2,0);
  }
 }
}

void setup() {
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

  //Intial State of Car
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.begin(115200);
  Serial2.begin(9600,SERIAL_8N1,RXp2,TXp2);
  //Bluetooth Name
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

void loop() {
    // This loop is empty because tasks are running in parallel
    delay(1000); // This delay doesn't affect the multitasking
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

  // Delay before the next reading
  //delay(100); // 1 second
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


void left() {
  ledcWrite(R, 200);
  ledcWrite(L, 200);

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  }

void right() {
  ledcWrite(R, 200);
  ledcWrite(L, 200);

  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void  backward() {
  ledcWrite(R, Speed);
  ledcWrite(L, Speed);

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}

void forward() {
  ledcWrite(R, Speed);
  ledcWrite(L, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void forward_Voice() {
//  while(ultrasonicS()<30){
//        if(c==1){
//          backward();delay(30);
//          c=0;  
//        }
//        
//        stop();
//        Serial.println("StopS");
//        //if(dirc!=1) break;
//      }
//      while(ultrasonicN()>40){
//        if(d==1){
//          backward();delay(30);
//          d=0;  
//        }
//        stop();
//        Serial.println("StopN");
//        //if(dirc!=1) break;
//      }
int a = 0;
int s = ultrasonicS();
Serial.println(s);
while(s>30){
  s = ultrasonicS();
  ledcWrite(R, Speed);
  ledcWrite(L, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  a++;
  if(a>300){
    s = 100;
    break;
  }
}
 stop();

s = 100;
  
}
  

void stop() {
    
    ledcWrite(R, Speed);
    ledcWrite(L, Speed);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    s='s';
}
