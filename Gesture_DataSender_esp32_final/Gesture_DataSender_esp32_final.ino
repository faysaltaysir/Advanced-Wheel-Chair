#include "BluetoothSerial.h"
const int x_ax = 34;
const int y_ax = 35;

int position;

int x;
int y;

BluetoothSerial SerialBT;

String MACadd = "08:D1:F9:9A:87:4E"; // motor control//
uint8_t address[6]  = {0x08, 0xD1, 0xF9, 0x9A, 0x87, 0x4E};//Write Drone side MAC address in HEX


bool connected;

void setup() {
    pinMode(x_ax,OUTPUT);
  pinMode(y_ax,OUTPUT);

  Serial.begin(115200);
  SerialBT.begin("ESP32test", true); 
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  
   connected = SerialBT.connect(address);
  
  if(connected) {
    Serial.println("Connected Succesfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
  // disconnect() may take upto 10 secs max
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();


}

uint8_t calculate_checksum(uint8_t *data) {
  uint8_t checksum = 0;
  checksum |= 0b11000000 & data[1];
  checksum |= 0b00110000 & data[2];
  checksum |= 0b00001100 & data[3];
  checksum |= 0b00000011 & data[4];
  return checksum;
}

void loop() {
  
  int speed  = 150;
  uint8_t send_data[6];
  int x = analogRead(x_ax);
  int y = analogRead(y_ax);

  if(x>=1700 && x<=2000 && y>=1700 && y<=2000){
    position = 0;
  }
  else if (x<1700){
    position = 3;
  }
  else if (x>2000){
    position = 4;
  }
  else if (y<1700){
    position = 2;
  }
  else if (y>2000){
    position = 1;
  }

Serial.print("     x       ");
Serial.print(x);
Serial.print("     y       ");
Serial.print(y);
Serial.print("         position      ");
Serial.println(position);






  send_data[0] = 'T';
  send_data[1] = position;
  send_data[2] = speed;
  send_data[3] = x;
  send_data[4] = y;
  send_data[5] = calculate_checksum(send_data);
  SerialBT.write(send_data, 6);

  delay(20);
}
