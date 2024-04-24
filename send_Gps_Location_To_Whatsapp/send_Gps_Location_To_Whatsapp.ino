#include <WiFi.h>         //Including wifi.h library it will take care of all wifi related task
#include <HTTPClient.h>   //Including HTTPClient.h library to use all api
#include <HardwareSerial.h> 
const char* ssid = "realme C25";             //Add your WiFi ssid
const char* password =  "11111111";    //Add your WiFi password
//fatin vai
String apiKey = "3893691";              //Add your Token number that bot has sent you on WhatsApp messenger
String phone_number = "8801876313623"; //Add your WhatsApp app registered phone number (same number that bot send you in url)

//shan
//String apiKey = "5046106";              //Add your Token number that bot has sent you on WhatsApp messenger
//String phone_number = "8801878904575"; //Add your WhatsApp app registered phone number (same number that bot send you in url)

String url;                            //url String will be used to store the final generated URL
String message = "";
String message1 = "";
int check = 0;
char ready;
const int buttonPin = 12;  // Pin connected to the push button
int buttonState = HIGH;   // Current state of the button
int lastButtonState = HIGH;  // Previous state of the button
unsigned long lastDebounceTime = 0;  // Last time the button state changed
unsigned long debounceDelay = 50;    // Debounce time in milliseconds




#include <HardwareSerial.h> // Include the hardware serial library for ESP32
#include <TinyGPS++.h>     // Include the TinyGPS++ library for parsing GPS data

// Define the serial port connected to the GPS module
HardwareSerial gpsSerial(1);

// Define the TinyGPS++ object
TinyGPSPlus gps;
//int swtch = 12;
int led = 4;
void setup() {
  pinMode(led,OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);  // Enable internal pullup resistor
  pinMode(2,OUTPUT);
  Serial.begin(115200); // Start serial communication for debugging
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // Start serial communication with GPS module
  WiFi.begin(ssid, password);              // Try to connect with the given SSID and PSS
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {  // Wait until WiFi is connected
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to the WiFi network"); // Print wifi connect message
  delay(5000);
  // use message_to_whatsapp function to send your own message
  message_to_whatsapp("We are successfully connected"); 
  check = 0;
}

void loop(){
  char print_buf[300];
  char print_buf2[300];
  gpsLocation(); 
  sprintf(print_buf,"I am in a Denger!\nhttps://www.google.com/maps?q=%lf,%lf",gps.location.lat(),gps.location.lng());
  sprintf(print_buf2,"I am in a Denger!\nhttps://www.google.com/maps?q=%s,%s","22.4619","91.9711"); 
  Serial.println(print_buf);
  int but = button();
  Serial.println(but);
  digitalWrite(led,0);
  if(but==0  && ready =='R'){
    Serial.println(but);
    digitalWrite(led,1);
    while(check!=1 ){
      digitalWrite(led,0);
      message_to_whatsapp(print_buf);  
      Serial.print("inside");
      digitalWrite(led,1);
    }
    digitalWrite(led,1);
    delay(4000);
    digitalWrite(led,0);
    digitalWrite(2,0);
    check = 0;
    but=1;
    //delay(200);
  }
  else if(but==0  && ready =='N'){
    Serial.println(but);
    digitalWrite(led,1);
    while(check!=1 ){
      digitalWrite(led,0);
      message_to_whatsapp(print_buf2);  
      Serial.print("inside");
      digitalWrite(led,1);
    }
    digitalWrite(led,1);
    delay(4000);
    digitalWrite(led,0);
    digitalWrite(2,0);
    check = 0;
    but=1;
    //delay(200);
  }
  else{
    Serial.println("No");
    digitalWrite(led,0);
  }    
}

void gpsLocation() {
  // Read data from GPS module
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // If new data is parsed successfully
      if (gps.location.isValid()) {
        // If valid location data is available
//        Serial.print("Latitude: ");
//        Serial.println(gps.location.lat(), 6); // Print latitude
//        Serial.print("Longitude: ");
//        Serial.println(gps.location.lng(), 6); // Print longitude
//        Serial.print("Altitude (meters): ");
//        Serial.println(gps.altitude.meters()); // Print altitude
        ready = 'R';
      } else {
        Serial.println("Waiting for GPS fix...");
        ready = 'N';
      }
    }
  }
}


void  message_to_whatsapp(String message)       // user define function to send meassage to WhatsApp app
{
  //adding all number, your api key, your message into one complete url
  url = "https://api.callmebot.com/whatsapp.php?phone=" + phone_number + "&apikey=" + apiKey + "&text=" + urlencode(message);

  postData(); // calling postData to run the above-generated url once so that you will receive a message.
}

void postData()     //userDefine function used to call api(POST data)
{
  int httpCode;     // variable used to get the responce http code after calling api
  HTTPClient http;  // Declare object of class HTTPClient
  http.begin(url);  // begin the HTTPClient object with generated url
  httpCode = http.POST(url); // Finaly Post the URL with this function and it will store the http code
  if (httpCode == 200)      // Check if the responce http code is 200
  {
    Serial.println("Sent ok."); // print message sent ok message
    check = 1;
  }
  else                      // if response HTTP code is not 200 it means there is some error.
  {
    Serial.println("Error."); // print error message.
    check = 0;
  }
  http.end();          // After calling API end the HTTP client object.
}

String urlencode(String str)  // Function used for encoding the url
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}


int button() {
  int reading = digitalRead(buttonPin);  // Read the state of the button

  return reading;
}
