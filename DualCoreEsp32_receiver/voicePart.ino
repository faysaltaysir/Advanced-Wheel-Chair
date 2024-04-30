#define RXp2 16
#define TXp2 17

#define trigN 12
#define echoN 14

#define trigS 27
#define echoS 26
int R_Speed = 150;
int L_Speed = 150;
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

}
String val="";
char str;
void loop() {
  // put your main code here, to run repeatedly:
    if(Serial2.available()>0){
      val = Serial2.readString();  
    }
    
    if(val[0]=='M'){
        Serial.println("Forward");
        voiceForward();
        int front = ultrasonicS();
        if(front < 30){
            Serial.println("something front");
            val[0] = 'S';
        }
        str = 'm';
    }
    if(val[0]=='B'){
        int count = 0;
        Serial.println("BackwardOUT");
        while(count<100 && val[0]!='S'){
            Serial.println("Backward");
            voiceBackward();
            int back = ultrasonicN();
            if(back < 30){
                Serial.println("something back");
                val[0] = 'S';
            }
            val[0] == 'S';  
            count++;
        }
        
    }
    if(val[0]=='L'){
        Serial.println("Left");
        voiceLeft();
        delay(1000);
        if(str == 'm') 
            val[0]='M';
        else
            val[0]='S';
    }
    if(val[0]=='R'){
        Serial.println("Right");
        voiceRight();
        delay(1000);
        if(str == 'm') 
            val[0]='M';
        else
            val[0]='S';
    }
    if(val[0]=='S'){
        Serial.println("Stop");
        stop();
        str = 's';
    }
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
  ledcWrite(R, R_Speed);
  ledcWrite(L, L_Speed);

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}


void voiceForward() {
  ledcWrite(R, R_Speed);
  ledcWrite(L, L_Speed);

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
