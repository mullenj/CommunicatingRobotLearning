#include <analogWrite.h>
char command = ' ';
int power = 0;
int duration = 0;
int vibrotactors[] = {15,25,27,33,26,12};
int driver_1 = 14;
int driver_2 = 32;
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
volatile double RPM = 0;
volatile uint32_t lastA = 0;
// Connect to the two encoder outputs!
#define ENCODER_A   21
#define ENCODER_B   4
// These let us convert ticks-to-RPM6
#define GEARING     20
#define ENCODERMULT 12
BluetoothSerial SerialBT;

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

void setup() {
  Serial.begin(115200);
  pinMode(vibrotactors[0], OUTPUT);
  pinMode(vibrotactors[1], OUTPUT);
  pinMode(vibrotactors[2], OUTPUT);
  pinMode(vibrotactors[3], OUTPUT);
  pinMode(vibrotactors[4], OUTPUT);
  pinMode(vibrotactors[5], OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(driver_1, OUTPUT);
  pinMode(driver_2, OUTPUT);
  SerialBT.begin("haptic_device_2"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  attachInterrupt(ENCODER_A, interruptA, RISING);
}
void loop() {
  recvWithEndMarker();
  process_command();
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
   
    while (SerialBT.available() > 0 && newData == false) {
        rc = SerialBT.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void process_command() {
    if (newData == true) {
        command = receivedChars[0];
        power = (receivedChars[1]- '0') * 150;
        duration = receivedChars[2] - '0';
        if(command == 'A' ){ //10%
          all_on(power,duration);
        }
        else if(command == 'L' ){ 
          left(power,duration); //20%
        }
        else if(command == 'R' ){
          right(power,duration); //30%
        }
        else if(command == 'T' ){
          top(power,duration);  // 40%
        }
        else if(command == 'B' ){
          bottom(power,duration); // 50%
        }
        else if(command == 'C' ){
          circle(power,duration); //60%
        }
        else if(command == 'S' ){ //70%
          striped(power,duration);
        }
        else if(command == 'O' ){ //80%
          tense(power,1);
        }
        else if(command == 'P' ){
          relax(power,1);
        }
        else if(command == 'N' ){ //90%
          ninety(power,1);
        }
        else if(command == 'H' ){ //100%
          hundred(power,1);
        }
        else if(command == 'H' ){
          LED_test(duration);
        }
        all_off();
        Serial.print(command);
        Serial.print(power);
        Serial.println(duration);
        newData = false;
    }
}

void all_on(int p, int d){
    analogWrite(driver_1, 1023*0.1); 
    analogWrite(driver_2, 0);
    delay(10000);
}
void left(int p, int d){ 
    analogWrite(driver_1, 1023*0.2); 
    analogWrite(driver_2, 0);
    delay(10000);
}
void right(int p, int d){
    analogWrite(driver_1, 1023*0.3); 
    analogWrite(driver_2, 0);
    delay(8000);
}
void top(int p, int d){
    analogWrite(driver_1, 1023*0.4); 
    analogWrite(driver_2, 0);
    delay(8000);
}
void bottom(int p, int d){
    analogWrite(driver_1, 1023*0.5); 
    analogWrite(driver_2, 0);
    delay(8000);
}
void circle(int p, int d){
    analogWrite(driver_1, 1023*0.6); 
    analogWrite(driver_2, 0);
    delay(8000);
}
void striped(int p, int d){
    analogWrite(driver_1, 1023*0.7); 
    analogWrite(driver_2, 0);
    delay(6000);
}
void tense(int p, int d){
    analogWrite(driver_1, 1023*0.8);
    analogWrite(driver_2, 0);
    delay(6000);
}
void relax(int p, int d){
    analogWrite(driver_1, 0);
    analogWrite(driver_2, p);
    delay(1000);
}
void ninety(int p, int d){
    analogWrite(driver_1, 1023*0.9);
    analogWrite(driver_2, 0);
    delay(5000);
}
void hundred(int p, int d){
    analogWrite(driver_1, 1023);
    analogWrite(driver_2, 0);
    delay(5000);
}
void LED_test(int d){
    Serial.println("LED test ran");
    digitalWrite(LED_BUILTIN,HIGH);
    delay(d*1000);
    digitalWrite(LED_BUILTIN,LOW);
}
void all_off(){
    analogWrite(vibrotactors[0],0); 
    analogWrite(vibrotactors[1],0); 
    analogWrite(vibrotactors[2],0); 
    analogWrite(vibrotactors[3],0);  
    analogWrite(vibrotactors[4],0); 
    analogWrite(vibrotactors[5],0);
    analogWrite(driver_1,0);  
    analogWrite(driver_2,0);  
}

void interruptA() {
  uint32_t currA = micros();
  if (lastA < currA) {
    double rev = currA - lastA;  // us
    rev = 1.0 / rev;            // rev per us
    rev *= 1000000;             // rev per sec
    rev *= 60;                  // rev per min
    rev /= GEARING;             // account for gear ratio
    rev /= ENCODERMULT;         // account for multiple ticks per rotation
    RPM = rev;
  }
  lastA = currA;
}
void printRPM() {
    Serial.println((int)RPM);
}
