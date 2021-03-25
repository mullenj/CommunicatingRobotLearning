#include <Arduino.h>
#include <analogWrite.h>
char k = 'A';
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

// These let us convert ticks-to-RPM
#define GEARING     20
#define ENCODERMULT 12

BluetoothSerial SerialBT;
void setup() {
  Serial.begin(115200);
  pinMode(vibrotactors[0], OUTPUT);
  pinMode(vibrotactors[1], OUTPUT);
  pinMode(vibrotactors[2], OUTPUT);
  pinMode(vibrotactors[3], OUTPUT);
  pinMode(vibrotactors[4], OUTPUT);
  pinMode(vibrotactors[5], OUTPUT);
  pinMode(driver_1, OUTPUT);
  pinMode(driver_2, OUTPUT);
  SerialBT.begin("haptic_device"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  attachInterrupt(ENCODER_A, interruptA, RISING);
}
void loop() {
  if (SerialBT.available()) {
    k = SerialBT.read();
    if( k == 'H' ){
      analogWrite(vibrotactors[0],300);
      delay(1000);
            analogWrite(vibrotactors[1],300);
      delay(1000);
            analogWrite(vibrotactors[2],300);
      delay(1000);
            analogWrite(vibrotactors[3],300);
      delay(1000);
            analogWrite(vibrotactors[4],300);
      delay(1000);
            analogWrite(vibrotactors[5],300);
      delay(1000);

      
    }
    else if( k == 'S' ){
      analogWrite(vibrotactors[1],300); 
    }
    else if( k == 'P' ){
      analogWrite(vibrotactors[2],300);  
    }
    else if( k == 'Y' ){
      analogWrite(vibrotactors[3],300); 
    }
    else if( k == 'U' ){
      analogWrite(vibrotactors[4],300); 
    }
    else if( k == 'T' ){
      analogWrite(vibrotactors[5],300); 
    }
    else if( k == 'F' ){
      analogWrite(driver_1, 300);
      analogWrite(driver_2, 0);
    }
    else if( k == 'R' ){
      analogWrite(driver_1, 0);
      analogWrite(driver_2, 300);
    }
    else if( k == 'K' ){
      analogWrite(vibrotactors[0],300);
      analogWrite(vibrotactors[1],300);
      analogWrite(vibrotactors[2],300);
    }
    else if( k == 'J' ){
      analogWrite(vibrotactors[3],300);
      analogWrite(vibrotactors[4],300);
      analogWrite(vibrotactors[5],300);
    }
    else if( k == 'L' ){
      analogWrite(vibrotactors[0],0); 
      analogWrite(vibrotactors[1],0); 
      analogWrite(vibrotactors[2],0); 
      analogWrite(vibrotactors[3],0);  
      analogWrite(vibrotactors[4],0); 
      analogWrite(vibrotactors[5],0);
      analogWrite(driver_1,0);  
      analogWrite(driver_2,0);       
    }
  }
  printRPM();
  delay(100);
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
