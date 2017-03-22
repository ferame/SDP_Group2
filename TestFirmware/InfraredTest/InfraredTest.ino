#include "SerialCommand.h"
#include <Wire.h>
#include "SPI.h"
#include <Arduino.h>

int error = 0;

const int  digitalPIN = 3;                 // sets pin 3 as digital pin
const int  analogPIN = A3;                 // sets pin A3 as analog pin

int run = 0;

SerialCommand sCmd;



void loop(){
  sCmd.readSerial();
  
  //For IR sensor
  digitalWrite (digitalPIN, HIGH);
  int  digitalReading = digitalRead(analogPIN);
  //End of IR sensor
  
  printSensorsValues(digitalReading);
  delay(66);
}

void pingMethod(){
  Serial.println("pang");
}

void printSensorsValues(int objectExists){
  Serial.println(objectExists);
}

void setup(){
  //For IR sensor
  Serial.begin(9600);
  pinMode (analogPIN, INPUT);
  pinMode (digitalPIN, OUTPUT);              // sets pin 2 as digital input 
  //End of IR sensor
}

