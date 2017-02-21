#include "SerialCommand.h"
#include "SDPArduino.h"
#include <Wire.h>
#include <Arduino.h>

#define RIGHT 3
#define BACK 4
#define LEFT 0
#define KICKERS 1


int run = 0;

SerialCommand sCmd;



void loop(){
  sCmd.readSerial();
}


void test(){
  run = 1;
}

void dontMove(){
  motorStop(BACK);
  motorStop(LEFT);
  motorStop(RIGHT);
}

void spinmotor(){
  int motor = atoi(sCmd.next());
  int power = atoi(sCmd.next());
  motorForward(motor, power);
}

void motorControl(int motor, int power){
  if(power == 0){
    motorStop(motor);
  } else if(power > 0){
    motorBackward(motor, power);
  } else {
    motorForward(motor, -power);
  }
}

void rationalMotors(){
  int back  = atoi(sCmd.next());
  int left  = atoi(sCmd.next());
  int right = atoi(sCmd.next());
  motorControl(BACK, back);
  motorControl(LEFT, left);
  motorControl(RIGHT, right);
}

void pingMethod(){
  Serial.println("pang");
}

void kicker(){
  int type = atoi(sCmd.next());
  if(type == 0){
    motorStop(KICKERS);
  } else if (type == 1){
    motorForward(KICKERS, 100);
  } else {
    motorBackward(KICKERS, 100);
  }
}

void completeHalt(){
  motorAllStop();
}

void setup(){
  sCmd.addCommand("f", dontMove); 
  sCmd.addCommand("h", completeHalt); 
  sCmd.addCommand("motor", spinmotor); 
  sCmd.addCommand("r", rationalMotors); 
  sCmd.addCommand("ping", pingMethod); 
  sCmd.addCommand("kick", kicker); 
  SDPsetup();
  helloWorld();
}

