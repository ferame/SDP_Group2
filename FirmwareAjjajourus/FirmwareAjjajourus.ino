#include "SerialCommand.h"
#include "SDPArduino.h"
#include "Accelerometer_Compass_LSM303D.h"
#include <Wire.h>
#include "SPI.h"
#include <Arduino.h>

#define RIGHT 3
#define BACK 4
#define LEFT 0
#define KICKERS 1

int mag[3];  // raw magnetometer values stored here
float heading;

int run = 0;

SerialCommand sCmd;



void loop(){
  sCmd.readSerial();
  
  //For compass
  Serial.println("\r\n**************");
  while(!Lsm303d.isMagReady());// wait for the magnetometer readings to be ready
  Lsm303d.getMag(mag);  // get the magnetometer values, store them in mag
  heading = Lsm303d.getHeading(mag);
	
  printCompassValues();
  delay(200);
  //End of compass
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

void printCompassValues()
{
	/* print both the level, and tilt-compensated headings below to compare */
        Serial.println("The clockwise angle between the magnetic north and x-axis: ");
	Serial.print(heading, 3); // this only works if the sensor is level
	Serial.println(" degrees");
}

void setup(){
  sCmd.addCommand("f", dontMove); 
  sCmd.addCommand("h", completeHalt); 
  sCmd.addCommand("motor", spinmotor); 
  sCmd.addCommand("r", rationalMotors); 
  sCmd.addCommand("ping", pingMethod); 
  sCmd.addCommand("kick", kicker);
 
  //For compass
  char rtn = 0;
  Serial.begin(9600);
  Serial.println("\r\npower on");
  rtn = Lsm303d.initI2C();
  if(rtn != 0)  // Initialize the LSM303, using a SCALE full-scale range
        {
		Serial.println("\r\nLSM303D is not found");
		while(1);
	}
	else
	{
		Serial.println("\r\nLSM303D is found");
	}
  //End of compass
  
  SDPsetup();
  helloWorld();
}

