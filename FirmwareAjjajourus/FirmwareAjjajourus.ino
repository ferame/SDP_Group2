#include "SerialCommand.h"
#include "SDPArduino.h"
#include "HMC5883L.h"
#include <Wire.h>
#include "SPI.h"
#include <Arduino.h>

#define RIGHT 3
#define BACK 4
#define LEFT 0
#define KICKERS 1

HMC5883L compass;
int error = 0;

const int  digitalPIN = 3;                 // sets pin 3 as digital pin
const int  analogPIN = A3;                 // sets pin A3 as analog pin

int run = 0;

SerialCommand sCmd;



void loop(){
  sCmd.readSerial();
  
  //For compass
  
    // Retrive the raw values from the compass (not scaled).
    MagnetometerRaw raw = compass.readRawAxis();
    
    int x = raw.XAxis;
    int y = raw.YAxis;
    int z = raw.ZAxis;
    
    x = x * 1.0 + 260;
    y = y * 0.9 - 80;
    
    float angle = atan2(y,x) * 180/PI + 180 - 35;//NORTH
    
    if (angle > 360) {angle = angle - 360;}
    if (angle < 0  ) {angle = angle + 360;}
    printCompassValues(angle);
//    Serial.print("x: ");
//    Serial.print(x);
//    Serial.print("  y: ");
//    Serial.print(y);
//    Serial.print("  z: ");
//    Serial.print(z);
//    Serial.print("  angle: ");
//    Serial.println(angle);
//    // Retrived the scaled values from the compass (scaled to the configured scale).
//    MagnetometerScaled scaled = compass.readScaledAxis();
//    
//    // Values are accessed like so:
//    int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
//  
//    // Calculate heading when the magnetometer is level, then correct for signs of axis.
//    float heading = atan2(scaled.YAxis, scaled.XAxis);
//    
//    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
//    // Find yours here: http://www.magnetic-declination.com/
//    // Mine is: -2��37' which is -2.617 Degrees, or (which we need) -0.0456752665 radians, I will use -0.0457
//    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
//    float declinationAngle = -0.038397;
//    heading += declinationAngle;
//    
//    // Correct for when signs are reversed.
//    if(heading < 0)
//      heading += 2*PI;
//      
//    // Check for wrap due to addition of declination.
//    if(heading > 2*PI)
//      heading -= 2*PI;
//     
//    // Convert radians to degrees for readability.
//    float headingDegrees = heading * 180/M_PI; 
//  
//    // Output the data via the serial port.
//    printCompassValues(headingDegrees);
//  
//    // Normally we would delay the application by 66ms to allow the loop
//    // to run at 15Hz (default bandwidth for the HMC5883L).
//    // However since we have a long serial out (104ms at 9600) we will let
//    // it run at its natural speed.    
  //End of compass
  
  //For IR sensor
  digitalWrite (digitalPIN, HIGH);
  int  digitalReading = digitalRead(analogPIN);
  printIRsensorValues(digitalReading);
  //End of IR sensor
  delay(66);
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

void printIRsensorValues(int objectExists)
{
   Serial.print(objectExists);
   Serial.println(" ball");
}

void printCompassValues(float headingDegrees)
{
   Serial.print(headingDegrees);
   Serial.println(" Degrees");
}

void setup(){
  sCmd.addCommand("f", dontMove); 
  sCmd.addCommand("h", completeHalt); 
  sCmd.addCommand("motor", spinmotor); 
  sCmd.addCommand("r", rationalMotors); 
  sCmd.addCommand("ping", pingMethod); 
  sCmd.addCommand("kick", kicker);
   
  Wire.begin();
  //For compass
  Serial.println("Starting the I2C interface.");
  Wire.begin(); // Start the I2C interface.

  Serial.println("Constructing new HMC5883L");
    
  Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.setScale(1.3); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.getErrorText(error));
  
  Serial.println("Setting measurement mode to continous.");
  error = compass.setMeasurementMode(MEASUREMENT_CONTINUOUS); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.getErrorText(error));
  //End of compass
  
  //For IR sensor
  Serial.begin(9600);
  pinMode (analogPIN, INPUT);
  pinMode (digitalPIN, OUTPUT);              // sets pin 2 as digital input 
  //End of IR sensor
  
  SDPsetup();
  helloWorld();
}

