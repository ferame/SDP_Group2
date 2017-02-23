#include "SerialCommand.h"
#include "SDPArduino.h"
<<<<<<< HEAD
#include "HMC5883L.h"
=======
#include "Accelerometer_Compass_LSM303D.h"
>>>>>>> a3675865ff209df6fefbbcdb44e2a571b11c7edf
#include <Wire.h>
#include "SPI.h"
#include <Arduino.h>

#define RIGHT 3
#define BACK 4
#define LEFT 0
#define KICKERS 1

<<<<<<< HEAD
HMC5883L compass;
int error = 0;

// int mag[3];  // raw magnetometer values stored here
// float heading;
=======
int mag[3];  // raw magnetometer values stored here
float heading;
>>>>>>> a3675865ff209df6fefbbcdb44e2a571b11c7edf

int run = 0;

SerialCommand sCmd;



void loop(){
  sCmd.readSerial();
  
  //For compass
<<<<<<< HEAD
  
    // Retrive the raw values from the compass (not scaled).
    MagnetometerRaw raw = compass.readRawAxis();
    // Retrived the scaled values from the compass (scaled to the configured scale).
    MagnetometerScaled scaled = compass.readScaledAxis();
    
    // Values are accessed like so:
    int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
  
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(scaled.YAxis, scaled.XAxis);
    
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -2��37' which is -2.617 Degrees, or (which we need) -0.0456752665 radians, I will use -0.0457
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = -0.038397;
    heading += declinationAngle;
    
    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
      
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
     
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI; 
  
    // Output the data via the serial port.
    printCompassValues(raw, scaled, heading, headingDegrees);
  
    // Normally we would delay the application by 66ms to allow the loop
    // to run at 15Hz (default bandwidth for the HMC5883L).
    // However since we have a long serial out (104ms at 9600) we will let
    // it run at its natural speed.
    delay(66);//of course it can be delayed longer
    
=======
  Serial.println("\r\n**************");
  while(!Lsm303d.isMagReady());// wait for the magnetometer readings to be ready
  Lsm303d.getMag(mag);  // get the magnetometer values, store them in mag
  heading = Lsm303d.getHeading(mag);
	
  printCompassValues();
  delay(200);
>>>>>>> a3675865ff209df6fefbbcdb44e2a571b11c7edf
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

<<<<<<< HEAD
void printCompassValues(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
   Serial.print("Raw:\t");
   Serial.print(raw.XAxis);
   Serial.print("   ");   
   Serial.print(raw.YAxis);
   Serial.print("\t");
//   Serial.print("   ");   
//   Serial.print(raw.ZAxis);
//   Serial.print("   \tScaled:\t");
//   
//   Serial.print(scaled.XAxis);
//   Serial.print("   ");   
//   Serial.print(scaled.YAxis);
//   Serial.print("   ");   
//   Serial.print(scaled.ZAxis);
//
//   Serial.print("   \tHeading:\t");
//   Serial.print(heading);
//   Serial.print(" Radians   \t");
   Serial.print(headingDegrees);
   Serial.println(" Degrees   \t");
=======
void printCompassValues()
{
	/* print both the level, and tilt-compensated headings below to compare */
        Serial.println("The clockwise angle between the magnetic north and x-axis: ");
	Serial.print(heading, 3); // this only works if the sensor is level
	Serial.println(" degrees");
>>>>>>> a3675865ff209df6fefbbcdb44e2a571b11c7edf
}

void setup(){
  sCmd.addCommand("f", dontMove); 
  sCmd.addCommand("h", completeHalt); 
  sCmd.addCommand("motor", spinmotor); 
  sCmd.addCommand("r", rationalMotors); 
  sCmd.addCommand("ping", pingMethod); 
  sCmd.addCommand("kick", kicker);
<<<<<<< HEAD
   
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
=======
 
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
>>>>>>> a3675865ff209df6fefbbcdb44e2a571b11c7edf
  //End of compass
  
  SDPsetup();
  helloWorld();
}

