/* Demo code for reading the encoders of the robot.

  Note: this code requires the following libraries (install them through the library manager):
     - SparkFun I2C Mux Arduino Library
     - AS5600 library
*/

// Include the current library
#include "MecatroUtils.h"

// Include the SensorBar library (for Line Follower) and Sparkfun I2C Mux (for multiplexer)
#include "sensorbar.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// Header for I2C communication
#include "Wire.h"

// Define the control loop period, in ms.
#define CONTROL_LOOP_PERIOD 5

QWIICMUX multiplexer;
// Wire1 is used for the Qwiic connector on this Arduino board. 
//SensorBar mySensorBar(SX1509_ADDRESS);
SensorBar mySensorBar(0x3E);

// Define the port numbers
#define SENSORBAR_PIN 4

void setup()
{
  // Setup serial communication with the PC - for debugging and logging.
  Serial.begin(230400);
  // Start I2C communication on the QWIIC port
  Wire1.begin();

  // Init multiplexer. Its I2C address is 0x70, and we communicate via the QWIIC port (Wire1).
  if (!multiplexer.begin(0x70, Wire1))
  {
    Serial.println("Error: I2C multiplexer not found. Check wiring.");
  }
  else
  {
    bool isInit = true;

    //Command for the IR to run all the time
    mySensorBar.clearBarStrobe();
    //Default dark on light
    mySensorBar.clearInvertBits();
    //Other option: light line on dark
    //mySensorBar.setInvertBits();
    
    //Don't forget to call .begin() to get the bar ready.  This configures HW.
    multiplexer.setPort(SENSORBAR_PIN);
    uint8_t returnStatus = mySensorBar.begin();
    if(!returnStatus)
    {
      Serial.println("sx1509 IC communication FAILED!");
      isInit = false;
    }


    if (isInit)
    {
      // Set I2C clock speed to 400kHz (fast mode). Do it after initializing everyone so that the clock speed
      // is not reset by a particular device. 
      Wire1.setClock(400000);
      // Configure motor control and feedback loop call.
      mecatro::configureArduino(CONTROL_LOOP_PERIOD);
    }
  }
}


void loop()
{
  // Don't forget to call this, otherwise nothing will happen !
  // This function never returns, put all your code inside mecatro::controlLoop.
  mecatro::run();
}


// This function is called periodically, every CONTROL_LOOP_PERIOD ms.
// Put all your code here.
void mecatro::controlLoop()
{
  // Set multiplexer to talk to line follower
  multiplexer.setPort(SENSORBAR_PIN);
  Serial.print("Line Folllower: ");
  Serial.print(mySensorBar.getPosition());
  Serial.println();

  // Keep the motor off, i.e. at 0 duty cycle (1 is full forward (i.e. trigonometric sense in the motor frame), -1 full reverse)
  mecatro::setMotorDutyCycle(0.0, 0.0);
}