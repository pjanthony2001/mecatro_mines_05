#include "MUX_TCA.h"

#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> 

//Enables a specific port number
void MUX_TCA::setPort(uint8_t portNumber) {
  if (!mux.setPort(portNumber)) {
    printDebug("MUX FAILED TO SET PORT");
  }
}

//Disables a specific port number
uint8_t MUX_TCA::getPort() {
  return mux.getPort();
}

bool MUX_TCA::init() {
  if (!mux.begin(QWIIC_MUX_DEFAULT_ADDRESS, Wire1)) {
    printDebug("ERROR: Mux initialisation failed!");
    return false;
  }
  return true;
}
