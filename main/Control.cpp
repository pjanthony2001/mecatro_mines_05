#include "Control.h"

using namespace BLA;

void ControlState::updateControlState(DEVICE_DATA& device_data) {
    if (!device_data.isValid) {
        printDebug("CONTROL: DEVICE SAMPLING NOT VALID");
    }

    

}
