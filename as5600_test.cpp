#include <Arduino.h>
#include "AS5600.h"

AS5600L as5600;   //  use default Wire


void setup()
{
    Serial.begin(115200);

    // AS5600 init
    Wire.begin(21,22,800000UL);
    as5600.setAddress(0x36);
    as5600.begin(4);  //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    int b = as5600.isConnected();
    Serial.print("Connect: ");
    Serial.println(b);

}


void loop()
{
    //  Serial.print(millis());
    //  Serial.print("\t");
    Serial.print(as5600.readAngle());
    Serial.print("\t");
    Serial.println(as5600.getCumulativePosition());
    //  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);

    delay(100);
}

