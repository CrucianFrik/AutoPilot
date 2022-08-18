#include "gps.h"

void setup()
{
    Serial.begin(115200);
    init_gps();
    calibrate_gps_alt();
}

void loop()
{
    update_gps();
    get_gps_vertical_speed();
}