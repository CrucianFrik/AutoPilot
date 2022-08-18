#include "bmp180.h"

void setup()
{
    Serial.begin(115200);
    init_bmp();
    calibrate_bmp();
}

void loop()
{
    update_flying_alt_bmp();
    bmp_vertical_speed = bmp_vy_filter.get_filtered(get_bmp_vertical_speed());
    Serial.println(String(bmp_vertical_speed) + " " + String(bmp_flying_alt));
}
