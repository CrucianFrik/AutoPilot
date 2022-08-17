#include "mpu9250_wrapper.h"



void setup() {
    Serial.begin(115200);
    init_mpu9250();
}



void loop() {
    get_mpu9250_data();
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(yaw);
    
}
