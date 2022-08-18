//test.ino
#include "all_data.h"

double my_pitch = 0.0, my_roll = 0.0;
void setup(){
   init_control(); 
}

void loop(){
  read_control();                    // read IBas data
  stabilization_mode_data_update(my_pitch, my_roll);
  control_servos();
}
