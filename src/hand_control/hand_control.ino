//test.ino
#include "all_data.h"


#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val




double roll,pitch,yaw;
double curr_roll, curr_pitch, curr_yaw;

void setup(){
   init_control(); 
}

void loop(){
   read_control();
  //  pitch_ctrl_effect_2 = pitch_pid.ctrl(((servo_control[8] - 1500.0) * (45.0/500.0)), pitch);                         
  //  roll_ctrl_effect_2  = roll_pid.ctrl(((servo_control[9] - 1500.0) * (45.0/500.0)), roll);
  if(control_mode_flag == 1){
    hand_control_mode();
  }
  else{
    if(control_mode_flag == 2){
//        pitch_ctrl_effect = pitch_ctrl_effect_2;
//        roll_ctrl_effect  = roll_ctrl_effect_2;
    }
    roll_ctrl_effect  = borders(1, -1, roll_ctrl_effect);
    pitch_ctrl_effect = borders(1, -1, pitch_ctrl_effect);
    
    eilerons_ctrl = -500*roll_ctrl_effect + 1500;
    pgo_l_ctrl = -0.7 * 500*(pitch_ctrl_effect - 0.06) + 1500;
    pgo_r_ctrl = 0.7 * 500*(pitch_ctrl_effect - 0.5) + 1500;

    eilerons_ctrl = borders(2000, 1000, eilerons_ctrl);
    pgo_l_ctrl    = borders(2000, 1000, pgo_l_ctrl);
    pgo_r_ctrl    = borders(2000, 1000, pgo_r_ctrl);
    
    engine.write(servo_control[2]);
    eileron_l.write(eilerons_ctrl);
    eileron_r.write(eilerons_ctrl);
    pgo_l.write(pgo_l_ctrl);
    pgo_r.write(pgo_r_ctrl);
  }
}
