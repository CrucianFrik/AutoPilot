#include "all_data.h"

TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t xBinarySemaphore;
double pitch = 0.0, roll = 0.0; 
#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

void control(void* pvParameters){
  while(true){
    read_control();
    // >>>> UPDATE CONTROL EFFECTS
    pitch_ctrl_effect_2 = pitch_pid.ctrl(((servo_control[8] - 1500.0) * (45.0/500.0)), pitch);                         
    roll_ctrl_effect_2  = roll_pid.ctrl(-((servo_control[9] - 1500.0) * (45.0/500.0)), roll);
    // <<<< UPDATE CONTROL EFFECTS
    if(control_mode_flag == 1){
      hand_control_mode();
    }
    else{
      if(control_mode_flag == 2){
        pitch_ctrl_effect = pitch_ctrl_effect_2;
        roll_ctrl_effect  = roll_ctrl_effect_2;
      }
      roll_ctrl_effect  = borders(1, -1, roll_ctrl_effect);
      pitch_ctrl_effect = borders(1, -1, pitch_ctrl_effect);
      
      eilerons_ctrl = 500*roll_ctrl_effect + 1500;
      pgo_l_ctrl = -0.7 * 500*pitch_ctrl_effect + 1500;
      pgo_r_ctrl = 0.7 * 500*(pitch_ctrl_effect - 0.3) + 1500;

      eilerons_ctrl = -borders(2000, 1000, eilerons_ctrl);
      pgo_l_ctrl    = borders(2000, 1000, pgo_l_ctrl);
      pgo_r_ctrl    = borders(2000, 1000, pgo_r_ctrl);
      
      engine.write(servo_control[2]);
      eileron_l.write(eilerons_ctrl);
      eileron_r.write(eilerons_ctrl);
      pgo_l.write(pgo_l_ctrl);
      pgo_r.write(pgo_r_ctrl);
    }
    vTaskDelay(1);
  }
}

void data_update(void* pvParameters){
  while(true){
    pitch = ((servo_control[10] - 1500.0) * (45.0/500.0));
    roll  = ((servo_control[11] - 1500.0) * (45.0/500.0));
  vTaskDelay(1);
  }
}
void setup() {
  Serial.begin(115200);
  init_control(); 
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"Task2",10000,NULL,10,&Task2,0);
  delay(5000);
  xTaskCreatePinnedToCore(control,"Task1",10000,NULL,10,&Task1,1); 
  delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
