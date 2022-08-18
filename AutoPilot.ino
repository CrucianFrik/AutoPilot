//test.ino
#include "src/hand_control/all_data.h"
#include "src/mpu9250/mpu9250_wrapper.h"

uint8_t mpuIntStatus,devStatus,fifoBuffer[64];
uint16_t packetSize,fifoCount,mag[3];
float f_mag[3],d_mag[4];
Quaternion q,q_mag;

#define DATA_UPT_TASK_PERIOD  1 //millis
#define CONTROL_TASK_PERIOD   1 //millis

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

long data_reading_timing=0;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t xBinarySemaphore;

void control(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    Serial.println("control");
    
    read_control();
//    pitch_ctrl_effect_2 = pitch_pid.ctrl(((servo_control[8] - 1500.0) * (45.0/500.0)), pitch);                         
//    roll_ctrl_effect_2  = roll_pid.ctrl(((servo_control[9] - 1500.0) * (45.0/500.0)), roll);
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
    vTaskDelayUntil( &xLastWakeTime, ( CONTROL_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void data_update(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    Serial.println("data_update");
    get_mpu9250_data();
    vTaskDelayUntil( &xLastWakeTime, ( DATA_UPT_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void setup() {
  Serial.begin(115200);

  init_control(); 
  init_mpu9250();
  
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"data_update",10000,NULL,10,&Task2,0);
  delay(5000);
  xTaskCreatePinnedToCore(control,"control",10000,NULL,10,&Task1,1); 
  delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
