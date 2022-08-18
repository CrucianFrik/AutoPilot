//test.ino
#include "src/hand_control/all_data.h"
#include "src/mpu9250/mpu9250_wrapper.h"
#include "src/sd/SdFat_wrapper.h"
#include "bmp180.h"

#define DATA_UPT_TASK_PERIOD  1 //millis
#define CONTROL_TASK_PERIOD   1 //millis
#define LOG_TASK_PERIOD   10 //millis

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
SemaphoreHandle_t xBinarySemaphore;

String filesname;

//global variables core control/log
float core1_roll=0.0;
float core1_pitch=0.0;
float core1_yaw=0.0;
double core1_bmp_vertical_speed=0.0;
double core1_bmp_flying_alt = 0.0;

double 

void init_tasks(){
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"data_update",10000,NULL,10,&Task2,0);
  delay(5000);
  xTaskCreatePinnedToCore(control,"control",10000,NULL,10,&Task1,1); 
  delay(500);
  xTaskCreatePinnedToCore(logs,"logs",10000,NULL,9,&Task3,1); 
  delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void control(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    
    //...

    //data update 
    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
      core1_pitch=pitch;
      core1_roll=roll;
      core1_yaw=yaw;

      core1_bmp_vertical_speed = bmp_vertical_speed;
      core1_bmp_flying_alt = bmp_flying_alt;
      xSemaphoreGive(xBinarySemaphore);
    } 
    vTaskDelayUntil( &xLastWakeTime, ( CONTROL_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void logs(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  while(true){   
    //log stream creation
    String log_data="";
    log_data+=String(millis())+",";
    log_data+=String(core1_roll)+",";
    log_data+=String(core1_pitch)+",";
    log_data+=String(core1_yaw)+",";
    log_data+=String(core1_bmp_flying_alt)+",";
    log_data+=String(core1_bmp_vertical_speed);
    //log string write
    sd_write(filename, log_data+"\n");
    vTaskDelayUntil( &xLastWakeTime, ( LOG_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void data_update(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    VectorFloat angles =  get_mpu9250_data();
    if (angles.x==angles.x && angles.y==angles.y && angles.z==angles.z){
      if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
        roll = angles.x;
        pitch = angles.y;
        yaw = angles.z;

        update_flying_alt_bmp();
        bmp_vertical_speed = bmp_vy_filter.get_filtered(get_bmp_vertical_speed());
        xSemaphoreGive(xBinarySemaphore);
        }       
   }
   vTaskDelayUntil( &xLastWakeTime, ( DATA_UPT_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void setup() {
  Serial.begin(115200);
  filename="/data.txt";

  init_mpu9250();
  init_sd();
  sd_write(ffilename, HEADER+"\n");

  init_bmp();
  calibrate_bmp();

  init_tasks();
}

void loop(){}
