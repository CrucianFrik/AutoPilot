//test.ino
//#include "src/hand_control/all_data.h"
#include "src/mpu9250_and_bmp/mpu9250_wrapper.h"
#include "src/mpu9250_and_bmp/bmp180.h"
#include "src/sd/SdFat_wrapper.h"

#define DATA_UPT_TASK_PERIOD  1 //millis
#define CONTROL_TASK_PERIOD   1 //millis
#define LOG_TASK_PERIOD   10 //millis

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
SemaphoreHandle_t xBinarySemaphore;

//globals control/log core
float roll_core1=0.0;
float pitch_core1=0.0;
float yaw_core1=0.0;
float altitude_core1;
float altitude;

String filestr;

void control(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    vTaskDelayUntil( &xLastWakeTime, ( CONTROL_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void logs(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  float current_roll=0.0,current_pitch=0.0,current_yaw=0.0;
  while(true){
    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
      pitch_core1=pitch;
      roll_core1=roll;
      yaw_core1=yaw;
      altitude_core1 = altitude;
      xSemaphoreGive(xBinarySemaphore);
    }    
    //log stream creation
    String log_data="";
    log_data+=String(millis())+",";
    log_data+=String(roll_core1)+",";
    log_data+=String(pitch_core1)+",";
    log_data+=String(yaw_core1)+",";
    log_data+=String(altitude_core1);
    //log string write
    sd_write(filestr, log_data+"\n");
    vTaskDelayUntil( &xLastWakeTime, ( LOG_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void data_update(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    VectorFloat angles =  get_mpu9250_data();
    float altitude_ = alt();
    if (angles.x==angles.x && angles.y==angles.y && angles.z==angles.z){
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
          roll = angles.x;
          pitch = angles.y;
          yaw = angles.z;
          altitude = altitude_;
          xSemaphoreGive(xBinarySemaphore);
          }       
    }
   vTaskDelayUntil( &xLastWakeTime, ( DATA_UPT_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void setup() {
  Serial.begin(115200);
  filestr="/data.txt";

  init_mpu9250();
  sd_init();
  sd_write(filestr, HEADER+"\n");

  init_bmp();
  
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"data_update",10000,NULL,10,&Task2,0);
  delay(5000);
  xTaskCreatePinnedToCore(control,"control",10000,NULL,10,&Task1,1); 
  delay(500);
  xTaskCreatePinnedToCore(logs,"logs",10000,NULL,9,&Task3,1); 
  delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
