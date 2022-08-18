//test.ino
#include "src/hand_control/all_data.h"
#include "src/mpu9250/mpu9250_wrapper.h"

#define DATA_UPT_TASK_PERIOD  1 //millis
#define CONTROL_TASK_PERIOD   1 //millis

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t xBinarySemaphore;

void control(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    Serial.println("control");
    vTaskDelayUntil( &xLastWakeTime, ( CONTROL_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void data_update(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    Serial.println("data_update");
    VectorFloat angles = get_mpu9250_data();
    if (angles.x==angles.x && angles.y==angles.y && angles.z==angles.z){
      if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
            roll = angles.x;
            pitch = angles.y;
            yaw = angles.z;
            xSemaphoreGive(xBinarySemaphore);
        }       
    }
    vTaskDelayUntil( &xLastWakeTime, ( DATA_UPT_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void setup() {
  Serial.begin(115200);

  init_mpu9250();
  
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"data_update",10000,NULL,10,&Task2,0);
  delay(5000);
  xTaskCreatePinnedToCore(control,"control",10000,NULL,10,&Task1,1); 
  delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
