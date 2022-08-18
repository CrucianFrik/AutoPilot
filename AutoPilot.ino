//test.ino
//#include "src/hand_control/all_data.h"
#include "src/mpu9250/mpu9250_wrapper.h"

#define DATA_UPT_TASK_PERIOD  1 //millis
#define CONTROL_TASK_PERIOD   1 //millis
#define LOG_TASK_PERIOD   10 //millis

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
SemaphoreHandle_t xBinarySemaphore;

String filestr;

void control(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    vTaskDelayUntil( &xLastWakeTime, ( CONTROL_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void logs(void* pvParameters){
  if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
    sd_write(filestr, String(roll)+" "+String(pitch)+"\n");
    Serial.println(String(pitch) + " " + String(roll));
    xSemaphoreGive(xBinarySemaphore);
  }    
  vTaskDelayUntil( &xLastWakeTime, ( LOG_TASK_PERIOD / portTICK_RATE_MS ) );
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
        xSemaphoreGive(xBinarySemaphore);
        }       
   }
   vTaskDelayUntil( &xLastWakeTime, ( DATA_UPT_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  filenumber = EEPROM.read(0);
  if ((filenumber<0) or (filenumber>256)){ filenumber=0; }
  filestr="/log_"+String(filenumber)+".txt";

  init_mpu9250();
  sd_init();
  
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
