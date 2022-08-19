//test.ino
#include "src/hand_control/all_data.h"
#include "src/mpu9250_and_bmp/mpu9250_wrapper.h"
#include "src/mpu9250_and_bmp/bmp180.h"
#include "src/sd/SdFat_wrapper.h"
#include "src/gps/gps.h"
#include "src/piezo/piezo.h"
#include "init.h"

#define DATA_UPT_TASK_PERIOD  1  //millis
#define CONTROL_TASK_PERIOD   10 //millis
#define LOG_TASK_PERIOD       100 //millis

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t xBinarySemaphore;

//globals controlcore
float altitude, altitude_;

double latitude, longitude;
String filestr;


void control(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    // update_gps();
    // latitude  = lat();
    // longitude = lng();

    if(INIT_ALL_LOGS){
      String log_data="";
      log_data += String(millis());
      // log_data += SEP + String(roll);
      log_data += SEP + String(pitch);
//      log_data += SEP + String(yaw);
      log_data += SEP + String(altitude);
      // if(INIT_GPS){
      //   log_data += SEP + String(latitude, 9);
      //   log_data += SEP +  String(longitude, 9);
      // }
      log_data += SEP + String(new_pitch_p, 5) + SEP + String(new_roll_p, 5);

      //log string write
      sd_write(filestr, log_data + END_SEP);
     }

    read_control();                    // read IBas data
    stabilization_mode_data_update(pitch, roll);
    control_servos();
    vTaskDelayUntil( &xLastWakeTime, ( CONTROL_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}


void data_update(void* pvParameters){
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(true){
   VectorFloat angles = get_mpu9250_data();
   if (angles.x==angles.x && angles.y==angles.y && angles.z==angles.z){
      if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
        roll = angles.x;
        pitch = angles.y;
        yaw = angles.z;
        altitude = altitude_;
        xSemaphoreGive(xBinarySemaphore);
     }
   }
     if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
       altitude_ = alt();
       xSemaphoreGive(xBinarySemaphore);
     } 
   vTaskDelayUntil( &xLastWakeTime, ( DATA_UPT_TASK_PERIOD / portTICK_RATE_MS ) );
  }
}

void setup() {
  Serial.begin(115200);
  filestr="/data.txt";
  if(INIT_GPS){init_gps(); delay_piezo(1000);}
  init_mpu9250();
  if(INIT_ALL_LOGS){sd_init(); sd_write(filestr, HEADER+"\n"); delay_piezo(100);}
  delay(1000);
  init_bmp(); 
  init_control();
  
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(data_update,"data_update",10000,NULL,10,&Task2,0);
  delay(5000);
  xTaskCreatePinnedToCore(control,"control",10000,NULL,10,&Task1,1); 
  delay(500);
  xSemaphoreGive(xBinarySemaphore); 
}

void loop(){}
