#include "all_data.h"

TaskHandle_t Task1;
TaskHandle_t Task2;
SemaphoreHandle_t xBinarySemaphore;

#define borders(Max, Min, val) (val > Max)? Max : (val < Min)? Min : val

void control(void* pvParameters){
  while(true){
    read_control();
    if(control_mode_flag == 1){
      hand_control_mode();
    }
    vTaskDelay(1);
  }
}

void data_update(void* pvParameters){
  while(true){
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
