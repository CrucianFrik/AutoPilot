#include "SdFat_wrapper.h"

void setup() {
  sd_init();
  sd_write("a.txt","34vld12");
  }

void loop(){
  
  }
