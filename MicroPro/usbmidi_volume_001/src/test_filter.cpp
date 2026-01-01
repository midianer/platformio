#include <Arduino.h>

static int32_t  output=0;
static int32_t  output1=0;
static int32_t  output2=0;

int32_t run_test_filter_loop(int32_t input) {
  //int32_t input = 100;
  int Qcoeff_sh = 10;
  int32_t Qcoeff = (1<<Qcoeff_sh);
  int32_t Icoeff = (int32_t) (0.05 * Qcoeff);
  output1 = Icoeff * input;
  output2 =  (Qcoeff - Icoeff) * output;
  output = output1 + (output2 >> Qcoeff_sh);
  return(output >> Qcoeff_sh);
}



void run_test_filter_one() {
  int32_t input = 100;
  int Qcoeff_sh = 10;
  int32_t Qcoeff = (1<<Qcoeff_sh);
  int32_t Icoeff = (int32_t) (0.05 * Qcoeff);
  int32_t  output=0;
  int32_t  output1=0;
  int32_t  output2=0;
  Serial.println("****************************************");
  Serial.println(input);
  Serial.println(Qcoeff);
  Serial.println(Icoeff);
  Serial.println(output);
  Serial.println("****************************************");


  for(int i=0; i<50; i++) {
    //Serial.println((Qcoeff - Icoeff) * output);
    //output = ((Icoeff * input) + (((Qcoeff - Icoeff) * output) >> Qcoeff_sh));
    output1 = Icoeff * input;
    output2 =  (Qcoeff - Icoeff) * output;
    output = output1 + (output2 >> Qcoeff_sh);
    Serial.println(output1);
    Serial.println(output2);
    Serial.println(output);
    Serial.println(output >> Qcoeff_sh);
    Serial.println("+++");
  }
}

