#include <Arduino.h>

//#define QQ_ONE (1<<QQcoeff_sh)

class C_Volume {
  private:
    float _a1;
    float _b1;
    int32_t ak1[3];
    int32_t bk1[3];
    int32_t ak2[3];
    int32_t bk2[3];
  public:
    //float a1;
    //float b1;
    C_Volume(float a1, float b1);
    void setup_filter();
    void get_coeff1(float a1, float b1, float l, int32_t ak[3], int32_t bk[3]);
    float set_ampl(int32_t ak[3], int32_t bk[3]);
    int32_t run_filter_one(int32_t input);
    int32_t run_filter_loop(int32_t input);
};

