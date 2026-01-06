#include <Arduino.h>

#ifndef __volume_h
#define __volume_h

//#define QQ_ONE (1<<QQcoeff_sh)

class C_Volume {
  private:
    float _a1;
    float _b1;
    String _id;
    int32_t ak1[3];
    int32_t bk1[3];
    int32_t ak2[3];
    int32_t bk2[3];
    int32_t yn=0, ynb=0;
    int32_t zz[3];
    int32_t zzb[3];
    float _sum_a = 0.0f;
    float _sum_b = 0.0f;

  public:
    //float a1;
    //float b1;
    C_Volume(float a1, float b1, String id);
    void setup_filter();
    void get_coeff1(float a1, float b1, float l, int32_t ak[3], int32_t bk[3]);
    float set_ampl(int32_t ak[3], int32_t bk[3]);
    int32_t run_filter_one(int32_t input);
    int32_t run_filter_loop(int32_t input);
    //void set_adc_cb(int32_t (*func_adc)(void));
    //int32_t (*get_adc)(void);
    //std::function<int32_t(void)> get_adc;
};

#endif
