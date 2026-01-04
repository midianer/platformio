#include <Arduino.h>
#include "volume.h"

const int QQcoeff_sh = 12;
#define QQ_ONE (1<<QQcoeff_sh)

//class C_Volume {
//  private:
//    float _a1;
//    float _b1;
//  public:
//    float a1;
//    float b1;
//    C_Volume(float a1, float b1);
//    void get_coeff1(float a1, float b1, float l, int32_t ak[3], int32_t bk[3], int32_t cc[]);
//};

C_Volume::C_Volume(float a1, float b1):
  _a1(a1),
  _b1(b1)
{}

void C_Volume::setup_filter() {
  // Bessel
  const float a1 = 1.3397;
  const float b1 = 0.4889;
  const float a2 = 0.7743;
  const float b2 = 0.3890;
  const int fg = 20;
  const int fs = 1000;
  const float l = 1 / tan(3.14159*fg/fs);
  get_coeff1(_a1, _b1, l, ak1, bk1);
  get_coeff1(a2, b2, l, ak2, bk2);
  //get_coeff(a1, b1, l, &ak1[0], &bk1[1], &bk1[2], cc);
  //get_coeff(a2, b2, l, ak2, bk2, cc);
}


void C_Volume::get_coeff1(float a1, float b1, float l, int32_t ak[3], int32_t bk[3]) {
  ak[0] = QQ_ONE * (1 / (1 + a1 * l + b1 * l * l));
  ak[1] = 0;
  ak[2] = 0;
  bk[0] = QQ_ONE;
  bk[1] = QQ_ONE * (2 * (1 - b1 * l * l) / (1 + a1 * l + b1 * l * l));
  bk[2] = QQ_ONE * ((1 - a1 * l + b1 * l * l) / (1 + a1 * l + b1 * l * l));
  Serial.println("C_Volume: ak[x]");
  Serial.println(ak[0]);
  ak[0] *= 1.0/set_ampl(ak, bk);
  Serial.println("C_Volume: ak[x]");
  Serial.println(ak[0]);
//  cc[0]=1234;
//  cc[5]=5678;
  return;
}

float C_Volume::set_ampl(int32_t ak[3], int32_t bk[3]) {
  float a0 = 0.0f;
  float b0 = 0.0f;
  for(int i=0; i<3; i++) {
    a0 += ak[i];
    b0 += bk[i];
    Serial.println(ak[i]);
    Serial.println(bk[i]);
  }
  Serial.println("C_Volume: +++set_ampl+++");
  Serial.println(a0);
  Serial.println(b0);
  return(a0 / b0);
}

int32_t C_Volume::run_filter_one(int32_t input) {
  static int32_t yn=0;
  static int32_t zz[3];
  Serial.println("----run_filter_one------");
  Serial.println(ak1[0]);
  Serial.println(bk1[0]);
  Serial.println(bk1[1]);
  Serial.println(bk1[2]);
  for(int i=0; i<100; i++) {
    yn = ak1[0] * input + (zz[0] >> QQcoeff_sh);
    zz[1] = zz[2] - (bk1[1] *  yn);
    zz[2] = - (bk1[2] *  yn);
    zz[0] = zz[1];
    zz[1] = zz[2];
    Serial.println("++++++++");
    Serial.println(yn);
    Serial.println(zz[0]);
    Serial.println(zz[1]);
    Serial.println(yn >> QQcoeff_sh);
  }
  return(yn);
}

int32_t C_Volume::run_filter_loop(int32_t input) {
  static int32_t yn=0, ynb=0;
  static int32_t zz[3];
  static int32_t zzb[3];
  yn = ak1[0] * input + (zz[0] >> QQcoeff_sh);
  zz[1] = zz[2] - (bk1[1] *  yn);
  zz[2] = - (bk1[2] *  yn);
  zz[0] = zz[1];
  zz[1] = zz[2];
  ynb = ak2[0] * ((yn + (QQ_ONE >> 2)) >> QQcoeff_sh) + (zzb[0] >> QQcoeff_sh);
  zzb[1] = zzb[2] - (bk2[1] *  ynb);
  zzb[2] = - (bk2[2] *  ynb);
  zzb[0] = zzb[1];
  zzb[1] = zzb[2];
  return((ynb + (QQ_ONE >> 2)) >>  QQcoeff_sh );
}

