#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MIDIUSB.h>
#include "volume.h"
#include "debounce.h"

/*******************************
**    usbmidi_volume_001      **
**    V1.0                    **
*******************************/

#define I2C_ADDR    0x27 // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

#define D2 SDA
#define D3 SCL
#define D8 8
#define D9 9
#define D10 10

//#define DEBUG_LOOP


const int QQcoeff_sh = 12;
#define QQ_ONE (1<<QQcoeff_sh)

const float C_a1 = 1.3397;
const float C_b1 = 0.4889;

LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
//LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned long currentMillis;
#ifdef DEBUG_LOOP
  const long interval = 200;
#else
  const long interval = 2;
#endif  
volatile uint16_t adc_value[3];

int adc_mux_arr[3] =
{ ((1 << MUX2) | (1 << MUX1) | (1 << MUX0) | (1 <<REFS0)),  // ADC(2) 
  ((1 << MUX2) | (1 << MUX1) | (0 << MUX0) | (1 <<REFS0)),  // ADC(1)
  ((1 << MUX2) | (0 << MUX1) | (1 << MUX0) | (1 <<REFS0))   // ADC(0)
};

// Creat a set of new characters
byte smiley[8] = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b10001,
  0b01110,
  0b00000
};

byte armsUp[8] = {
  0b00100,
  0b01010,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
  0b00100,
  0b01010
};

byte frownie[8] = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b00000,
  0b01110,
  0b10001
};

void controlChange(byte channel, byte control, byte value);
void setupadc();
void get_coeff(float a1, float b1, float l, int32_t *al0, int32_t *be0, int32_t *be1, int32_t cc[]);
void get_coeff1(float a1, float b1, float l, int32_t ak[3], int32_t bk[3], int32_t cc[]);
void setup_filter();
int32_t get_adc(int adc_num);
int32_t run_filter_loop(int32_t input);
int32_t run_filter_one(int32_t input);
int32_t run_test_filter_loop(int32_t input);
void run_test_filter_one();
float set_ampl(int32_t ak[3], int32_t bk[3]);

int32_t ak1[3];
int32_t bk1[3] = {QQ_ONE,0,0};;
int32_t ak2[3];
int32_t bk2[3];
int32_t cc[3];
volatile uint16_t isr_cnt;

C_Volume vol1(C_a1, C_b1, "VOL1");
C_Volume vol2(C_a1, C_b1, "VOL2");

ClDebounce redLED(4, LED_BUILTIN_RX, 0);
ClDebounce greenLED(5, LED_BUILTIN_TX, 1);


void setup() {

  Serial.begin(115200);
  lcd.begin (16,2);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home();
  lcd.clear();
  lcd.print("USBMidi Volume 1");
  lcd.setCursor ( 0, 1 );
  lcd.print("V1.0");

  delay(5000);

  Serial.println("prg: USBMidi Volume 1");

  //Use predefined PINS consts

  //Wire.begin(D2, D3);


  lcd.home();
  lcd.clear();
  lcd.print("Startup finished");

  lcd.createChar (0, smiley);    // load character to the LCD
  lcd.createChar (1, armsUp);    // load character to the LCD
  lcd.createChar (2, frownie);   // load character to the LCD
  delay(3000);

  pinMode(LED_BUILTIN_RX, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(D9, OUTPUT);
  pinMode(D10, OUTPUT);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN_RX, LOW);
  digitalWrite(LED_BUILTIN_RX, HIGH);
  setupadc();
  setup_filter();
  Serial.println("ak, bk, set_ampl");
  for(int i=0; i<3; i++)
    Serial.println(ak1[i]);
  for(int i=0; i<3; i++)
    Serial.println(bk1[i]);
  //Serial.println(set_ampl(ak1, bk1));
  //run_test_filter_one();
  run_filter_one(100);

  vol1.setup_filter();
  vol2.setup_filter();
  vol1.run_filter_one(50);
  get_adc(0);
  Serial.println("****get_adc****");
  Serial.println(get_adc(0) >> 3);
  //vol1.set_adc_cb(get_adc);
  //Serial.println("****get_adc****");
  //Serial.println(vol1.get_adc() >> 3);
  //C_Volume cv1(C_a1, C_b1);
  //cv1.setup_filter();
  //C_Volume cv2(C_a1, C_b1);
  //cv2.setup_filter();
  //C_Volume cv3(C_a1, C_b1);
  //cv3.setup_filter();
}

 int xk, xk1;


void loop()
{ 
  static int16_t adc_val_old[3];
  static int16_t adc_sh[3];
  static int16_t adc_filt[3];
  
  digitalWrite(LED_BUILTIN_RX, HIGH);
//  xk = (a * 16 + 16 * 15 * xk) >> 8;
//  if(xk>>8 != xk1) {
//    controlChange(0, 0x7, xk & 0x7f);
//    MidiUSB.flush();
//  }
//  xk1 = xk>>8;
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    digitalWrite(D10, HIGH);
    // save the last time you blinked the LED
    //previousMillis = currentMillis;  //volumeChange(0, 0x7f);
    previousMillis += interval;  //volumeChange(0, 0x7f);
    adc_sh[0] = adc_value[0] >> 3;
    adc_sh[1] = adc_value[1] >> 3;
    adc_sh[2] = adc_value[2] >> 3;
    #ifdef DEBUG_LOOP 
      Serial.println("adc_sh");
      Serial.println(adc_sh[0]);
      Serial.println(adc_sh[1]);
      Serial.println(adc_sh[2]);
    #endif
    isr_cnt = 2;
    ADMUX = adc_mux_arr[isr_cnt];
    ADCSRA |= (1 <<ADSC); // Optional: Neue Konvertierung starten
    //adc_filt = analogRead(A0);
    //adc_filt = run_test_filter_loop(a);
    adc_filt[0] = vol1.run_filter_loop(adc_sh[0]);
    adc_filt[1] = vol2.run_filter_loop(adc_sh[1]);
    #ifdef DEBUG_LOOP 
    Serial.println("adc_filt");
    Serial.println(adc_filt[0]);
    Serial.println(adc_filt[1]);
    Serial.println(adc_filt[2]);
    #endif
    if (adc_filt[0] != adc_val_old[0]) {
      digitalWrite(D8, HIGH);
      controlChange(0, 0x7, max(adc_filt[0],0));
      digitalWrite(D8, LOW);
      adc_val_old[0]=adc_filt[0];
      //Serial.println(adc_value);
      //Serial.println(adc_filt);
    }
    if (adc_filt[1] != adc_val_old[1]) {
      digitalWrite(D8, HIGH);
      controlChange(1, 0x7, max(adc_filt[1],0));
      digitalWrite(D8, LOW);
      adc_val_old[1]=adc_filt[1];
      //Serial.println(adc_value);
      //Serial.println(adc_filt);
    }
    //MidiUSB.flush();
    //Serial.println(ADMUX, HEX);
    //Serial.println(ADCSRA, HEX);
    //Serial.println(ADCSRB, HEX);
    //Serial.println(a, HEX);
    digitalWrite(D10, LOW);
  }
  digitalWrite(LED_BUILTIN_RX, LOW);
  greenLED.myFunction(2000);
  redLED.myFunction(1000);

}


void loop1()
{
  // Do a little animation by writing to the same location
  lcd.setCursor ( 8, 1 );
  lcd.print (char(2));
  delay (500);
  lcd.setCursor ( 8, 1 );
  lcd.print ( char(0));
  delay (500);
  lcd.setCursor ( 8, 1 );
  lcd.print ( char(1));
  delay (500);

}

void controlChange(byte channel, byte control, byte value) {

  midiEventPacket_t event = {0x0B, (byte)(0xB0 | channel), control, value};

  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}


// ADC-Interrupt Service Routine
ISR(ADC_vect) {
  digitalWrite(D9, LOW);
  digitalWrite(D9, HIGH);
  //adc_value = analogRead(A0);
  adc_value[isr_cnt] = ADC; // Wert aus dem ADC-Register lesen
  //digitalWrite(D9, adc_value[0]>>2);
  // Hier Wert verarbeiten oder in eine Queue legen
  if(isr_cnt > 0) {
    isr_cnt--;
    ADMUX = adc_mux_arr[isr_cnt];
    ADCSRA |= (1 <<ADSC); // Neue Konvertierung starten
  }
  digitalWrite(D9, LOW);
  digitalWrite(D9, HIGH);
}

int32_t get_adc(int adc_num) {
  return adc_value[adc_num];
}

void setupadc() {
  // ... ADC-Pins als Input konfigurieren
  ADCSRA = 0; // Clear ADCSRA
  ADMUX = ((1 << MUX2) | (1 << MUX1) | (1 << MUX0) | (1 <<REFS0)); // Referenzspannung einstellen
  ADCSRA |= ((1 <<ADEN) | (1 <<ADIE) | (1 <<ADPS2) | (1 <<ADPS1) | (0 <<ADPS0)) ; // ADC aktivieren, Interrupt aktivieren, Prescaler setzen
//  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE);
  // ADMUX (Kanal wählen, z.B. A0)
  // Oder Timer-Trigger konfigurieren, z.B. ADCSRA |= (1 <<ADATE); ADTS Bits setzen
  DIDR0 = (1 << ADC0D);
  sei(); // Globale Interrupts aktivieren
  ADCSRA |= (1 <<ADSC); // Erste Konvertierung starten
}

void setup_filter() {
  // Bessel
  const float a1 = 1.3397;
  const float b1 = 0.4889;
  const float a2 = 0.7743;
  const float b2 = 0.3890;
  const int fg = 20;
  const int fs = 1000;
  const float l = 1 / tan(3.14159*fg/fs);
  get_coeff1(a1, b1, l, ak1, bk1, cc);
  get_coeff1(a2, b2, l, ak2, bk2, cc);
  //get_coeff(a1, b1, l, &ak1[0], &bk1[1], &bk1[2], cc);
  //get_coeff(a2, b2, l, ak2, bk2, cc);
  Serial.println(cc[0]);
}

void get_coeff(float a1, float b1, float l, int32_t *al0, int32_t *be0, int32_t *be1, int32_t cc[]) {
  *al0 = QQ_ONE * (1 / (1 + a1 * l + b1 * l * l));
  *be0 = QQ_ONE * (2 * (1 - b1 * l * l) / (1 + a1 * l + b1 * l * l));
  *be1 = QQ_ONE * ((1 - a1 * l + b1 * l * l) / (1 + a1 * l + b1 * l * l));
  Serial.println("*al0");
  Serial.println(*al0);
  //*al0 *= 1.0/set_ampl(ak, bk);
  cc[0]=1234;
  cc[5]=5678;
  return;
}

void get_coeff1(float a1, float b1, float l, int32_t ak[3], int32_t bk[3], int32_t cc[]) {
  ak[0] = QQ_ONE * (1 / (1 + a1 * l + b1 * l * l));
  ak[1] = 0;
  ak[2] = 0;
  bk[0] = QQ_ONE;
  bk[1] = QQ_ONE * (2 * (1 - b1 * l * l) / (1 + a1 * l + b1 * l * l));
  bk[2] = QQ_ONE * ((1 - a1 * l + b1 * l * l) / (1 + a1 * l + b1 * l * l));
  Serial.println("ak[x]");
  Serial.println(ak[0]);
  ak[0] *= 1.0/set_ampl(ak, bk);
  Serial.println("ak[x]");
  Serial.println(ak[0]);
  cc[0]=1234;
  cc[5]=5678;
  return;
}

float set_ampl(int32_t ak[3], int32_t bk[3]) {
  float a0 = 0.0f;
  float b0 = 0.0f;
  for(int i=0; i<3; i++) {
    a0 += ak[i];
    b0 += bk[i];
    Serial.println(ak[i]);
    Serial.println(bk[i]);
  }
  Serial.println("+++set_ampl+++");
  Serial.println(a0);
  Serial.println(b0);
  return(a0 / b0);
}

int32_t run_filter_one(int32_t input) {
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

int32_t run_filter_loop(int32_t input) {
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



