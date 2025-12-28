#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MIDIUSB.h>

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

#define Q14_ONE (1<<12)

LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
//LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned long currentMillis;
const long interval = 5;  // interval at which to blink (milliseconds)
volatile uint16_t adc_value = ADC; // Wert aus dem ADC-Register lesen
volatile uint16_t a, aold;

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
void get_coeff(float a1, float b1, float l, float *al0, float *be0, float *be1);
void setup_filter();
void run_filter();
void run_test_filter();

float ak1[3];
float bk1[3] = {Q14_ONE,0,0};
int32_t zz[3];

void setup() {

  Serial.begin(115200);
  delay(5000);
  Serial.println("prg: USBMidi Volume 1");

  //Use predefined PINS consts

  //Wire.begin(D2, D3);

  lcd.begin (16,2);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home();
  lcd.clear();
  lcd.print("USBMidi Volume 1");
  lcd.setCursor ( 0, 1 );
  lcd.print("V1.0");

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
  digitalWrite(LED_BUILTIN_RX, LOW);
  digitalWrite(LED_BUILTIN_RX, HIGH);
  setupadc();
  setup_filter();
  for(int i=0; i<3; i++)
    Serial.println(ak1[i]);
  for(int i=0; i<3; i++)
    Serial.println(bk1[i]);
  run_test_filter();
}

 int xk, xk1;


void loop()
{ 
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
    a = adc_value >> 3;
    a = a*2;
    a = a/2;
    a = a*2;
    a = a/2;
    a = a*2;
    a = a/2;
    a = a*2;
    a = a/2;
    ADCSRA |= (1 <<ADSC); // Optional: Neue Konvertierung starten
    //a = analogRead(A0);
    if (a != aold) {
      digitalWrite(D8, HIGH);
      controlChange(0, 0x7, a);
      digitalWrite(D8, LOW);
      aold=a;
    }
    //MidiUSB.flush();
    //Serial.println(ADMUX, HEX);
    //Serial.println(ADCSRA, HEX);
    //Serial.println(ADCSRB, HEX);
    //Serial.println(a, HEX);
    digitalWrite(D10, LOW);
  }
  digitalWrite(LED_BUILTIN_RX, LOW);
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
  adc_value = ADC; // Wert aus dem ADC-Register lesen
  digitalWrite(D9, adc_value>>2);
  // Hier Wert verarbeiten oder in eine Queue legen
  digitalWrite(D9, LOW);
  digitalWrite(D9, HIGH);
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
  const int fg = 10;
  const int fs = 1000;
  const float l = 1 / tan(3.14159*fg/fs);
  get_coeff(a1, b1, l, &ak1[0], &bk1[1], &bk1[2]);
}

void get_coeff(float a1, float b1, float l, float *al0, float *be0, float *be1) {
  *al0 = Q14_ONE * (1 / (1 + a1 * l + b1 * l * l));
  *be0 = Q14_ONE * (2 * (1 - b1 * l * l) / (1 + a1 * l + b1 * l * l));
  *be1 = Q14_ONE * ((1 - a1 * l + b1 * l * l) / (1 + a1 * l + b1 * l * l));
  return;
}

void run_filter() {
//    yn = al00 * uz + z0
//    z1 = z2 - (be01 *  yn)
//    z2 = - (be02 *  yn)
//    z0 = z1
//    z1 = z2
}

int32_t  output=0;

void run_test_filter() {
  int32_t input = 100;
  int Qcoeff_sh = 10;
  int32_t Qcoeff = (1<<Qcoeff_sh);
  int32_t Icoeff = (int32_t) (0.20 * Qcoeff);
  Serial.println("****************************************");
  Serial.println(input);
  Serial.println(Qcoeff);
  Serial.println(Icoeff);
  Serial.println(output);


  for(int i=0; i<100; i++) {
    //Serial.println((Qcoeff - Icoeff) * output);
    output = ((Icoeff * input) + (((Qcoeff - Icoeff) * output) >> Qcoeff_sh));
    Serial.println(output);
    Serial.println(output >> Qcoeff_sh);
  }
}
