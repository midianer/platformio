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

LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
//LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned long currentMillis;
const long interval = 200;  // interval at which to blink (milliseconds)
volatile uint16_t adc_value = ADC; // Wert aus dem ADC-Register lesen

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


void setup() {

  Serial.begin(115200);
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
  delay(3000);

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
  digitalWrite(LED_BUILTIN_RX, LOW);
  digitalWrite(LED_BUILTIN_RX, HIGH);
  //setupadc();
}

 int xk, xk1;
 int a;


void loop()
{ 
  digitalWrite(LED_BUILTIN_RX, HIGH);
  //a = adc_value;
//  xk = (a * 16 + 16 * 15 * xk) >> 8;
//  if(xk>>8 != xk1) {
//    controlChange(0, 0x7, xk & 0x7f);
//    MidiUSB.flush();
//  }
//  xk1 = xk>>8;
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    digitalWrite(D8, LOW);
    // save the last time you blinked the LED
    //previousMillis = currentMillis;  //volumeChange(0, 0x7f);
    previousMillis -= interval;  //volumeChange(0, 0x7f);
    //ADCSRA |= (1 <<ADSC); // Optional: Neue Konvertierung starten
    digitalWrite(D8, HIGH);
    a = analogRead(A0);
    controlChange(0, 0x7, a >> 3);
    MidiUSB.flush();
    //Serial.println(a);
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
  adc_value = ADC; // Wert aus dem ADC-Register lesen
  // Hier Wert verarbeiten oder in eine Queue legen
  digitalWrite(D9, HIGH);
}

void setupadc() {
  // ... ADC-Pins als Input konfigurieren
  ADMUX = (1 <<REFS0); // Referenzspannung einstellen
  ADCSRA = (1 <<ADEN) | (1 <<ADIE) | (1 <<ADPS2) | (1 <<ADPS1); // ADC aktivieren, Interrupt aktivieren, Prescaler setzen
  // Oder Timer-Trigger konfigurieren, z.B. ADCSRA |= (1 <<ADATE); ADTS Bits setzen
  sei(); // Globale Interrupts aktivieren
  ADCSRA |= (1 <<ADSC); // Erste Konvertierung starten
}

