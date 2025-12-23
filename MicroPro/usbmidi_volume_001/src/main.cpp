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

LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
//LiquidCrystal_I2C lcd(0x27, 16, 2);

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

}

 int xk, xk1;

void loop()
{ 
 int a = analogRead(A0);
 xk = (a * 16 + 16 * 15 * xk) >> 8;
 if(xk>>8 != xk1) {
   controlChange(0, 0x7, xk & 0x7f);
   MidiUSB.flush();
 }
 xk1 = xk>>8;
 Serial.println(a);
 //volumeChange(0, 0x7f);
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


