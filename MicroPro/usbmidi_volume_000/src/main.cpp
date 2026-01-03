#include <Arduino.h>
//#include "USB-MIDI.h"
#include "MIDIUSB.h"
//#include <MIDI.h>     
#include <stdio.h>
//using namespace std;

//USBMIDI_CREATE_DEFAULT_INSTANCE();
//#include <iostream>
using namespace std;

// Class definition
class GfG
{
  public:
    // Data member
    int val;

    // Member function
    void show()
    {
//        cout << "Value: " << val << endl;
    }
};

class Car {        // The class
  public:          // Access specifier
    String brand;  // Attribute
    String model;  // Attribute
    int year;      // Attribute
    Car(String x, String y, int z); // Constructor declaration
};

// Constructor definition outside the class
Car::Car(String x, String y, int z) {
  brand = x;
  model = y;
  year = z;
}

void volumeChange(byte channel, byte volume);
void controlChange(byte channel, byte control, byte value);


//==============================================================
//                  SETUP
//==============================================================
void setup(void){
  Serial.begin(115200);
  Serial.println("NodeMCU_Volume");
  Car carObj1("BMW", "X5", 1999);
  Car carObj2("Ford", "Mustang", 1969);
}

//==============================================================
//                     LOOP
//==============================================================
void loop(void){
 int a = analogRead(A0);
 Serial.println(a);
 //volumeChange(0, 0x7f);
 controlChange(0, 0x7, 127);
 MidiUSB.flush();
 delay(2000);
 //volumeChange(0, 0x0);
 controlChange(0, 0x7, 0);
 MidiUSB.flush();
 digitalWrite(LED_BUILTIN_RX, LOW);
 delay(1000);
 digitalWrite(LED_BUILTIN_RX, HIGH);
 delay(1000);
}


void volumeChange(byte channel, byte volume) {

  midiEventPacket_t volumeChange = {0x0c, byte(0xC0 | channel), volume};

  MidiUSB.sendMIDI(volumeChange);
}

void controlChange(byte channel, byte control, byte value) {

  midiEventPacket_t event = {0x0B, (byte)(0xB0 | channel), control, value};

  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}



