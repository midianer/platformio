//#include "MIDIUSB.h"
#include <Arduino.h>
#include "USB-MIDI.h"
#include <MIDI.h>     
#include <stdio.h>
using namespace std;

// USBMIDI RD700SX
// micro_rd700sx.build.vid=0x1209
// micro_rd700sx.build.pid=0x1F08
// micro_rd700sx.build.usb_product="RD700SX"

#define noOfButtons 2     //Exactly what it says; must be the same as the number of elements in buttonPins
#define bounceDelay 20    //Minimum delay before regarding a button as being pressed and debounced
#define minButtonPress 1  //Number of times the button has to be detected as pressed before the press is considered to be valid

const int buttonPins[] = {3, 2};      // Input pins to use, connect buttons between these pins and 0V
uint32_t previousMillis[noOfButtons];       // Timers to time out bounce duration for each button
uint8_t pressCount[noOfButtons];            // Counts the number of times the button is detected as pressed, when this count reaches minButtonPress button is regared as debounced 
uint8_t testCount[noOfButtons];             //Test count, incremented once per button press

//MIDI_CREATE_INSTANCE(HardwareSerial, Serial1,    midiA);
struct MyMIDISettings : public MIDI_NAMESPACE::DefaultSettings
{
  // When setting UseSenderActiveSensing to true, MIDI.read() *must* be called
  // as often as possible (1000 / SenderActiveSensingPeriodicity per second).
  static const bool UseSenderActiveSensing = false;
};

USBMIDI_NAMESPACE::usbMidiTransport usbMIDI(0);
MIDI_NAMESPACE::MidiInterface<USBMIDI_NAMESPACE::usbMidiTransport, MyMIDISettings> midiA((USBMIDI_NAMESPACE::usbMidiTransport&)usbMIDI);

void setup() {
  uint8_t i;
  pinMode(LED_BUILTIN_TX, OUTPUT);
  digitalWrite(LED_BUILTIN_TX, HIGH);
  pinMode(LED_BUILTIN_RX, OUTPUT);
  digitalWrite(LED_BUILTIN_RX, HIGH);
  for (i = 0; i < noOfButtons; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  midiA.begin(MIDI_CHANNEL_OMNI);
  midiA.turnThruOff();
}

void loop1() {
  digitalWrite(LED_BUILTIN_RX, HIGH);
  digitalWrite(LED_BUILTIN_TX, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN_RX, LOW);
  digitalWrite(LED_BUILTIN_TX, LOW);
  delay(1000);
}

void loop() {
  midiEventPacket_t rx;
  midiEventPacket_t tx;
  midiEventPacket_t noteOn80;
  midiEventPacket_t noteOn90;
  midiEventPacket_t noteOnB0;
  midiEventPacket_t notePgm;
  uint8_t m_chan;
  byte m_byt1;
  uint8_t m_byt2;
  byte m_type;


  //do {
    rx = MidiUSB.read();
    if (rx.header != 0) {
      digitalWrite(LED_BUILTIN_TX, LOW);
      m_byt1 = rx.byte1;
      midiA.send(m_byt1, (uint8_t)rx.byte2, (uint8_t)rx.byte3, (uint8_t)(rx.byte1 && 0xf));
      //delay(1000);
      digitalWrite(LED_BUILTIN_TX, HIGH);
    }
  //} while (rx.header != 0);
  if(midiA.read()) {
    digitalWrite(LED_BUILTIN_RX, LOW);
    m_chan = midiA.getChannel();
    m_byt1 = midiA.getData1();
    m_byt2 = midiA.getData2();
    m_type = midiA.getType();
    switch(m_type) {
      case midi::NoteOff:
        //midiEventPacket_t MidiEv8 = {0x08, 0x80, m_byt1, m_byt2};
        noteOn80 = {0x08, byte(m_type | m_chan), m_byt1, m_byt2};
        MidiUSB.sendMIDI(noteOn80);
        MidiUSB.flush();
      break;
      case midi::NoteOn:
        //midiEventPacket_t MidiEv9 = {0x09, 0x90, midiA.getData1(), midiA.getData2()};
        //MidiUSB.sendMIDI(MidiEv9);
        noteOn90 = {0x09, byte(m_type | m_chan), m_byt1, m_byt2};
        //midiEventPacket_t noteOn90 = {0x09, 0x90 | 0, midiA.getData1(), midiA.getData2()};        
        MidiUSB.sendMIDI(noteOn90);
        MidiUSB.flush();
      break;
      case midi::ControlChange:
        //midiEventPacket_t MidiEv9 = {0x09, 0x90, midiA.getData1(), midiA.getData2()};
        //MidiUSB.sendMIDI(MidiEv9);
        noteOnB0 = {0x0B, byte(m_type | m_chan), m_byt1, m_byt2};
        //midiEventPacket_t noteOn90 = {0x09, 0x90 | 0, midiA.getData1(), midiA.getData2()};        
        MidiUSB.sendMIDI(noteOnB0);
        MidiUSB.flush();
      break;
      case 0xC0:
        notePgm = {0x0C, byte(m_type | m_chan), m_byt1};        
        MidiUSB.sendMIDI(notePgm);
        MidiUSB.flush();
      break;
      default:
      break;
    }
    //delay(1000);
    digitalWrite(LED_BUILTIN_RX, HIGH);
    //MidiUSB.sendMIDI(
    //  midiA.getType(),
    //  midiA.getData1(),
    //  midiA.getData2(),
    //  midiA.getChannel() && 0xf
    //);
  };
}


// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).
//void noteOn(byte channel, byte pitch, byte velocity) {
//
//  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
//
//  MidiUSB.sendMIDI(noteOn);
//}
//
//void noteOff(byte channel, byte pitch, byte velocity) {
//
//  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
//
//  MidiUSB.sendMIDI(noteOff);
//}


