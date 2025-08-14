//#include "MIDIUSB.h"
#include <Arduino.h>
#include "USB-MIDI.h"
#include <MIDI.h>     
#include <stdio.h>
using namespace std;
//extern usb_midi_class usbMIDI;

//MIDI_CREATE_INSTANCE(HardwareSerial, midi2, MidiUSB);  // USB MIDI
//USBMIDI_CREATE_DEFAULT_INSTANCE();

///////////////////////////////////////////////////////////////////////////////////////////////////////////

#define noOfButtons 2     //Exactly what it says; must be the same as the number of elements in buttonPins
#define bounceDelay 50    //Minimum delay before regarding a button as being pressed and debounced
#define minButtonPress 1  //Number of times the button has to be detected as pressed before the press is considered to be valid

#define DebugSerial 2

void doStuff(uint8_t buttonNumber);
void debounce();
void noteOn(byte channel, byte pitch, byte velocity);
void noteOff(byte channel, byte pitch, byte velocity);
void controlChange(byte channel, byte control, byte value);
void loop_debounce();
void loop_CLdebounce();
void loop4();

///////////////////////////////////////////////////////////////////////////////////////////////////////////

class ClDebounce {       // The class
  public:             // Access specifier
    ClDebounce(int pin, int ledno, int button);
    int myNum;        // Attribute (int variable)
    String myString;  // Attribute (string variable)
    void myFunction(int blinkRate);
  private:
    int _pin;
    int _ledno;
    uint32_t _previousMillis;
    uint32_t _currentMillis;
    uint32_t _pressCount;
    uint8_t _button;
};
ClDebounce::ClDebounce(int pin, int ledno, int button) {
	pinMode(pin, INPUT);
	_pin = pin;
	_ledno = ledno;
    _button = button;
}
void ClDebounce::myFunction(int blinkRate){
    _currentMillis = millis();
    digitalWrite(_ledno, digitalRead(_pin));
    if (digitalRead(_pin)==LOW) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
        _previousMillis = _currentMillis;        //Set previousMillis to millis to reset timeout
        _pressCount = 0;                        //Set the number of times the button has been detected as pressed to 0
    } else {
      if ((_currentMillis - _previousMillis) > bounceDelay) {
        _previousMillis = _currentMillis;        //Set previousMillis to millis to reset timeout
        if (_pressCount < 10) {
            ++_pressCount;
        }
        if (_pressCount == minButtonPress) {
          doStuff(_button);                             //Button has been debounced. Call function to do whatever you want done.
        }
      }
    }
}

ClDebounce redLED(4, LED_BUILTIN_RX, 0);
ClDebounce greenLED(5, LED_BUILTIN_TX, 1);

///////////////////////////////////////////////////////////////////////////////////////////////////////////

// USBMIDI ArduinoTaster1
// micro111.build.vid=0x1209
// micro111.build.pid=0x1F01
// micro111.build.usb_product="ArduinoTaster1"

//const int buttonPins[] = {3, 2};      // Input pins to use, connect buttons between these pins and 0V
const int buttonPins[] = {4, 5};      // Input pins to use, connect buttons between these pins and 0V
uint32_t previousMillis[noOfButtons];       // Timers to time out bounce duration for each button
uint8_t pressCount[noOfButtons];            // Counts the number of times the button is detected as pressed, when this count reaches minButtonPress button is regared as debounced 
uint8_t testCount[noOfButtons];             //Test count, incremented once per button press


///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef button_h
#define button_h

//#include "Arduino.h"

class Button
{
  private:
    uint8_t btn;
    uint16_t state;
  public:
    void begin(uint8_t button) {
      btn = button;
      state = 0;
      pinMode(btn, INPUT_PULLUP);
    }
    bool debounce() {
      state = (state<<1) | digitalRead(btn) | 0xfe00;
      Serial.println(state);
      return (state == 0xff00);
    }
};
#endif

Button btn1;
Button btn2;

//https://github.com/e-tinkers/button

///////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  uint8_t i;
  #if (DebugSerial > 0) 
    Serial.begin(115200);
  #endif
  pinMode(LED_BUILTIN_TX, OUTPUT);
  digitalWrite(LED_BUILTIN_TX, HIGH);
  pinMode(LED_BUILTIN_RX, OUTPUT);
  digitalWrite(LED_BUILTIN_RX, LOW);
  for (i = 0; i < noOfButtons; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  //btn1.begin(4);
  //btn2.begin(5);
  //  loop();
}

void loop() {
	//loop_debounce();
	loop_CLdebounce();
}

void loopx() {
  // press button 1 to turn on the LED
  digitalWrite(LED_BUILTIN_RX, LOW);
  digitalWrite(LED_BUILTIN_TX, LOW);
  if (btn1.debounce()) {
    digitalWrite(LED_BUILTIN_RX, HIGH);
    controlChange(0, 0x12, 0);
    #if (DebugSerial > 0)
        Serial.println("Pressed 0x12");
    #endif
  }
  // press button 2 to turn off the LED
  if (btn2.debounce()) {
    digitalWrite(LED_BUILTIN_TX, LOW);
    controlChange(0, 0x13, 0);
    #if (DebugSerial > 0)
        Serial.println("Pressed 0x13");
    #endif
  }
}

void loop2() {
  for(int i=0; i < 5; i++) {
    digitalWrite(LED_BUILTIN_RX, HIGH);
    digitalWrite(LED_BUILTIN_TX, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN_RX, LOW);
    digitalWrite(LED_BUILTIN_TX, LOW);
    delay(100);
  }
}

void loop1() {
  digitalWrite(LED_BUILTIN_RX, HIGH);
  digitalWrite(LED_BUILTIN_TX, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN_RX, LOW);
  digitalWrite(LED_BUILTIN_TX, LOW);
  delay(1000);
}

void loop4() {
  digitalWrite(LED_BUILTIN_RX, digitalRead(buttonPins[0]));
  digitalWrite(LED_BUILTIN_TX, digitalRead(buttonPins[1]));
}


void controlChange(byte channel, byte control, byte value) {

  midiEventPacket_t event = {0x0B, (byte)(0xB0 | channel), control, value};

  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}

void loop_CLdebounce() {
  greenLED.myFunction(2000);
  redLED.myFunction(1000);
}

void loop_debounce() {
  debounce();
  //delay(10);     //Your other code goes here instead of this delay. DO NOT leave this delay here, it's ONLY for demonstration.
}

void debounce() {
  uint8_t i;
  uint32_t currentMillis;
  digitalWrite(LED_BUILTIN_RX, digitalRead(buttonPins[0]));
  digitalWrite(LED_BUILTIN_TX, digitalRead(buttonPins[1]));
  #if (DebugSerial > 4)
      if(digitalRead(buttonPins[0])==HIGH) {
        Serial.println("B0 pressed");
      }
      if(digitalRead(buttonPins[1])==HIGH) {
        Serial.println("B1 pressed");
      }
  #endif            
  for (i = 0; i < noOfButtons; ++i) {
    currentMillis = millis();
    if (digitalRead(buttonPins[i])==LOW) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
        previousMillis[i] = currentMillis;        //Set previousMillis to millis to reset timeout
        pressCount[i] = 0;                        //Set the number of times the button has been detected as pressed to 0
    } else {
      if ((currentMillis - previousMillis[i]) > bounceDelay) {
        //Serial.print("AAAAAAAAAAAA: ");
        //Serial.println(i);
        #if (DebugSerial > 2)
            Serial.print("A ");
            Serial.println (currentMillis);
            Serial.print("B ");
            Serial.println (previousMillis[0]);
            Serial.print("C ");
            Serial.println (previousMillis[1]);
        #endif
        previousMillis[i] = currentMillis;        //Set previousMillis to millis to reset timeout
        if (pressCount[i] < 10) {
            ++pressCount[i];
        }
        #if (DebugSerial > 1)
            Serial.print(" pressCount 0x12 = ");
            Serial.println (pressCount[0]);
            Serial.print(" pressCount 0x13 = ");
            Serial.println (pressCount[1]);
        #endif
        if (pressCount[i] == minButtonPress) {
          doStuff(i);                             //Button has been debounced. Call function to do whatever you want done.
        }
      }
    }
  }
  for (i = 0; i < noOfButtons; ++i) {
  }
}

// Function to do whatever you want done once the button has been debounced.
// In this example it increments a counter and send the count to the serial monitor.
// Put your own functions here to do whatever you like.
void doStuff(uint8_t buttonNumber) {
  #if (DebugSerial > 0)
    ++testCount[buttonNumber];
    Serial.print("Button ");
    Serial.print(buttonNumber);
    Serial.print(" testcount = ");
    Serial.println (testCount[buttonNumber]);
  #endif
  switch(buttonNumber) {
    case 0: controlChange(0, 0x12, 0);
            #if (DebugSerial > 0)
              Serial.println("Pressed 0x12");
            #endif
            break;
    case 1: controlChange(0, 0x13, 0);
            #if (DebugSerial > 0)
              Serial.println("Pressed 0x13");
            #endif
            break;
    default:
            break;
  }
}//void readButtons()
//{
//
//  for (int i = 0; i < NUM_BUTTONS; i++)
//
//  {
//
//    if (digitalRead(buttons[i]) == LOW)
//
//    {
//
//      bitWrite(pressedButtons, i, 1);
//
//      delay(50);
//
//    }
//
//    else
//
//      bitWrite(pressedButtons, i, 0);
//
//  }
//}
//
//void readIntensity()
//{
//
//  int val = analogRead(intensityPot);
//
//  intensity = (uint8_t) (map(val, 0, 1023, 0, 127));
//}
//
//void playNotes()
//{
//
//  for (int i = 0; i < NUM_BUTTONS; i++)
//
//  {
//
//    if (bitRead(pressedButtons, i) != bitRead(previousButtons, i))
//
//    {
//
//      if (bitRead(pressedButtons, i))
//
//      {
//
//        bitWrite(previousButtons, i , 1);
//
//        noteOn(0, notePitches[i], intensity);
//
//        MidiUSB.flush();
//
//      }
//
//      else
//
//      {
//
//        bitWrite(previousButtons, i , 0);
//
//        noteOff(0, notePitches[i], 0);
//
//        MidiUSB.flush();
//
//      }
//
//    }
//
//  }
//}

// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

void noteOn(byte channel, byte pitch, byte velocity) {

  midiEventPacket_t noteOn = {0x09, byte(0x90 | channel), pitch, velocity};

  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {

  midiEventPacket_t noteOff = {0x08, byte(0x80 | channel), pitch, velocity};

  MidiUSB.sendMIDI(noteOff);
}




//void setup() {
//  uint8_t i;
//  uint32_t baudrate = 115200;
//  Serial.begin(baudrate);
//  Serial.println("");
//  Serial.print("Serial port connected: ");
//  Serial.println(baudrate);
//  for (i = 0; i < noOfButtons; ++i) {
//    pinMode(buttonPins[i], INPUT_PULLUP);
//    Serial.print("Testcount ");
//    Serial.print(i);
//    Serial.print(" = ");
//    Serial.println(testCount[i]);
//  }
//}





//constexpr uint8_t FLIP = 6;
//constexpr uint8_t MAX  = FLIP*2;
//
//void loop(Test& test)
//{
//  static uint8_t count = MAX;
//
//  if (test.read()) {
//    if (count < MAX) {
//      if (++count == FLIP) {
//        count = MAX;
//        test.report(1);
//      }
//    }
//  }
//  else {
//    if (count > 0) {
//      if (--count == FLIP) {
//        count = 0;
//        test.report(0);
//      }
//    }
//  }
//
//  test.sleep(1);
//}
//




