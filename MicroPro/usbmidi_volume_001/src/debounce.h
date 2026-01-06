#include <Arduino.h>

#ifndef __debounce_h
#define __debounce_h

#define DebugSerial 1

#define noOfButtons 2     //Exactly what it says; must be the same as the number of elements in buttonPins
#define bounceDelay 50    //Minimum delay before regarding a button as being pressed and debounced
#define minButtonPress 1  //Number of times the button has to be detected as pressed before the press is considered to be valid

void doStuff(uint8_t buttonNumber);
void controlChange(byte channel, byte control, byte value);

class ClDebounce {
  private:
    int _pin;
    int _ledno;
    uint32_t _previousMillis;
    uint32_t _currentMillis;
    uint32_t _pressCount;
    uint8_t _button;
  public:             // Access specifier
    ClDebounce(int pin, int ledno, int button);
    int myNum;        // Attribute (int variable)
    String myString;  // Attribute (string variable)
    void myFunction(int blinkRate);
};

#endif
