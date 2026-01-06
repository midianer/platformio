#include <Arduino.h>
#include "debounce.h"

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

// Function to do whatever you want done once the button has been debounced.
// In this example it increments a counter and send the count to the serial monitor.
// Put your own functions here to do whatever you like.
void doStuff(uint8_t buttonNumber) {
  #if (DebugSerial > 0)
    //++testCount[buttonNumber];
    //Serial.print("Button ");
    //Serial.print(buttonNumber);
    //Serial.print(" testcount = ");
    //Serial.println (testCount[buttonNumber]);
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
}

