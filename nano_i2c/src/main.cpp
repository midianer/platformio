#include <Arduino.h>

#include <Wire.h>

void receiveEvent(int howMany);
char readend;

void setup()
{

  Serial.begin(115200);

  //I2C-Adresszuweisung:
  Wire.begin(0x40);

  //Handler für das I2C-Empfangsereignis festlegen (siehe unten)
  Wire.onReceive(receiveEvent);
 
  pinMode(13,OUTPUT); digitalWrite(13,LOW); // Bord-LED
}


void loop() {
    
  if(readend == 0x0d) {
    Serial.print("L");
    Wire.beginTransmission(0x40);
    Wire.write(0x55);
    Wire.write(0x55);
    Wire.write(0x55);
    Wire.write(0x55);
    Wire.write(0x0d);
    Wire.endTransmission();    // stop transmitting
  }
  readend = 0;
    
} // derzeit nix drin hier (siehe unten)


void receiveEvent(int howMany){

  while(Wire.available())
  {
    char c = Wire.read();
    readend = c;
   
    if(c == 'a')
    {
      digitalWrite(13,HIGH);
    }
    else if(c == 0x0d)
    {
      digitalWrite(13,LOW);
    }
  }
}
