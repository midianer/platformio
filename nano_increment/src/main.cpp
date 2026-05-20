#include <Arduino.h>

//#define PIN_A0 A0
//#define PIN_A1 A1
//#define PIND6 PD6
//#define PIND7 PD7
#define XPIND2 PIND2
#define XPIND3 PIND3
#define BoardLED 13

volatile int IncCnt=0;

void SetPD2();
void SetPD3();

void setup()
{

  Serial.begin(115200);

  pinMode(BoardLED,OUTPUT); digitalWrite(BoardLED,LOW); // Board-LED
  pinMode(PIN_A0, INPUT);
  pinMode(PIN_A1, INPUT);
  pinMode(XPIND2, INPUT);
  pinMode(PIND3, INPUT);
  pinMode(PIND6, OUTPUT);
  pinMode(PIND7, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(XPIND2), SetPD2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(XPIND3), SetPD3, CHANGE);
}


void loop() {
  //int pa0, pa1, pd2, pd3;
  //digitalWrite(13,LOW);
  //digitalWrite(13,HIGH);
  delay(1000);
//  pa0 = digitalRead(PIN_A0);
//  pd2 = digitalRead(XPIND2);
//  pd3 = digitalRead(XPIND3);
//  digitalWrite(PIND6, pd2);
//  digitalWrite(PIND7, pd3);
//  digitalWrite(13, pd2);
  Serial.println(IncCnt);
}


void SetPD2() {
  int pd2, pd3;
  pd2 = digitalRead(XPIND2);
  pd3 = digitalRead(XPIND3);
  if(pd2==0){
    if(pd3==0) IncCnt--;
    if(pd3==1) IncCnt++;
  }
  if(pd2==1){
    if(pd3==0) IncCnt++;
    if(pd3==1) IncCnt--;
  }
  digitalWrite(PIND6, pd2);
  digitalWrite(13, pd2);
}

void SetPD3() {
  int pd2, pd3;
  pd2 = digitalRead(XPIND2);
  pd3 = digitalRead(XPIND3);
  if(pd3==0){
    if(pd2==0) IncCnt++;
    if(pd2==1) IncCnt--;
  }
  if(pd3==1){
    if(pd2==0) IncCnt--;
    if(pd2==1) IncCnt++;
  }
  digitalWrite(PIND7, pd3);
}
