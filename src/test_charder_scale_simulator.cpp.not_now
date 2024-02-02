
// 按一下 button, println("1"), LED 顯示綠色一秒，然後 LED 顯示紅色。
// 按兩下 button, println("2"), LED 顯示藍色一秒，然後 LED 顯示紅色。
#include <Arduino.h>
#include <Wire.h>
#include <M5Atom.h>

#include "OneButton.h"

// define GPIO39 for button
#define buttonPin 39
bool buttonDown = false;

// use OneButton
OneButton button(buttonPin, true);
void myClickFunction()
{
  Serial.println("1");
  M5.dis.drawpix(0, 0x00ff00);
  delay(1000);
  M5.dis.drawpix(0, 0xff0000);  
}

void myDoubleClickFunction()
{
  Serial.println("2");
  M5.dis.drawpix(0, 0x0000ff);  
  delay(1000);
  M5.dis.drawpix(0, 0xff0000);    
}

void setup()
{
  M5.begin(true, false, true); // Init Atom-Matrix(Initialize serial port, LED).  初始化 ATOM-Matrix(初始化串口、LED点阵)
  delay(50);                   // delay 50ms.  延迟50ms

  M5.dis.drawpix(0, 0xff0000);

  // button GPIO - pressed when boot up t0 force to use bridge AP mode
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(115200);
  delay(1000);
  // Set device in STA mode to begin with
  
  // OneButton callbacks
  // link the myClickFunction function to be called on a click event.
  button.attachClick(myClickFunction);

  // link the doubleclick function to be called on a doubleclick event.
  button.attachDoubleClick(myDoubleClickFunction);

  // set 80 msec. debouncing time. Default is 50 msec.
  button.setDebounceTicks(80);
}

char receivedChar;
String content="";
void loop()
{
  button.tick();
}
