
#include <Arduino.h>
#include <BleMouse.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "i2c.h"
#include "joystick.h"
#include <math.h>

#define jb 27 //GPIOP27
#define lb 2 //GIOP2
#define rb 15 //GIOP15
#define mode 13

int state = 0;
uint32_t last_state = 0;

#define report_hz 100
#define report_ms 1000/report_hz

#define debounce_ms 10

BleMouse bleMouse("ESP32 AIR MOUSE", "KiransMouse", 100);

struct button {
  int pin;
  bool stable;
  bool lastRead;
  uint32_t lastChange;
};

bool readPressedRaw(int pin){
  return digitalRead(pin) == LOW;
}

bool button_update(button &b, uint32_t now) {
  bool pressed = readPressedRaw(b.pin);
  if(pressed != b.lastRead) {
    b.lastRead = pressed;
    b.lastChange = now;
  }

  if((now - b.lastChange) > debounce_ms) {
    b.stable = b.lastRead;
  }

  return b.stable;
}

void setup() {
  // put your setup code here, to run once:
  delay(2000);
  Serial.begin(9600);
  delay(1000);

  pinMode(jb, INPUT_PULLUP);
  pinMode(lb, INPUT_PULLUP);
  pinMode(rb, INPUT_PULLUP);
  pinMode(mode, INPUT_PULLUP);

  joy_setup();

  mpu_setup();

  bleMouse.begin();
}

uint32_t last_report_ms = 0;

button left{lb, false, false, 0};
button right{rb, false, false, 0};
button joy{jb, false, false, 0};
button mode_control{mode, false, false, 0};

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t now = millis();

  if(!bleMouse.isConnected()) {
    //Serial.println("Not connected");
    //delay(2000);
    return;
  }

  //Serial.println("BLE mouse connected");

  if (now - last_report_ms < report_ms) return;

  last_report_ms = now;

  bool left_button = button_update(left, now);
  bool right_button = button_update(right, now);
  bool joy_button = button_update(joy, now);
  bool mode_button = button_update(mode_control, now);

  if (now - last_state > 20 && mode_button) {
    last_state = now;
    state ^= 1;
    if (state == 1) mpu_wake(); else mpu_sleep();
  }
  left_button = left_button || joy_button;
  read_joy();
  read_mpu();
  
  bool mid_button = false;
  if (left_button && right_button) mid_button = true;

  if (left_button && !mid_button) bleMouse.press(MOUSE_LEFT); else bleMouse.release(MOUSE_LEFT);  
  if (right_button && !mid_button) bleMouse.press(MOUSE_RIGHT); else bleMouse.release(MOUSE_RIGHT);
  if (mid_button) bleMouse.press(MOUSE_MIDDLE); else bleMouse.release(MOUSE_MIDDLE);

  // Serial.print("left ");
  // Serial.print(left_button);
  // Serial.print(" middle ");
  // Serial.print(mid_button);
  // Serial.print(" right ");
  // Serial.println(right_button);
  // Serial.println();
  // Serial.print(mode_button); Serial.print("  "); Serial.println(state);
  
  if (state == 0) {
    bleMouse.move(2*joy_x, 2*joy_y, 0);
  }
  else {
    bleMouse.move(dx, dy, -joy_y);
  }  
  //Serial.print("REPORT SENT");
  
}
