
#include <Arduino.h>
#include <BleMouse.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <i2c.h>
#include <joystick.h>
#include <math.h>

#define LED 2
#define jb 5
#define lb 24
#define rb 25

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

void flash_complete(){
  digitalWrite(LED, HIGH);
  Serial.println("LED is on");
  Serial.println("BLE Starting ...");
  delay(1000);
  // digitalWrite(LED, LOW);
  // Serial.println("LED is off");
  // delay(1000);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  flash_complete();

  pinMode(jb, INPUT_PULLUP);
  pinMode(lb, INPUT_PULLUP);
  pinMode(jb, INPUT_PULLUP);

  joy_setup();

  mpu_setup();

}

uint32_t last_report_ms = 0;

button left{lb, false, false, 0};
button right{rb, false, false, 0};
button joy{jb, false, false, 0};

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t now = millis();

  if(!bleMouse.isConnected()) {
    Serial.println("Not connected");
    return;
  }

  if (now - last_report_ms) return;
  last_report_ms = now;

  bool left_button = button_update(left, now);
  bool right_button = button_update(right, now);
  bool joy_button = button_update(joy, now);

  read_joy();
  read_mpu();
  bool mid_button = false;
  if (left_button && right_button) mid_button = true;

  if (left_button) bleMouse.press(MOUSE_LEFT); else bleMouse.release(MOUSE_LEFT);
  if (right_button) bleMouse.press(MOUSE_RIGHT); else bleMouse.release(MOUSE_RIGHT);
  if (joy_button) bleMouse.press(MOUSE_LEFT); else bleMouse.release(MOUSE_LEFT);  
  if (mid_button) bleMouse.press(MOUSE_MIDDLE); else bleMouse.release(MOUSE_MIDDLE);

  if (dx != 0 || dy != 0 || joy_y != 0){
    bleMouse.move(dx, dy, joy_y);
  }
}
