#include<joystick.h>
#include <Arduino.h>
#include <math.h>

int clamp(int v) {
  if (v > max_joy) return max_joy;
  else if (v < -max_joy) return -max_joy;
  return v;
}
int deadzone(int v) {
  if (fabs(v) < deadzone_joy) return 0;
  return v;
}

void calibrateJoystickCenter() {
  int N = 60;
  long sumx, sumy = 0;
  for (int i = 0; i < N; i++) {
    sumx += analogRead(vx);
    sumy += analogRead(vy);
    delay(5);
  }

  joy_center_x = (int)(sumx/N);
  joy_center_y = (int)(sumy/N);

  Serial.println("Calibrated Joystick Center");
}

void joy_setup() {
  analogReadResolution(12);
  calibrateJoystickCenter();
  Serial.println("Joystick initialized");
}

void read_joy(){
  joy_x = clamp(deadzone(analogRead(vx) - joy_center_x));
  joy_y = clamp(deadzone(analogRead(vy) - joy_center_y));

}