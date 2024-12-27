#include <Arduino.h>
#include <rmt_led_strip.hpp>

// 1 LED, pin 15
arduino::ws2812 leds(15,1);

void setup() {
  Serial.begin(115200);
  leds.initialize();
  // set led index 0 to purple
  leds.color(0,255,0,255);
  leds.update();
}

void loop() {
  // put your main code here, to run repeatedly:
}
