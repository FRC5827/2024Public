#include <FastLED.h>

#define DEBUG 0

#define LED_PIN 3
#define SWITCH_PIN 2

// Total number of LEDs
#define NUM_LEDS 91

// Full string
// Max current draw of 0.702 A @ 5V
#define START_LED 0
#define COUNT_LEDS 91

// Top string only
// Max current draw of 0.290 A @ 5V
//#define START_LED 20
//#define COUNT_LEDS 26

#define PURPLE_HUE 192
#define ORANGE_HUE 6
#define DEFAULT_SAT 255

CRGB leds[NUM_LEDS];
 
void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  #if DEBUG == 1
    Serial.begin(9600);
    Serial.println("setup complete");
  #endif
}
 
int fade_value = 0;
int fade_increment = 1;
int detected_flash_delay_on = 1000;
int detected_flash_delay_off = 500;
bool detected_flash_state = true;
bool even_leds = true;

void fill_leds(int hue, int fade_value) {

  for (int i = START_LED; i < START_LED + COUNT_LEDS; i++) {
    if (i % 2 == even_leds) {
      leds[i] = CHSV(hue, DEFAULT_SAT, fade_value);
    } else {
      leds[i] = CHSV(hue, DEFAULT_SAT, 0);
    }
  }

  if (
    // Don't move the LEDs if it's orange (or it will appear like it's not moving at all).
    // Otherwise 'move' them three times during the cycle.
    (hue != ORANGE_HUE && fade_value == 0) ||
     fade_value == 127 || fade_value == 255) {
    even_leds = !even_leds;
  }
}

void loop() {

  if (digitalRead(SWITCH_PIN) == HIGH) {
    
    detected_flash_state = true;
    fill_leds(PURPLE_HUE, fade_value);

    #if DEBUG == 1
    if (fade_value % 10 == 0) {
      Serial.print("digital read : ");
      Serial.print(digitalRead(SWITCH_PIN));
      Serial.print(" fade_value : ");
      Serial.print(fade_value);
      Serial.println(); 
    }
    #endif

    fade_value += fade_increment;

    if (fade_value == 255 || fade_value == 0){
      fade_increment *= -1; 
    }

    FastLED.show();
    delay(10);

  } else {
    // Detected game piece
    if (detected_flash_state) {
      fill_leds(ORANGE_HUE, 255);
    } else {
      fill_leds(ORANGE_HUE, 0);
    }

    FastLED.show();
    if (detected_flash_state) {
      delay(detected_flash_delay_on);
    } else {
      delay(detected_flash_delay_off);
    }
    detected_flash_state = !detected_flash_state;

    #if DEBUG == 1
    Serial.print("digital read : ");
    Serial.print(digitalRead(SWITCH_PIN));
    Serial.println();
    #endif
  }
}
