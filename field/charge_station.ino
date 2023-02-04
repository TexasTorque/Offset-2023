// Copyright 2023 Texas Torque
//
// This file is part of Torque-2023, which is not licensed for distribution.
// For more details, see ./license.txt or write <jus@justusl.com>.
// 
// charge_station.ino - Arduino code for the lights on the charge station.
// Authored by Omar Afzal and Justus Languell

#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <FastLED.h>
#include <Wire.h>

#define LED_PIN 7
#define NUM_LEDS 70
#define BRIGHTNESS 64
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

CRGB leds[NUM_LEDS];
CRGBPalette16 current_palette_;
TBlendType current_blending_;

extern CRGBPalette16 rwb_palette_;
extern const TProgmemPalette16 rwb_palette_p_ PROGMEM;
static int fill_color_ = 0, green_ = 100, red_ = 0;

float timer_millis_ = 0;
boolean balanced_ = false;

Adafruit_ADXL345_Unified accelerometer_ = Adafruit_ADXL345_Unified();

double start_time_;
uint8_t brightness_ = 255;

void setup(void) {
    Serial.begin(9600);
    if (!accelerometer_.begin()) {
        Serial.println("No ADXL345 sensor detected.");
        while (1)
            ;
    }

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
            .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

    current_palette_ = RainbowColors_p;
    current_blending_ = LINEARBLEND;
    start_time_ = millis();
}

void loop(void) {
    float elapsed = millis() - start_time_;
    if (elapsed < 2.5e3) {
        float f = fmod(elapsed / 250, 2.0);
        brightness_ = f <= 1 ? 0 : 255;
        fill_color_ = green_;
    } else {
        sensors_event_t event;
        accelerometer_.getEvent(&event);

        if (event.acceleration.x < 1.2 && event.acceleration.x > 0) {
            if (!balanced_) timer_millis_ = millis();
            balanced_ = true;
            if (millis() - timer_millis_ > 50) {
                fill_color_ = green_;
            }
        } else {
            fill_color_ = red_;
            balanced_ = false;
        }
        Serial.println(event.acceleration.x);
    }

    FillLEDS(fill_color_);
    FastLED.show();
}

void FillLEDS(uint8_t colorIndex) {
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = ColorFromPalette(current_palette_, colorIndex, brightness_,
                                   current_blending_);
    }
}
