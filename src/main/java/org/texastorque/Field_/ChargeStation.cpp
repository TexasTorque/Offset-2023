#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <FastLED.h>

#define LED_PIN     7
#define NUM_LEDS    70
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;
static int fillColor = 0, green = 100, red = 0;

double timerMillis = 0;
boolean balanced = false;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

double start_time_;
 uint8_t brightness = 255;


void setup(void)
{
  Serial.begin(9600);
    if (!accel.begin())
    {
      Serial.println("No ADXL345 sensor detected.");
      while (1);
    }

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );

  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;

  start_time_ = millis(); 
}
void loop(void)
{

  
  float elapsed = millis() - start_time_;
  if (elapsed < 2.5e3) {
    
    float f = fmod(elapsed / 250, 2.0);
    brightness = f <= 1 ? 0 : 255;
    Serial.println(f);
    fillColor = green;
    
  } else {
    
    sensors_event_t event;
    accel.getEvent(&event);
  
    if (event.acceleration.x < 1.2 && event.acceleration.x > 0) {
      if (!balanced) timerMillis = millis();
  
      balanced = true;
  
      if (millis() - timerMillis > 50) {
        Serial.print("Times up");
        fillColor = green;
      }
    }
    else {
      fillColor = red;
      balanced = false;
    }
      Serial.println(event.acceleration.x);

  }

  FillLEDsFromPaletteColors( fillColor);
 

  FastLED.show();


}

void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
  for ( int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
  }
}
