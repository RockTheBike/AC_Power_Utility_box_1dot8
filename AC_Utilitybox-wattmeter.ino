#define VOLTLEDSTRIPPIN         13 // what pin the data input to the voltage LED strip is connected to
#define WATTHOUR_DISPLAY_PIN    4  // what pin the 8x32 addressible LED panel data pin connects to
#define PEDALOMETER_TOWER_PIN   12  // what pin the tubular pedaloeter tower data pin connects to

/* AC utility box with wattmeter and pedalometer
 * using addressible LEDs for both */

#include <Adafruit_NeoPixel.h>
char versionStr[] = "AC Utility Box with wattmeter & addressible pedalometer";

#define VOLTPIN A0 // Voltage Sensor Pin
#define AMPSPIN A3 // Current Sensor Pin
#define NOISYZERO 1.0  // assume any smaller measurement should be 0
#define OVERSAMPLING 25.0 // analog oversampling
#define AMPCOEFF 8.74
#define AMPOFFSET 507.0 // when current sensor is at 0 amps this is the ADC value
float wattage = 0; // what is our present measured wattage
#define NUM_VOLTLEDS 48 // four 12-LED strips side by side, facing the same direction
Adafruit_NeoPixel voltLedStrip = Adafruit_NeoPixel(NUM_VOLTLEDS, VOLTLEDSTRIPPIN, NEO_GRB + NEO_KHZ800);

// levels at which each LED turns green (normally all red unless below first voltage)
const float ledLevels[12+1] = { // the pedalometer is four strips of 12 side by side...
  22.0, 22.5, 23.0, 23.5, 24.0, 24.5, 25.0, 25.5, 26.0, 26.5, 27.0, 27.5, 28.0 };

#define WATTHOUR_DISPLAY_PIXELS (8*32)
// bottom right is first pixel, goes up 8, left 1, down 8, left 1...
// https://www.aliexpress.com/item/8-32-Pixel/32225275406.html
#include "font1.h"
uint32_t fontColor = Adafruit_NeoPixel::Color(175,157,120);
uint32_t backgroundColor = Adafruit_NeoPixel::Color(0,0,0);
Adafruit_NeoPixel wattHourDisplay = Adafruit_NeoPixel(WATTHOUR_DISPLAY_PIXELS, WATTHOUR_DISPLAY_PIN, NEO_GRB + NEO_KHZ800);

#define PEDALOMETER_TOWER_PIXELS        120 // number of pixels in RAM array
#define PEDALOMETER_TOWER_MULTIPLES     6 // number of times RAM array / physical LEDs repeat
Adafruit_NeoPixel pedalometerTower = Adafruit_NeoPixel(PEDALOMETER_TOWER_PIXELS, PEDALOMETER_TOWER_PIN, NEO_GRB + NEO_KHZ800, PEDALOMETER_TOWER_MULTIPLES); // requires RTB fork of Adafruit_NeoPixel library

#define WATTDISPLAYVOLTAGE 19.5 // below this voltage, watthour display is blanked
#define LEDBRIGHTNESS 127 // brightness of addressible LEDs (0 to 255)
#define BLINK_PERIOD 1200
#define FAST_BLINK_PERIOD 300
uint32_t red = Adafruit_NeoPixel::Color(LEDBRIGHTNESS,0,0); // load these handy Colors
uint32_t green = Adafruit_NeoPixel::Color(0,LEDBRIGHTNESS,0);
uint32_t blue = Adafruit_NeoPixel::Color(0,0,LEDBRIGHTNESS);
uint32_t white = Adafruit_NeoPixel::Color(80,80,80); // reduced brightness to keep power consumption on 5V line under control
uint32_t dark = Adafruit_NeoPixel::Color(0,0,0);

#define DISCORELAY 2 // relay cutoff output pin // NEVER USE 13 FOR A RELAY
#define VOLTCOEFF 13.36  // larger number interprets as lower voltage
#define STATE_OFF 0
#define STATE_ON 1
#define STATE_BLINK_LOW 2
#define STATE_BLINK_HIGH 3
#define STATE_RAMP 4

#define MAX_VOLTS 30.0 // when to open the safety relay
#define RECOVERY_VOLTS 23.0 // when to close the safety relay
#define DANGER_VOLTS 30.8  // when to fast-flash white (slow-flash above last ledLevels)
bool dangerState = false;
int lastLedLevel = 0; // for LED strip hysteresis
int nowLedLevel = 0; // for LED strip
#define LEDLEVELHYSTERESIS 0.6 // how many volts of hysteresis for gas gauge

int adcvalue = 0;

float volts = 0;

float plusRailAmps=0;
int plusRailAmpsRaw;

#define AVG_CYCLES 50 // average measured voltage over this many samples
#define DISPLAY_INTERVAL_MS 500 // when auto-display is on, display every this many milli-seconds

unsigned long timeDisplay = 0;

void setup() {
  pedalometerTower.begin(); // initialize the addressible LEDs
  pedalometerTower.show(); // clear their state
  voltLedStrip.begin(); // initialize the addressible LEDs
  voltLedStrip.show(); // clear their state
  Serial.begin(57600);
  wattHourDisplay.begin();
  wattHourDisplay.show();
  Serial.print(versionStr);
  Serial.print(" AMPOFFSET:");
  Serial.print(AMPOFFSET);
  Serial.print(" AMPCOEFF:");
  Serial.println(AMPCOEFF);
  pinMode(DISCORELAY, OUTPUT);
  digitalWrite(DISCORELAY,LOW);
  getVoltages();
}

void loop() {
  getVoltages();
  getCurrent();
  doSafety();
  doLeds();

  if(millis() - timeDisplay > DISPLAY_INTERVAL_MS){
    timeDisplay = millis();
    printDisplay();
    updateDisplay();
  }
}

void setPedalometersLevel(char levNum, uint32_t pixelColor) { // 12x4 boxlid array AND 60x12 tower pedalometer
  voltLedStrip.setPixelColor(levNum,pixelColor); // boxlid array is four 12-strips side-by-side
  voltLedStrip.setPixelColor(12+levNum,pixelColor);
  voltLedStrip.setPixelColor(24+levNum,pixelColor);
  voltLedStrip.setPixelColor(36+levNum,pixelColor);
  for (char led = 0; led < 5; led++) { // 60 LEDs high = 5 LEDs per level
    pedalometerTower.setPixelColor(levNum*5+led,pixelColor); // tower is 60 lights up, 60 down, repeated 6 times by pedalometerTower.show()
    pedalometerTower.setPixelColor((119-levNum*5)-led,pixelColor); // this is the 60 down side
  }
}

void doLeds(){
  bool blinkState = millis() % BLINK_PERIOD > BLINK_PERIOD / 2;
  bool fastBlinkState = millis() % FAST_BLINK_PERIOD > FAST_BLINK_PERIOD / 2;

  nowLedLevel = 0; // init value for this round
  for(char levNum = 0; levNum < 12; levNum++) { // go through all but the last voltage in ledLevels[]
    setPedalometersLevel(levNum,dark);  // default to dark
    if (volts > ledLevels[12]) { // if voltage beyond highest level
      if (blinkState) { // make the lights blink
        setPedalometersLevel(levNum,white);  // blinking white
      } else {
        setPedalometersLevel(levNum,red);  // blinking red
      }
    }
    if (volts > ledLevels[levNum]) { // but if enough voltage
      nowLedLevel = levNum+1; // store what level we light up to
    }
  }

  if (nowLedLevel > 0 && volts <= ledLevels[12]) { // gas gauge in effect
    if ((volts + LEDLEVELHYSTERESIS > ledLevels[nowLedLevel]) && (lastLedLevel == nowLedLevel+1)) {
        nowLedLevel = lastLedLevel;
      } else {
        lastLedLevel = nowLedLevel;
      }
    uint32_t pixelColor;
    for(int i = 0; i < nowLedLevel; i++) { // gas gauge effect
      if (nowLedLevel < 5) {
        pixelColor = red;
      } else if (nowLedLevel > 12) {
        if (blinkState) { // blinking white
          pixelColor = white;
        } else {
          pixelColor = dark;
        }
      } else if (nowLedLevel > 11) {
        pixelColor = white;
      } else {
        pixelColor = green;
        if (i >= 10) pixelColor = white; // override with white for levels 10 and 11
      }
      setPedalometersLevel(i,pixelColor);
    }
  } else {
  lastLedLevel = 0; // don't confuse the hysteresis
  }

  if (dangerState){ // in danger fastblink white
    for(int i = 0; i < 12; i++) {
      if (fastBlinkState) { // make the lights blink FAST
        setPedalometersLevel(i,white);  // blinking white
      } else {
        setPedalometersLevel(i,red);  // blinking red
      }
    }
  }
  voltLedStrip.show(); // actually update the LED strip
  pedalometerTower.show(); // actually update the LED strip
} // END doLeds()

void doSafety() {
  if (volts > MAX_VOLTS){
    digitalWrite(DISCORELAY, HIGH);
  }

  if (volts < RECOVERY_VOLTS){
    digitalWrite(DISCORELAY, LOW);
  }

  dangerState = (volts > DANGER_VOLTS);
}

void getCurrent(){
  plusRailAmpsRaw = 0; // reset adder
  for(int j = 0; j < OVERSAMPLING; j++) plusRailAmpsRaw += analogRead(AMPSPIN) - AMPOFFSET;
  plusRailAmps = ((float)plusRailAmpsRaw / OVERSAMPLING) / AMPCOEFF * -1; // negative
  if( plusRailAmps < NOISYZERO ) plusRailAmps = 0; // we assume anything near or below zero is a reading error
  wattage = volts * plusRailAmps;
}

void getVoltages(){
  adcvalue = analogRead(VOLTPIN);
  volts = average(adc2volts(adcvalue), volts);
}

float average(float val, float avg){
  if(avg == 0) avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

float adc2volts(float adc){
  // v = adc * 110/10 * 5 / 1024 == adc * 0.0537109375;
  return adc * (1 / VOLTCOEFF); // 55 / 1024 = 0.0537109375;
}

void printDisplay(){
  Serial.print("volts: ");
  Serial.print(volts);
  Serial.print(", DC Amps: ");
  Serial.print(plusRailAmps);
  Serial.print(" (");
  Serial.print((float)plusRailAmpsRaw / OVERSAMPLING,1);
  Serial.print(")");
  Serial.print(", DC Watts: ");
  Serial.println(wattage);
}

void updateDisplay() {
  char buf[]="    "; // stores the number we're going to display
  //sprintf(buf,"%4d",millis()/100);// for testing display
  if (volts >= WATTDISPLAYVOLTAGE) { // if voltage above minimum
    sprintf(buf,"%4d",((int)(wattage)/10UL) * 10UL); // quantize to tens of watts
  } // otherwise buf will remain blank as initialized
  writeWattHourDisplay(buf);
}

void writeWattHourDisplay(char* text) {
#define DISPLAY_CHARS   4 // number of characters in display
#define FONT_W 8 // width of font
#define FONT_H 8 // height of font
  for (int textIndex=0; textIndex<DISPLAY_CHARS; textIndex++) {
    char buffer[FONT_H][FONT_W]; // array of horizontal lines, top to bottom, left to right
    for(int fontIndex=0; fontIndex<sizeof(charList); fontIndex++){ // charList is in font1.h
      if(charList[fontIndex] == text[textIndex]){ // if fontIndex is the index of the desired letter
        int pos = fontIndex*FONT_H; // index into CHL where the character starts
        for(int row=0;row<FONT_H;row++){ // for each horizontal row of pixels
          memcpy_P(buffer[row], (PGM_P)pgm_read_word(&(CHL[pos+row])), FONT_W); // copy to buffer from flash
        }
      }
    }
    for (int fontXIndex=0; fontXIndex<FONT_W; fontXIndex++) {
      for (int fontYIndex=0; fontYIndex<FONT_H; fontYIndex++) {
        uint32_t pixelColor = buffer[fontYIndex][FONT_W-1-fontXIndex]=='0' ? fontColor : backgroundColor; // here is where the magic happens
        if ((FONT_W*(DISPLAY_CHARS-1-textIndex) + fontXIndex) & 1) { // odd columns are top-to-bottom
          wattHourDisplay.setPixelColor((FONT_H*FONT_W)*(DISPLAY_CHARS-1-textIndex) + fontXIndex*FONT_H +           fontYIndex ,pixelColor);
        } else { // even columns are bottom-to-top
          wattHourDisplay.setPixelColor((FONT_H*FONT_W)*(DISPLAY_CHARS-1-textIndex) + fontXIndex*FONT_H + (FONT_H-1-fontYIndex),pixelColor);
        }
      }
    }
  }
  // wattHourDisplay.setPixelColor((FONT_W-1)*FONT_H+7,fontColor); // light up the decimal point
  // wattHourDisplay.setPixelColor((FONT_W  )*FONT_H+0,backgroundColor); // keep decimal point visible
  // wattHourDisplay.setPixelColor((FONT_W-2)*FONT_H+0,backgroundColor); // keep decimal point visible
  wattHourDisplay.show(); // send the update out to the LEDs
}
