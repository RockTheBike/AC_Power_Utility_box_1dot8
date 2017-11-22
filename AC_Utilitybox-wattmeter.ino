/* AC utility box with wattmeter and pedalometer
 * using addressible LEDs for both */

char versionStr[] = "AC Utility Box with wattmeter & addressible pedalometer";

#define VOLTPIN A0 // Voltage Sensor Pin
#define AMPSPIN A3 // Current Sensor Pin
#define NOISYZERO 1.0  // assume any smaller measurement should be 0
#define OVERSAMPLING 25.0 // analog oversampling
#define AMPCOEFF 8.111
#define AMPOFFSET 510.6 // when current sensor is at 0 amps this is the ADC value
float wattage = 0; // what is our present measured wattage
#define WATTHOUR_DISPLAY_PIN    4
#define WATTHOUR_DISPLAY_PIXELS (8*32)
// bottom right is first pixel, goes up 8, left 1, down 8, left 1...
// https://www.aliexpress.com/item/8-32-Pixel/32225275406.html
#include <Adafruit_NeoPixel.h>
#include "font1.h"
uint32_t fontColor = Adafruit_NeoPixel::Color(175,157,120);
uint32_t backgroundColor = Adafruit_NeoPixel::Color(0,0,0);
Adafruit_NeoPixel wattHourDisplay = Adafruit_NeoPixel(WATTHOUR_DISPLAY_PIXELS, WATTHOUR_DISPLAY_PIN, NEO_GRB + NEO_KHZ800);

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

int adcvalue = 0;

float volts = 0;

float plusRailAmps=0;
int plusRailAmpsRaw;

#define AVG_CYCLES 50 // average measured voltage over this many samples
#define DISPLAY_INTERVAL_MS 500 // when auto-display is on, display every this many milli-seconds

unsigned long time = 0;
unsigned long timeDisplay = 0;

void setup() {
  Serial.begin(57600);
  wattHourDisplay.begin();
  wattHourDisplay.show();
  Serial.println(versionStr);
  pinMode(DISCORELAY, OUTPUT);
  digitalWrite(DISCORELAY,LOW);
  getVoltages();
}

void loop() {
  getVoltages();
  getCurrent();
  doSafety();

  if(time - timeDisplay > DISPLAY_INTERVAL_MS){
    timeDisplay = time;
    printDisplay();
    updateDisplay();
  }
}

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
  plusRailAmps = ((float)plusRailAmpsRaw / OVERSAMPLING) / AMPCOEFF;
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
  Serial.print(wattage);

  Serial.println("");
  Serial.println();
}

void updateDisplay() {
  char buf[]="    "; // stores the number we're going to display
  //sprintf(buf,"%4d",millis()/100);// for testing display
  sprintf(buf,"%4d",((int)(wattage)/10UL) * 10UL); // quantize to tens of watts
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
