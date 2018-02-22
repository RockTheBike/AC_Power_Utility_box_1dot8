/**** Split-rail Pedalometer
* Arduino code to run the sLEDgehammer
* ver. 1.9
* Written by:
* Thomas Spellman <thomas@thosmos.com>
* Jake <jake@spaz.org>
* Paul@rockthebike.com
*
* Notes:
* 1.6 - moved version to the top, started protocol of commenting every change in file and in Git commit
* 1.7 - jake 6-21-2012 disable minusalert until minus rail is pedaled at least once (minusAlertEnable and startupMinusVoltage)
* 1.8 -- FF added annoying light sequence for when relay fails or customer bypasses protection circuitry.+
* 1.9 Started using Chinese inverters labeled "1000W Pure Sine Inverter with AC and DC voltage output screens.
*/

char versionStr[] = "AC Utility Box with wattmeter";

const int pwm = 0;
const int onoff = 1;

const int numLevels = 5;

const int numPins = 4;
int pin[numPins] = {3, 5, 6, 9};

// voltages at which to turn on each level
float levelVolt[numLevels] = {21.8, 22.8, 25.8, 28.6, 29.5};
int levelMode=0; // 0 = off, 1 = blink, 2 = steady
int whichPin[]={3, 5, 6, 9, 9};
int levelType[numLevels] = {pwm, pwm, pwm, pwm, pwm};

#define VOLTPIN A0 // Voltage Sensor Input
#define AMPSPIN A3 // Current Sensor Pin
#define NOISYZERO 1.0  // assume any smaller measurement should be 0
#define OVERSAMPLING 25.0 // analog oversampling
#define AMPCOEFF 3.5 // this is an oddball ebay current sensor, tested to 35 amps
#define AMPOFFSET 515.0 // when current sensor is at 0 amps this is the ADC value
float wattage = 0; // what is our present measured wattage
#define WATTHOUR_DISPLAY_PIN    4
#define WATTHOUR_DISPLAY_PIXELS (8*32)
#define WATTHOUR_DISPLAY_MIN_VOLTAGE 12.0 // turn off display below this voltage
// bottom right is first pixel, goes up 8, left 1, down 8, left 1...
// https://www.aliexpress.com/item/8-32-Pixel/32225275406.html
#include <Adafruit_NeoPixel.h>
#include "font1.h"
uint32_t fontColor = Adafruit_NeoPixel::Color(175,157,120);
uint32_t backgroundColor = Adafruit_NeoPixel::Color(0,0,0);
Adafruit_NeoPixel wattHourDisplay = Adafruit_NeoPixel(WATTHOUR_DISPLAY_PIXELS, WATTHOUR_DISPLAY_PIN, NEO_GRB + NEO_KHZ800);

const int relayPin=2;
const float voltcoeff = 13.25;  // larger numer interprets as lower voltage

//MAXIMUM VOLTAGE TO GIVE LEDS

const float maxVoltLEDs = 21.0; //LED
const float maxVoltPlusRail = 30.0;  //
const float dangerVoltPlusRail = 30.8;


//Hysteresis variables
const float plusRailComeBackInVoltage = 28.2;
const float plusRailComeBackInVoltagetwelveVoltMode = 13.7;
int plusRailHysteresis=0;
int dangerVoltageState=0;
//int minusRailHysteresis=0;

// vars to store temp values
int adcvalue = 0;

//Voltage related variables.
float voltage = 0;
//float minusRailVoltage=0;
//float startupMinusVoltage=0;  // inital read of minus rail, to see if minus is being pedaled at all

//Current related variables
float plusRailAmps=0;
float ACAmps=0;
int plusRailAmpsRaw;
int ACAmpsRaw;

// on/off state of each level (for use in status output)
int state[numLevels];
int desiredState[numPins];

const int AVG_CYCLES = 50; // average measured voltage over this many samples
const int DISPLAY_INTERVAL_MS = 500; // when auto-display is on, display every this many milli-seconds
const int VICTORYTIME=4000;

int readCount = 0; // for determining how many sample cycle occur per display interval

// vars for current PWM duty cycle
int pwmValue = 255;
boolean updatePwm = false;

int blinkState=0;
int fastBlinkState=0;
unsigned long time = 0;
unsigned long currentTime = 0;
unsigned long lastFastBlinkTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long timeDisplay = 0;

// current display level
int level = -1;

// var for looping through arrays
int i = 0;
int x = 0;
int y = 0;


void setup() {
  Serial.begin(57600);
  wattHourDisplay.begin();
  wattHourDisplay.show();
  Serial.println(versionStr);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin,LOW);
  for(i = 0; i < numPins; i++) pinMode(pin[i],OUTPUT);
  getVoltages();
}

boolean levelLock = false;
int senseLevel = -1;

void loop() {
  getVoltages();
  getCurrent();
  //setpwmvalue(); // leave at 255 (default full power)
  readCount++;
  time = millis();
  currentTime=millis();
  if (((currentTime - lastBlinkTime) > 600) && blinkState==1){
    blinkState=0;
    lastBlinkTime=currentTime;
  } else if (((currentTime - lastBlinkTime) > 600) && blinkState==0){
     blinkState=1;
     lastBlinkTime=currentTime;
  }
  if (((currentTime - lastFastBlinkTime) > 180) && fastBlinkState==1){
    fastBlinkState=0;
    lastFastBlinkTime=currentTime;
  } else if (((currentTime - lastFastBlinkTime) > 150) && fastBlinkState==0){
     fastBlinkState=1;
     lastFastBlinkTime=currentTime;
  }
  if (voltage > maxVoltPlusRail){
    digitalWrite(relayPin, HIGH);
    plusRailHysteresis=1;
  }
  if (plusRailHysteresis==1 && voltage < plusRailComeBackInVoltage){
    digitalWrite(relayPin, LOW);
    plusRailHysteresis=0;
  }
  if (voltage > dangerVoltPlusRail){
    dangerVoltageState=1;
  } else {
    dangerVoltageState=0;
  }
  senseLevel = -1;
  if (voltage <=levelVolt[0]){ // if voltage is below first level
    senseLevel=0;
    //desiredState[0]=1; // blink first row of LEDs
  } else {
    for(i = 0; i < numLevels; i++) {
      if(voltage >= levelVolt[i]){ // turn on rows if voltage is above their level
        senseLevel = i;
        desiredState[i]=2;
        levelMode=2;
      } else desiredState[i]=0;
      //if (voltage >= levelVolt[1]) desiredState[0]=0; // turn off red row if green is lit up
    }
  }
  level=senseLevel;
  if (level == (numLevels-1)){
    desiredState[level-1] = 1; //workaround to blink white
  }
  if (dangerVoltageState){
    for(i = 0; i < numLevels; i++) {
      desiredState[i] = 3;
    }
  }
  for(i = 0; i < numPins; i++) {
    if(levelType[i]==pwm) {
      if(desiredState[i]==2){
        analogWrite(whichPin[i], pwmValue);
        state[i] = 2;
      }
      else if (desiredState[i]==0){
        analogWrite(whichPin[i], 0);
        state[i] = 0;}
      else if (desiredState[i]==1 && blinkState==1){
        analogWrite(whichPin[i], pwmValue);
        state[i] = 1;}
      else if (desiredState[i]==1 && blinkState==0){
        analogWrite(whichPin[i], 0);
        state[i] = 1;}
      else if (desiredState[i]==3 && fastBlinkState==1){
        analogWrite(whichPin[i], pwmValue);
        state[i] = 1;}
      else if (desiredState[i]==3 && fastBlinkState==0){
        analogWrite(whichPin[i], 0);
        state[i] = 1;
      }
    }
  } //end for
  if(time - timeDisplay > DISPLAY_INTERVAL_MS){
    timeDisplay = time;
    printDisplay();
    updateDisplay();
    readCount = 0;
  }
} // end loop()

void setpwmvalue() {

  // The effective voltage of a square wave is
  // Veff = Vin * sqrt(k)
  // where k = t/T is the duty cycle

  // If you want to make 12V from a 20V input you have to use
  // k = (Veff/Vin)^2 = (12/20)^2 = 0.36

  int newVal = 0;

  if (voltage <= maxVoltLEDs) {
    newVal = 255.0;
  }
  else {
    newVal = maxVoltLEDs / voltage * 255.0;
  }

// if(voltage <= 24) {
// pwmvalue24V = 255.0;
// }
// else {
// pwmvalue24V = sq(24 / voltage) * 255.0;
// }

  if(newVal != pwmValue) {
    pwmValue = newVal;
    updatePwm = true;
  }
}

void getCurrent(){
  plusRailAmpsRaw = 0; // reset adder
  for(int j = 0; j < OVERSAMPLING; j++) plusRailAmpsRaw += analogRead(AMPSPIN) - AMPOFFSET;
  plusRailAmps = ((float)plusRailAmpsRaw / OVERSAMPLING) / AMPCOEFF * -1; // current flow from PLUSOUT to PLUSRAIL
  if( plusRailAmps < NOISYZERO ) plusRailAmps = 0; // we assume anything near or below zero is a reading error
  wattage = voltage * plusRailAmps;
}

void getVoltages(){

 //first two lines are for + rail
  adcvalue = analogRead(VOLTPIN);
  voltage = average(adc2volts(adcvalue), voltage);

}

//Future simplification / generalization
void protectUltracap(int whichRail){

  // Turn OFF the relay to protect either the minus or the plus rail

}

//Future simplification / generalization
void restoreUltracapAfterHysteresis(){
  // Turn ON the relay for normal operation
}

float average(float val, float avg){
  if(avg == 0)
    avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

static int volts2adc(float v){
 /* voltage calculations
*
* Vout = Vin * R2/(R1+R2), where R1 = 100k, R2 = 10K
* 30V * 10k/110k = 2.72V // at ADC input, for a 55V max input range
*
* Val = Vout / 5V max * 1024 adc val max (2^10 = 1024 max vaue for a 10bit ADC)
* 2.727/5 * 1024 = 558.4896
*/
//int led3volts0 = 559;

/* 24v
* 24v * 10k/110k = 2.181818181818182
* 2.1818/5 * 1024 = 446.836363636363636
*/
//int led2volts4 = 447;

//adc = v * 10/110/5 * 1024 == v * 18.618181818181818;

return v * voltcoeff;
}



float adc2volts(float adc){
  // v = adc * 110/10 * 5 / 1024 == adc * 0.0537109375;
  return adc * (1 / voltcoeff); // 55 / 1024 = 0.0537109375;
}

void printDisplay(){
  Serial.print("volts: ");
  Serial.print(voltage);
  Serial.print(", DC Amps: ");
  Serial.print(plusRailAmps);
  Serial.print(" (");
  Serial.print((float)plusRailAmpsRaw / OVERSAMPLING,1);
  Serial.print(") (");
  Serial.print(analogRead(AMPSPIN));
  Serial.print(")");
  Serial.print(", DC Watts: ");
  Serial.print(wattage);

  Serial.print(", Levels ");
  for(i = 0; i < numLevels; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(state[i]);
    Serial.print(", ");
  }
  Serial.println();
}

void updateDisplay() {
  char buf[]="    "; // stores the number we're going to display
  //sprintf(buf,"%4d",millis()/100);// for testing display
  if (voltage > WATTHOUR_DISPLAY_MIN_VOLTAGE) sprintf(buf,"%4d",((int)(wattage)/10UL) * 10UL); // quantize to tens of watts
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
