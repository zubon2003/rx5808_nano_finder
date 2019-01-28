//必要部品
//1 Arduino(UNO NANO PROMINIどれでも？)
//2 押しボタンスイッチ付きロータリーエンコーダー(インクリメンタル式）例 BANGGOODで売っている KY-040
//3 圧電スピーカー 例 秋月で売ってるムラタPKM17EPP-4001-B0
//4 SSD1306 128x64ドット i2C接続のもの
//5 RX5808(SPI化してあるもの。最近のBanggoodのものならOK)
//6 アンテナ（指向性の高いもの)
//7 5V電源

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "EEPROM.h"

#define screenWidth 128
#define screenHeight 64
#define oledReset 4
Adafruit_SSD1306 display(screenWidth,screenHeight,&Wire,oledReset);

//PIN Assign

// SPI pins for RX5808 control
#define spiDataPin    17 //A3 RX5808のCH1
#define slaveSelectPin  16 //RX5808のCH2
#define spiClockPin   15 //A1 RX5808のCH3
#define rssiPin   14 //A0 RX5808のRSSI

//Rotary Encoder Pins
#define rotaryAPin 9 // D9  
#define rotaryBPin 8 // D8  
#define buttonPin  6 //D6

//Buzzer Pin
#define tonePin    7 //D7

uint16_t ACTUAL_FREQ = 5705;
uint16_t SCALE = 1;
uint8_t  CURRENTSTATE = 0;
uint8_t  LASTSTATE = 0;
uint8_t  MATCHCOUNT = 0;
uint8_t  PATBUFFER = 0;
bool     INPUTMATCH = false;

void setup()
{
  // initialize with the I2C addr 0x3D or 0x3C(for the 128x64)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  // SPI pins for RX5808 control
  pinMode(spiDataPin, OUTPUT);
  pinMode(spiClockPin, OUTPUT);
  pinMode(slaveSelectPin, OUTPUT);

  //Rotary Encoder Pins
  pinMode(rotaryAPin, INPUT_PULLUP);
  pinMode(rotaryBPin, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  if (digitalRead(rotaryAPin)) LASTSTATE |= B10;
  if (digitalRead(rotaryBPin)) LASTSTATE |= B01;

  ACTUAL_FREQ = 5705;
  setChannel();
}

void loop()
{
  uint8_t tmpRssi = 0;
  uint32_t nextTime = 0;
  int8_t encoderState;


  initDisplay();
  printFreq();

  while (1)
  {
    if (buttonPushed()) SCALE = SCALE * 10;
    if (SCALE == 1000) SCALE = 1;

    CURRENTSTATE = 0;
    if (digitalRead(rotaryAPin)) CURRENTSTATE |= B10;
    if (digitalRead(rotaryBPin)) CURRENTSTATE |= B01;

    if (LASTSTATE == CURRENTSTATE) {
      if (!INPUTMATCH) {
        MATCHCOUNT++;
        if (MATCHCOUNT >= 5) {
          // 状態確定
          INPUTMATCH = true;
          encoderState = checkEncoderState(CURRENTSTATE);
          if (encoderState != 0) {
            ACTUAL_FREQ += encoderState * SCALE;
            ACTUAL_FREQ = constrain(ACTUAL_FREQ, 5600, 6000);
            printFreq();
            setChannel();
          }
        }
      }
    }
    else {
      LASTSTATE = CURRENTSTATE;
      MATCHCOUNT = 0;
      INPUTMATCH = false;
    }

    if (nextTime <  millis() | nextTime == 0)
    {
      tmpRssi = mappedRSSI();
      tone(tonePin, 55 + tmpRssi * 10);

      setCursorPrint(tmpRssi);
      display.display();
      nextTime = millis() + 100;
    }
  }
}

uint8_t buttonPushed()
{
  if ( digitalRead(buttonPin) == LOW )
  {
    delay(100);
    if (digitalRead(buttonPin) == LOW) return (1);
  }
  return (0);
}

int8_t checkEncoderState(uint8_t newState) {

  PATBUFFER <<= 2;
  PATBUFFER |= newState;
  if (PATBUFFER == 0x4B) {
    // +PATTERN 0x4B = B01001011 -> 01 00 10 11の順にエンコーダーのピンの状態が変わったらプラス方向
    return 1;
  }

  else if (PATBUFFER == 0x87) {
    // -PATTERN 0x87 = B10000111 -> 10 00 01 11の順にエンコーダーのピンの状態が変わったらマイナス方向
    return -1;
  }

  else {
    return 0;
  }
}

void printFreq()
{
  display.fillRect(0, 0, 127, 40, BLACK);
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(70, 10);
  display.println(ACTUAL_FREQ);
  display.setTextSize(1);
}

void setCursorPrint(uint8_t rssi)
{
  display.setTextSize(2);
  display.fillRect(80, 40, 128, 52, BLACK);

  if (rssi < 10) display.setCursor(80 + 24, 40);
  else if (rssi < 100) display.setCursor(80 + 12, 40);
  else display.setCursor(80, 40);

  display.print(rssi);
  display.fillRect(0, 58, 100, 63, BLACK);
  display.fillRect(0, 58, rssi, 63, WHITE);
}

void initDisplay()
{
  display.clearDisplay();
}

uint16_t mappedRSSI()
{
  uint16_t rawRssi = 0;
  uint16_t mapRssi = 0;
  rawRssi = readRSSI();
  rawRssi = constrain(rawRssi, 100, 300);
  mapRssi = map(rawRssi, 100, 300, 0, 100);
  return (mapRssi);
}

uint16_t readRSSI()
{
  uint16_t rssi = 0;
  rssi = analogRead(rssiPin);
  return (rssi);
}

void setChannel()
{
  setChannelModule(ACTUAL_FREQ);
  delay(10);
}
