#include <Arduino.h>
#include <bluefruit.h>
#include <ble_gap.h>

// shutdown flash memory
#include "Adafruit_SPIFlash.h"
#include "SdFat.h"
#include <SPI.h>
// for flashTransport definition
#include "flash_config.h"

#include <BLEWeighingScale.h>

Adafruit_SPIFlash flash(&flashTransport);

// save power without serial interface...
#define DEBUG

// Weighing Scale (Weight Information Service) helper class instance
BLEWeighingScale bleScale;

// DIS (Device Information Service) helper class instance
BLEDis bledis;

// BAS (Battery Service) helper class instance
BLEBas blebas;    

//---- ADC Battery Monitoring ----------------------------
const int adcPin = PIN_VBAT;

uint16_t setBleWeightFeature();
bool setBleWeightMeasurement(float weightKg);

//------------------------------------------------------------------------------
uint32_t getBatteryVoltage()
//------------------------------------------------------------------------------
{
  analogReference(AR_INTERNAL); // reference voltage 0..3.6V
  analogReadResolution(12);     // analog resolution 12bit
  uint32_t adcValue = analogRead(adcPin);
  adcValue = analogRead(adcPin);
  #ifdef DEBUG
    Serial.print("Battery: ");
    Serial.println(adcValue);
  #endif
  return adcValue;  // mV
}

/**
 * Start BLE Advertising
 */
void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 1600);   // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

//---------------------------------------------------------------------------
void stopAdvertising()
//---------------------------------------------------------------------------
{
  #ifdef DEBUG
    Serial.println("Clearing Advertisement Data");
    Serial.println("----------------------------------");
  #endif
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.stop();
}

/**
 * On connect
 */
void connectCallback(uint16_t conn_handle) {
  stopAdvertising();

  int batteryVoltage = getBatteryVoltage();
  int batteryLevel = map(batteryVoltage, 0, 4095, 0, 100);
  #ifdef DEBUG
    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage);
    Serial.print("Battery Level: ");
    Serial.println(batteryLevel);
  #endif
  blebas.write(batteryLevel);
  bleScale.indicate(13.65);
}

/**
 * On disconnect
 */
void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  startAdvertising();
}

/**
 * Setup the Feather
 */
void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  Bluefruit.begin();
  Bluefruit.setName("BleWeightDevice");
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // Configure and Start the Device Information Service
  #ifdef DEBUG
    Serial.println("Configuring the Device Information Service");
  #endif
  bledis.setManufacturer("SWASILAN ENGINEERING");
  bledis.setModel("NRF52840-NAU7802-001");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  #ifdef DEBUG
    Serial.println("Configuring the Battery Service");
  #endif
  blebas.begin();
  blebas.write(100);

  bleScale.begin();
  bleScale.indicate(13.65);

  startAdvertising();
}

//---------------------------------------------------------------------------
void loop() 
//---------------------------------------------------------------------------
{  
  if ( Bluefruit.connected() ) {
    #ifdef DEBUG
      Serial.println("Connected");
    #endif
    bleScale.indicate(13.65);
  } else {
    #ifdef DEBUG
      Serial.println("Not connected");
    #endif
  }
  delay(2000);
}
