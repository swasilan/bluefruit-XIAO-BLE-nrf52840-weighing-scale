#include <Arduino.h>
#include <bluefruit.h>
#include <ble_gap.h>

// shutdown flash memory
#include "Adafruit_SPIFlash.h"
#include "SdFat.h"
#include <SPI.h>
// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

// save power without serial interface...
#define DEBUG

// GATT Service - 0x181D - Weight Sensing
BLEService        bleWeightService = BLEService(UUID16_SVC_WEIGHT_SCALE);

BLECharacteristic bleWeightFeature = BLECharacteristic(UUID16_CHR_WEIGHT_SCALE_FEATURE);
BLECharacteristic bleWeightMeasurement = BLECharacteristic(UUID16_CHR_WEIGHT_MEASUREMENT);

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
 * Set up the Environmental Sensing Service (BLE)
 */
void setupWeightService() {
  bleWeightService.begin();

  bleWeightFeature.setProperties(CHR_PROPS_READ);
  bleWeightFeature.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bleWeightFeature.setTempMemory();
  bleWeightFeature.setFixedLen(4);
  bleWeightFeature.begin();

  bleWeightMeasurement.setProperties(CHR_PROPS_INDICATE);
  bleWeightMeasurement.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bleWeightMeasurement.setFixedLen(3);
  bleWeightMeasurement.begin(); 

  setBleWeightFeature();
}

/**
 * Start BLE Advertising
 */
void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleWeightService);
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

  setBleWeightMeasurement(13.65);
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

  setupWeightService();
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
    setBleWeightMeasurement(13.5);
  } else {
    #ifdef DEBUG
      Serial.println("Not connected");
    #endif
  }
  delay(2000);
}


/*
 * Initializes our BLE Characteristic
 * that describes what features our Weight device provides.
 */
uint16_t setBleWeightFeature() {
  unsigned long val = 0;          // field value
  unsigned char bytes[4] = {0};   // field value, encoded for transmission.
  
  /*
   * Flags.
   * 
   * bit 0 = 0 means no Time stamp provided.
   * bit 1 = 0 means multiple scale users are NOT supported.
   * bit 2 = 0 means no BMI supported.
   * bits 3..6 = 7 means 0.01 kg resolution (that says nothing about accuracy).
   * bits 7..9 = 0 means height resolution unspecified.
   * bits 10..31 are reserved and should be set to 0.
   */

  val |= 0x0 << 0;
  val |= 0x0 << 1;
  val |= 0x0 << 2;
  val |= 0x7 << 3;
  val |= 0x0 << 7;

  // BLE GATT multi-byte values are encoded Least-Significant Byte first.
  bytes[0] = (unsigned char) val;
  bytes[1] = (unsigned char) (val >> 8);
  bytes[2] = (unsigned char) (val >> 16);
  bytes[3] = (unsigned char) (val >> 24);

  return bleWeightFeature.write(bytes, sizeof(bytes));
}

/*
 * Sets our BLE Characteristic given a weight measurement (in kg)
 * 
 * See https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.weight_measurement.xml
 *  for details of the encoding of weight in BLE.
 */
bool setBleWeightMeasurement(float weightKg) {
  unsigned char flags = 0;      // description of the weight
  uint16_t newVal = 0;          // field value: the weight in BLE format
  unsigned char bytes[3] = {0}; // data, encoded for transmission.
  
  /*
   * Set the flags:
   * bit 0 = 0 means we're reporting in SI units (kg and meters)
   * bit 1 = 0 means there is no time stamp in our report
   * bit 2 = 0 means User ID is NOT in our report
   * bit 3 = 0 means no BMI and Height are in our report
   * bits 4..7 are reserved, and set to zero.
   */

  flags |= 0x0 << 0;
  flags |= 0x0 << 1;
  flags |= 0x0 << 2;
  flags |= 0x0 << 3;

  // Convert the weight into BLE representation
  newVal = (uint16_t) ((weightKg * 1000.0 / 5.0) + 0.5);

  /*
   * Because we are a continuous, periodic measurement device,
   * we set the BLE value and notify any BLE Client every time
   * we make a measurement, even if the value hasn't changed.
   * 
   * If instead we were designed to keep a BLE Client informed
   * of the changing value, we'd set the value only if the value changes.
   */

  bytes[0] = flags;

  // BLE GATT multi-byte values are encoded Least-Significant Byte first.
  bytes[1] = (unsigned char) newVal;
  bytes[2] = (unsigned char) (newVal >> 8);

  return bleWeightMeasurement.indicate(bytes, sizeof(bytes));
}