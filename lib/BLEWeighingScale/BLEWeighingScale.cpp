#include "BLEWeighingScale.h"

BLEWeighingScale::BLEWeighingScale() : 
    BLEService(UUID16_SVC_WEIGHT_SCALE), 
    _weightFeature(UUID16_CHR_WEIGHT_SCALE_FEATURE), 
    _weightMeasurement(UUID16_CHR_WEIGHT_MEASUREMENT)
{

}

err_t BLEWeighingScale::begin(void) {
    // Invoke base class begin()
    VERIFY_STATUS( BLEService::begin() );

    _weightFeature.setProperties(CHR_PROPS_READ);
    _weightFeature.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    _weightFeature.setTempMemory();
    _weightFeature.setFixedLen(4);
    VERIFY_STATUS( _weightFeature.begin() );

    _weightMeasurement.setProperties(CHR_PROPS_INDICATE);
    _weightMeasurement.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    _weightMeasurement.setFixedLen(3);
    VERIFY_STATUS( _weightMeasurement.begin() ); 

    //setBleWeightFeature();

    return ERROR_NONE;
}

bool BLEWeighingScale::indicate(float weightKg) {
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

    return _weightMeasurement.indicate(bytes, sizeof(bytes));
}

/*
 * Initializes our BLE Characteristic
 * that describes what features our Weight device provides.
 */
uint16_t BLEWeighingScale::setBleWeightFeature(void) {
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

    return _weightFeature.write(bytes, sizeof(bytes));
}