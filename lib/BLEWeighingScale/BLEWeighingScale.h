#ifndef BLEWEIGHINGSCALE_H_
#define BLEWEIGHINGSCALE_H_

#include "bluefruit_common.h"

#include "BLECharacteristic.h"
#include "BLEService.h"

class BLEWeighingScale : public BLEService
{
  public:
    BLEWeighingScale(void);

    virtual err_t begin(void);

    bool indicate (float weightKg);

  protected:
    BLECharacteristic _weightFeature;
    BLECharacteristic _weightMeasurement;

  private:
    uint16_t setBleWeightFeature(void);
};

#endif /* BLEWEIGHINGSCALE_H_ */