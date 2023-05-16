#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
//#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

#include <math.h>

namespace esphome {
namespace mlx90641 {

class MLX90641Component : public PollingComponent, public i2c::I2CDevice {
 public:
  struct MlxDeviceParams{
        int16_t kVdd;
        int16_t vdd25;
        float KvPTAT;
        float KtPTAT;
        uint16_t vPTAT25;
        float alphaPTAT;
        int16_t gainEE;
        float tgc;
        float cpKv;
        float cpKta;
        uint8_t resolutionEE;
        uint8_t calibrationModeEE;
        float KsTa;
        float ksTo[8];
        int16_t ct[8];
        uint16_t alpha[192]; 
        uint8_t alphaScale;   
        int16_t offset[2][192];    
        int8_t kta[192];
        uint8_t ktaScale;    
        int8_t kv[192];
        uint8_t kvScale;
        float cpAlpha;
        int16_t cpOffset;
        float emissivityEE; 
        uint16_t brokenPixel;
    };
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

  void set_ambient_sensor(sensor::Sensor *ambient_sensor) { ambient_sensor_ = ambient_sensor; }
  void set_object_sensor(sensor::Sensor *object_sensor) { object_sensor_ = object_sensor; }
//  void set_frame_sensor(text_sensor::TextSensor *frame_sensor) {frame_sensor_ = frame_sensor; }

  void set_emissivity(float emissivity) { emissivity_ = emissivity; }

 protected:
  int read_1616_(uint16_t address, uint16_t len, uint16_t *data);
  int write_1616_(uint16_t address, uint16_t data);
  int HammingDecode(uint16_t *eeData);
  int ValidateFrameData(uint16_t *frameData);
  int ValidateAuxData(uint16_t *auxData);
  int MLX90641_ExtractParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void MLX90641_CalculateTo(uint16_t *frameData, const MlxDeviceParams *params, float emissivity, float tr, float *result);
  float MLX90641_GetVdd(uint16_t *frameData, const MlxDeviceParams *params);
  float MLX90641_GetTa(uint16_t *frameData, const MlxDeviceParams *params);
  void ExtractVDDParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractPTATParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractGainParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractTgcParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractEmissivityParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractResolutionParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractKsTaParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractKsToParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractAlphaParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractOffsetParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractKtaPixelParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractKvPixelParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  void ExtractCPParameters(uint16_t *eeData, MlxDeviceParams *mlx90641);
  int ExtractDeviatingPixels(uint16_t *eeData, MlxDeviceParams *mlx90641);
  int CheckEEPROMValid(uint16_t *eeData);

  sensor::Sensor *ambient_sensor_{nullptr};
  sensor::Sensor *object_sensor_{nullptr};
//  text_sensor::TextSensor *frame_sensor_{nullptr};

  float emissivity_{NAN};
};
}  // namespace mlx90641
}  // namespace esphome
