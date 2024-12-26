#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include <driver/spi_common.h>
#include <driver/spi_master.h>

#include "sx127x.h"

namespace esphome {
namespace piscine_temp {

class PiscineTempSensor : public Component, public sensor::Sensor {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  spi_bus_config_t config;
  spi_device_interface_config_t dev_cfg;
  spi_device_handle_t spi_device;
  sx127x device;

};

}  // namespace piscine_temp
}  // namespace esphome