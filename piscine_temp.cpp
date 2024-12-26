#include "piscine_temp.h"
#include "esphome/core/log.h"

#include <string>

namespace esphome {
namespace piscine_temp {

static const char *const TAG = "piscine.temp.sensor";

#define RST 23
#define DIO0 26

PiscineTempSensor* my;

static void rx_callback(sx127x *device, uint8_t *data, uint16_t data_length) {

  ESP_LOGI(TAG, "sx127x receive data");

  if (data_length != 18)
    return;

  struct __attribute__((packed)) info_t {
    uint8_t magic[3];
    uint8_t mode;
    uint8_t type;
    uint16_t swversion;
    uint8_t battery;
    uint16_t devide_id;
    uint16_t temp;
    uint16_t temp2;
    uint16_t hum;
    uint16_t crc;
  };

  const info_t *info = (info_t*)data;
  if (!(info->magic[0]==0xd3 && info->magic[1]==0x91 && info->magic[2]==0x0f))
    return;

  float temp = ((float)info->temp)/10.0;

  ESP_LOGI(TAG, "mode=%u type=%u sw=%u batt=%u id=%u temp=%0.1f temp2=%0.1f hum=%0.1f crc=%u",
    info->mode, info->type, info->swversion, info->battery, info->devide_id, temp, ((float)info->temp2)/10.0, ((float)info->hum)/10.0, info->crc);

  my->publish_state(temp);
}

void PiscineTempSensor::setup() {
  config.mosi_io_num = 27;
  config.miso_io_num = 19;
  config.sclk_io_num = 5;
  config.quadwp_io_num = -1;
  config.quadhd_io_num = -1;
  config.max_transfer_sz = 0;

  dev_cfg.command_bits = 0;
  dev_cfg.address_bits = 8;
  dev_cfg.dummy_bits = 0;
  dev_cfg.mode = 0;
  dev_cfg.clock_speed_hz = 4000000ul;
  dev_cfg.spics_io_num = 18;
  dev_cfg.queue_size = 16;

  // reset
  gpio_set_direction((gpio_num_t) RST, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t) RST, 0);
  delay(5);
  gpio_set_level((gpio_num_t) RST, 1);
  delay(5);
  ESP_LOGI(TAG, "sx127x was reset");

  esp_err_t ret;

  #define CHECK_OR_FAIL(x) if ((x) != ESP_OK) mark_failed();

  CHECK_OR_FAIL( spi_bus_initialize(HSPI_HOST, &config, 1) );
  CHECK_OR_FAIL( spi_bus_add_device(HSPI_HOST, &dev_cfg, &spi_device) );
  CHECK_OR_FAIL( sx127x_create(spi_device, &device) );
  CHECK_OR_FAIL( sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, &device) );
  CHECK_OR_FAIL( sx127x_set_frequency(433920000, &device) );
  CHECK_OR_FAIL( sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, &device) );

  CHECK_OR_FAIL( sx127x_fsk_ook_set_bitrate(10000.0, &device) );
  CHECK_OR_FAIL( sx127x_fsk_set_fdev(15000.0, &device) );

  CHECK_OR_FAIL( sx127x_fsk_ook_rx_set_afc_auto(false, &device) );
  CHECK_OR_FAIL( sx127x_fsk_ook_rx_set_afc_bandwidth(10000.0, &device) );
  CHECK_OR_FAIL( sx127x_fsk_ook_rx_set_bandwidth(60000.0, &device) );

  uint8_t syncWord[] = {0x2d, 0xd4};
  CHECK_OR_FAIL( sx127x_fsk_ook_set_syncword(syncWord, 2, &device) );

  CHECK_OR_FAIL( sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0, 0, &device) );
  CHECK_OR_FAIL( sx127x_fsk_ook_set_crc(SX127X_CRC_NONE, &device) );

  CHECK_OR_FAIL( sx127x_fsk_ook_rx_set_trigger(SX127X_RX_TRIGGER_PREAMBLE, &device) );
  CHECK_OR_FAIL( sx127x_fsk_ook_rx_set_rssi_config(SX127X_8, 0, &device) );
  CHECK_OR_FAIL( sx127x_fsk_ook_rx_set_preamble_detector(true, 2, 0x0A, &device) );

  CHECK_OR_FAIL( sx127x_fsk_ook_set_preamble_type(SX127X_PREAMBLE_AA, &device) );

  CHECK_OR_FAIL( sx127x_set_preamble_length(1800, &device) );

  CHECK_OR_FAIL( sx127x_fsk_ook_set_packet_encoding(SX127X_NRZ, &device) );
  CHECK_OR_FAIL( sx127x_fsk_ook_set_packet_format(SX127X_FIXED, 18, &device) );
  CHECK_OR_FAIL( sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, &device) );
  CHECK_OR_FAIL( sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, &device) );

  my = this;
  sx127x_rx_set_callback(rx_callback, &device);

  pinMode(DIO0, INPUT_PULLDOWN);

  CHECK_OR_FAIL(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, &device));

  ESP_LOGI(TAG, "sx127x radio setup ok.");

  #undef CHECK_OR_FAIL
}

void PiscineTempSensor::loop() {
  if (digitalRead(DIO0)) {
    sx127x_handle_interrupt(&device);
  }
};

void PiscineTempSensor::dump_config() {

};


}  // namespace piscine_temp
}  // namespace esphome