#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace tef::lora {
static constexpr auto kLogTag = "lora";

// Register definitions
static constexpr int kRegFifo = 0x00;
static constexpr int kRegOpMode = 0x01;
static constexpr int kRegFrfMsb = 0x06;
static constexpr int kRegFrfMid = 0x07;
static constexpr int kRegFrfLsb = 0x08;
static constexpr int kRegPaConfig = 0x09;
static constexpr int kRegLna = 0x0c;
static constexpr int kRegFifoAddrPtr = 0x0d;
static constexpr int kRegFifoTxBaseAddr = 0x0e;
static constexpr int kRegFifoRxBaseAddr = 0x0f;
static constexpr int kRegFifoRxCurrentAddr = 0x10;
static constexpr int kRegIrqFlags = 0x12;
static constexpr int kRegRxNbBytes = 0x13;
static constexpr int kRegPktSnrValue = 0x19;
static constexpr int kRegPktRssiValue = 0x1a;
static constexpr int kRegModemConfig1 = 0x1d;
static constexpr int kRegModemConfig2 = 0x1e;
static constexpr int kRegPreambleMsb = 0x20;
static constexpr int kRegPreambleLsb = 0x21;
static constexpr int kRegPayloadLength = 0x22;
static constexpr int kRegModemConfig3 = 0x26;
static constexpr int kRegRssiWideband = 0x2c;
static constexpr int kRegDetectionOptimize = 0x31;
static constexpr int kRegDetectionThreshold = 0x37;
static constexpr int kRegSyncWord = 0x39;
static constexpr int kRegDioMapping1 = 0x40;
static constexpr int kRegDioMapping2 = 0x41;
static constexpr int kRegVersion = 0x42;
// Transceiver modes
static constexpr int kModeLongRangeMode = 0x80;
static constexpr int kModeSleep = 0x00;
static constexpr int kModeStdby = 0x01;
static constexpr int kModeTx = 0x03;
static constexpr int kModeRxContinuous = 0x05;
static constexpr int kModeRxSingle = 0x06;
// PA configuration
static constexpr int kPaBoost = 0x80;
// IRQ masks
static constexpr int kIrqTxDoneMask = 0x08;
static constexpr int kIrqPayloadCrcErrorMask = 0x20;
static constexpr int kIrqRxDoneMask = 0x40;
static constexpr int kPaOutputRfoPin = 0;
static constexpr int kPaOutputPaBoostPin = 1;
static constexpr int kTimeoutReset = 100;

// SPI Stuff
#if CONFIG_SPI2_HOST
static constexpr spi_host_device_t kHostId = SPI2_HOST;
#elif CONFIG_SPI3_HOST
static constexpr spi_host_device_t kHostId = SPI3_HOST;
#endif

static spi_device_handle_t _spi;
static int _implicit;
static long _frequency;
static int _send_packet_lost = 0;
static int _cr = 0;
static int _sbw = 0;
static int _sf = 0;

// use spi_device_transmit
static constexpr bool kSpiTransmit = true;

// use buffer io
// A little faster
static constexpr bool kBufferIo = true;

/**
 * Write a value to a register.  * @param reg Register index.
 * @param val Value to write.
 */
void lora_write_reg(int reg, int val) {
  uint8_t out[2];
  out[0] = 0x80 | reg;
  out[1] = val;
  uint8_t in[2];

  spi_transaction_t t;
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  // gpio_set_level(CONFIG_CS_GPIO, 0);
  if (kSpiTransmit == true)
    spi_device_transmit(_spi, &t);
  else
    spi_device_polling_transmit(_spi, &t);

  // gpio_set_level(CONFIG_CS_GPIO, 1);
}

/**
 * Write a buffer to a register.
 * @param reg Register index.
 * @param val Value to write.
 * @param len Byte length to write.
 */
void lora_write_reg_buffer(int reg, uint8_t *val, int len) {
  uint8_t *out;
  out = (uint8_t *)malloc(len + 1);
  out[0] = 0x80 | reg;
  for (int i = 0; i < len; i++) {
    out[i + 1] = val[i];
  }

  spi_transaction_t t;
  t.flags = 0;
  t.length = 8 * (len + 1);
  t.tx_buffer = out;
  t.rx_buffer = nullptr;

  // gpio_set_level(CONFIG_CS_GPIO, 0);
  if (kSpiTransmit == true)
    spi_device_transmit(_spi, &t);
  else
    spi_device_polling_transmit(_spi, &t);
  // gpio_set_level(CONFIG_CS_GPIO, 1);
  free(out);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int lora_read_reg(int reg) {
  uint8_t out[2];
  out[0] = reg;
  out[1] = 0xff;
  uint8_t in[2];

  spi_transaction_t t;
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  // gpio_set_level(CONFIG_CS_GPIO, 0);
  if (kSpiTransmit == true)
    spi_device_transmit(_spi, &t);
  else
    spi_device_polling_transmit(_spi, &t);
  // gpio_set_level(CONFIG_CS_GPIO, 1);
  return in[1];
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 * @param len Byte length to read.
 */
void lora_read_reg_buffer(int reg, uint8_t *val, int len) {
  uint8_t *out;
  uint8_t *in;
  out = (uint8_t *)malloc(len + 1);
  in = (uint8_t *)malloc(len + 1);
  out[0] = reg;
  for (int i = 0; i < len; i++) {
    out[i + 1] = 0xff;
  }

  spi_transaction_t t;
  t.flags = 0;
  t.length = 8 * (len + 1);
  t.tx_buffer = out;
  t.rx_buffer = in;

  // gpio_set_level(CONFIG_CS_GPIO, 0);
  if (kSpiTransmit == true)
    spi_device_transmit(_spi, &t);
  else
    spi_device_polling_transmit(_spi, &t);
  // gpio_set_level(CONFIG_CS_GPIO, 1);
  for (int i = 0; i < len; i++) {
    val[i] = in[i + 1];
  }
  free(out);
  free(in);
}

/**
 * Perform physical reset on the Lora chip
 */
void lora_reset(void) {
  gpio_set_level((gpio_num_t)CONFIG_RST_GPIO, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level((gpio_num_t)CONFIG_RST_GPIO, 1);
  vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void lora_explicit_header_mode(void) {
  _implicit = 0;
  lora_write_reg(kRegModemConfig1, lora_read_reg(kRegModemConfig1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void lora_implicit_header_mode(int size) {
  _implicit = 1;
  lora_write_reg(kRegModemConfig1, lora_read_reg(kRegModemConfig1) | 0x01);
  lora_write_reg(kRegPayloadLength, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void lora_idle(void) {
  lora_write_reg(kRegOpMode, kModeLongRangeMode | kModeStdby);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void lora_sleep(void) {
  lora_write_reg(kRegOpMode, kModeLongRangeMode | kModeSleep);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void lora_receive(void) {
  lora_write_reg(kRegOpMode, kModeLongRangeMode | kModeRxContinuous);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void lora_set_tx_power(int level) {
  // RF9x module uses kPaBoost pin
  if (level < 2)
    level = 2;
  else if (level > 17)
    level = 17;
  lora_write_reg(kRegPaConfig, kPaBoost | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void lora_set_frequency(long frequency) {
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  lora_write_reg(kRegFrfMsb, (uint8_t)(frf >> 16));
  lora_write_reg(kRegFrfMid, (uint8_t)(frf >> 8));
  lora_write_reg(kRegFrfLsb, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void lora_set_spreading_factor(int sf) {
  if (sf < 6)
    sf = 6;
  else if (sf > 12)
    sf = 12;

  if (sf == 6) {
    lora_write_reg(kRegDetectionOptimize, 0xc5);
    lora_write_reg(kRegDetectionThreshold, 0x0c);
  } else {
    lora_write_reg(kRegDetectionOptimize, 0xc3);
    lora_write_reg(kRegDetectionThreshold, 0x0a);
  }

  lora_write_reg(
    kRegModemConfig2,
    (lora_read_reg(kRegModemConfig2) & 0x0f) | ((sf << 4) & 0xf0));
  _sf = sf;
}

/**
 * Get spreading factor.
 */
int lora_get_spreading_factor(void) {
  return (lora_read_reg(kRegModemConfig2) >> 4);
}

/**
 * Set Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 * @param mode mode of DIO(0 to 3)
 */
void lora_set_dio_mapping(int dio, int mode) {
  if (dio < 4) {
    int _mode = lora_read_reg(kRegDioMapping1);
    if (dio == 0) {
      _mode = _mode & 0x3F;
      _mode = _mode | (mode << 6);
    } else if (dio == 1) {
      _mode = _mode & 0xCF;
      _mode = _mode | (mode << 4);
    } else if (dio == 2) {
      _mode = _mode & 0xF3;
      _mode = _mode | (mode << 2);
    } else if (dio == 3) {
      _mode = _mode & 0xFC;
      _mode = _mode | mode;
    }
    lora_write_reg(kRegDioMapping1, _mode);
    ESP_LOGD(kLogTag, "kRegDioMapping1=0x%02x", _mode);
  } else if (dio < 6) {
    int _mode = lora_read_reg(kRegDioMapping2);
    if (dio == 4) {
      _mode = _mode & 0x3F;
      _mode = _mode | (mode << 6);
    } else if (dio == 5) {
      _mode = _mode & 0xCF;
      _mode = _mode | (mode << 4);
    }
    ESP_LOGD(kLogTag, "kRegDioMapping2=0x%02x", _mode);
    lora_write_reg(kRegDioMapping2, _mode);
  }
}

/**
 * Get Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 */
int lora_get_dio_mapping(int dio) {
  if (dio < 4) {
    int _mode = lora_read_reg(kRegDioMapping1);
    ESP_LOGD(kLogTag, "kRegDioMapping1=0x%02x", _mode);
    if (dio == 0) {
      return ((_mode >> 6) & 0x03);
    } else if (dio == 1) {
      return ((_mode >> 4) & 0x03);
    } else if (dio == 2) {
      return ((_mode >> 2) & 0x03);
    } else if (dio == 3) {
      return (_mode & 0x03);
    }
  } else if (dio < 6) {
    int _mode = lora_read_reg(kRegDioMapping2);
    ESP_LOGD(kLogTag, "kRegDioMapping2=0x%02x", _mode);
    if (dio == 4) {
      return ((_mode >> 6) & 0x03);
    } else if (dio == 5) {
      return ((_mode >> 4) & 0x03);
    }
  }
  return 0;
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
void lora_set_bandwidth(int sbw) {
  if (sbw < 10) {
    lora_write_reg(
      kRegModemConfig1, (lora_read_reg(kRegModemConfig1) & 0x0f) | (sbw << 4));
    _sbw = sbw;
  }
}

/**
 * Get bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
int lora_get_bandwidth(void) {
  // int bw;
  // bw = lora_read_reg(kRegModemConfig1) & 0xf0;
  // ESP_LOGD(kLogTag, "bw=0x%02x", bw);
  // bw = bw >> 4;
  // return bw;
  return ((lora_read_reg(kRegModemConfig1) & 0xf0) >> 4);
}

/**
 * Set coding rate
 * @param cr Coding Rate(1 to 4)
 */
void lora_set_coding_rate(int cr) {
  // if (denominator < 5) denominator = 5;
  // else if (denominator > 8) denominator = 8;

  // int cr = denominator - 4;
  if (cr < 1)
    cr = 1;
  else if (cr > 4)
    cr = 4;
  lora_write_reg(
    kRegModemConfig1, (lora_read_reg(kRegModemConfig1) & 0xf1) | (cr << 1));
  _cr = cr;
}

/**
 * Get coding rate
 */
int lora_get_coding_rate(void) {
  return ((lora_read_reg(kRegModemConfig1) & 0x0E) >> 1);
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void lora_set_preamble_length(long length) {
  lora_write_reg(kRegPreambleMsb, (uint8_t)(length >> 8));
  lora_write_reg(kRegPreambleLsb, (uint8_t)(length >> 0));
}

/**
 * Get the size of preamble.
 */
long lora_get_preamble_length(void) {
  long preamble;
  preamble = lora_read_reg(kRegPreambleMsb) << 8;
  preamble = preamble + lora_read_reg(kRegPreambleLsb);
  return preamble;
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void lora_set_sync_word(int sw) { lora_write_reg(kRegSyncWord, sw); }

/**
 * Enable appending/verifying packet CRC.
 */
void lora_enable_crc(void) {
  lora_write_reg(kRegModemConfig2, lora_read_reg(kRegModemConfig2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void lora_disable_crc(void) {
  lora_write_reg(kRegModemConfig2, lora_read_reg(kRegModemConfig2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int lora_init(void) {
  esp_err_t ret;

  /*
   * Configure CPU hardware to communicate with the radio chip
   */
  gpio_reset_pin((gpio_num_t)CONFIG_RST_GPIO);
  gpio_set_direction((gpio_num_t)CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
  gpio_reset_pin((gpio_num_t)CONFIG_CS_GPIO);
  gpio_set_direction((gpio_num_t)CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)CONFIG_CS_GPIO, 1);

  spi_bus_config_t bus;
  bus.mosi_io_num = CONFIG_MOSI_GPIO;
  bus.miso_io_num = CONFIG_MISO_GPIO;
  bus.sclk_io_num = CONFIG_SCK_GPIO;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 0;

  // ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
  ret = spi_bus_initialize(kHostId, &bus, SPI_DMA_CH_AUTO);
  assert(ret == ESP_OK);

  spi_device_interface_config_t dev;
  dev.mode = 0;
  dev.clock_speed_hz = 9000000;
  dev.spics_io_num = CONFIG_CS_GPIO;
  dev.flags = 0;
  dev.queue_size = 7;
  dev.pre_cb = nullptr;
  // ret = spi_bus_add_device(VSPI_HOST, &dev, &_spi);
  ret = spi_bus_add_device(kHostId, &dev, &_spi);
  assert(ret == ESP_OK);

  /*
   * Perform hardware reset.
   */
  lora_reset();

  /*
   * Check version.
   */
  uint8_t version;
  uint8_t i = 0;
  while (i++ < kTimeoutReset) {
    version = lora_read_reg(kRegVersion);
    ESP_LOGD(kLogTag, "version=0x%02x", version);
    if (version == 0x12) break;
    vTaskDelay(2);
  }
  ESP_LOGD(kLogTag, "i=%d, kTimeoutReset=%d", i, kTimeoutReset);
  if (i == kTimeoutReset + 1) return 0;  // Illegal version
  // assert(i < kTimeoutReset + 1); // at the end of the loop above, the max
  // value i can reach is kTimeoutReset + 1

  /*
   * Default configuration.
   */
  lora_sleep();
  lora_write_reg(kRegFifoRxBaseAddr, 0);
  lora_write_reg(kRegFifoTxBaseAddr, 0);
  lora_write_reg(kRegLna, lora_read_reg(kRegLna) | 0x03);
  lora_write_reg(kRegModemConfig3, 0x04);
  lora_set_tx_power(17);

  lora_idle();
  return 1;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void lora_send_packet(uint8_t *buf, int size) {
  /*
   * Transfer data to radio.
   */
  lora_idle();
  lora_write_reg(kRegFifoAddrPtr, 0);

  if (kBufferIo == true)
    lora_write_reg_buffer(kRegFifo, buf, size);
  else
    for (int i = 0; i < size; i++) lora_write_reg(kRegFifo, *buf++);

  lora_write_reg(kRegPayloadLength, size);

  /*
   * Start transmission and wait for conclusion.
   */
  lora_write_reg(kRegOpMode, kModeLongRangeMode | kModeTx);
#if 0
   while((lora_read_reg(kRegIrqFlags) & kIrqTxDoneMask) == 0)
      vTaskDelay(2);
#endif
  int loop = 0;
  int max_retry;
  if (_sbw < 2) {
    max_retry = 500;
  } else if (_sbw < 4) {
    max_retry = 250;
  } else if (_sbw < 6) {
    max_retry = 125;
  } else if (_sbw < 8) {
    max_retry = 60;
  } else {
    max_retry = 30;
  }
  ESP_LOGD(kLogTag, "_sbw=%d max_retry=%d", _sbw, max_retry);
  while (1) {
    int irq = lora_read_reg(kRegIrqFlags);
    ESP_LOGD(kLogTag, "lora_read_reg=0x%x", irq);
    if ((irq & kIrqTxDoneMask) == kIrqTxDoneMask) break;
    loop++;
    if (loop == max_retry) break;
    vTaskDelay(2);
  }
  if (loop == max_retry) {
    _send_packet_lost++;
    ESP_LOGE(kLogTag, "lora_send_packet Fail");
  }
  lora_write_reg(kRegIrqFlags, kIrqTxDoneMask);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int lora_receive_packet(uint8_t *buf, int size) {
  int len = 0;

  /*
   * Check interrupts.
   */
  int irq = lora_read_reg(kRegIrqFlags);
  lora_write_reg(kRegIrqFlags, irq);
  if ((irq & kIrqRxDoneMask) == 0) return 0;
  if (irq & kIrqPayloadCrcErrorMask) return 0;

  /*
   * Find packet size.
   */
  if (_implicit)
    len = lora_read_reg(kRegPayloadLength);
  else
    len = lora_read_reg(kRegRxNbBytes);

  /*
   * Transfer data from radio.
   */
  lora_idle();
  lora_write_reg(kRegFifoAddrPtr, lora_read_reg(kRegFifoRxCurrentAddr));
  if (len > size) len = size;
  if (kBufferIo == true)
    lora_read_reg_buffer(kRegFifo, buf, len);
  else
    for (int i = 0; i < len; i++) *buf++ = lora_read_reg(kRegFifo);

  return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int lora_received(void) {
  if (lora_read_reg(kRegIrqFlags) & kIrqRxDoneMask) return 1;
  return 0;
}

/**
 * Returns RegIrqFlags.
 */
int lora_get_irq(void) { return (lora_read_reg(kRegIrqFlags)); }

/**
 * Return lost send packet count.
 */
int lora_packet_lost(void) { return (_send_packet_lost); }

/**
 * Return last packet's RSSI.
 */
int lora_packet_rssi(void) {
  return (lora_read_reg(kRegPktRssiValue) - (_frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float lora_packet_snr(void) {
  return ((int8_t)lora_read_reg(kRegPktSnrValue)) * 0.25;
}

/**
 * Shutdown hardware.
 */
void lora_close(void) {
  lora_sleep();
  //   close(__spi);  FIXME: end hardware features after lora_close
  //   close(__cs);
  //   close(__rst);
  //   _spi = -1;
  //   __cs = -1;
  //   __rst = -1;
}

void lora_dump_registers(void) {
  int i;
  printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
  for (i = 0; i < 0x40; i++) {
    printf("%02X ", lora_read_reg(i));
    if ((i & 0x0f) == 0x0f) printf("\n");
  }
  printf("\n");
}
}  // namespace tef::lora