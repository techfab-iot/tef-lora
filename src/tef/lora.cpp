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

// GPIO Pins
static gpio_num_t kGpioReset = GPIO_NUM_NC;
static gpio_num_t kGpioCs = GPIO_NUM_NC;
static gpio_num_t kGpioSck = GPIO_NUM_NC;
static gpio_num_t kGpioMiso = GPIO_NUM_NC;
static gpio_num_t kGpioMosi = GPIO_NUM_NC;

static spi_host_device_t kHostId = SPI2_HOST;

static spi_device_handle_t _spi = {0};
static int _implicit;
static long _frequency;
static int _send_packet_lost = 0;
static int _cr = 0;
static int _sbw = 0;
static int _sf = 0;
static int clock_speed = 9000000;

// use spi_device_transmit
static constexpr bool kSpiTransmit = true;

// use buffer io
// A little faster
static constexpr bool kBufferIo = true;

/**
 * Write a value to a register.  * @param reg Register index.
 * @param val Value to write.
 */
void writeReg(int reg, int val) {
  uint8_t out[2];
  out[0] = 0x80 | reg;
  out[1] = val;
  uint8_t in[2];

  spi_transaction_t t = {0};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  // gpio_set_level(kGpioCs, 0);
  if (kSpiTransmit == true)
    spi_device_transmit(_spi, &t);
  else
    spi_device_polling_transmit(_spi, &t);

  // gpio_set_level(kGpioCs, 1);
}

/**
 * Write a buffer to a register.
 * @param reg Register index.
 * @param val Value to write.
 * @param len Byte length to write.
 */
void writeRegBuffer(int reg, uint8_t *val, int len) {
  uint8_t *out;
  out = (uint8_t *)malloc(len + 1);
  out[0] = 0x80 | reg;
  for (int i = 0; i < len; i++) {
    out[i + 1] = val[i];
  }

  spi_transaction_t t = {0};
  t.flags = 0;
  t.length = 8 * (len + 1);
  t.tx_buffer = out;
  t.rx_buffer = nullptr;

  // gpio_set_level(kGpioCs, 0);
  if (kSpiTransmit == true)
    spi_device_transmit(_spi, &t);
  else
    spi_device_polling_transmit(_spi, &t);
  // gpio_set_level(kGpioCs, 1);
  free(out);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int readReg(int reg) {
  uint8_t out[2];
  out[0] = reg;
  out[1] = 0xff;
  uint8_t in[2];

  spi_transaction_t t = {0};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  // gpio_set_level(kGpioCs, 0);
  if (kSpiTransmit == true)
    spi_device_transmit(_spi, &t);
  else
    spi_device_polling_transmit(_spi, &t);
  // gpio_set_level(kGpioCs, 1);
  return in[1];
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 * @param len Byte length to read.
 */
void readRegBuffer(int reg, uint8_t *val, int len) {
  uint8_t *out;
  uint8_t *in;
  out = (uint8_t *)malloc(len + 1);
  in = (uint8_t *)malloc(len + 1);
  out[0] = reg;
  for (int i = 0; i < len; i++) {
    out[i + 1] = 0xff;
  }

  spi_transaction_t t = {0};
  t.flags = 0;
  t.length = 8 * (len + 1);
  t.tx_buffer = out;
  t.rx_buffer = in;

  // gpio_set_level(kGpioCs, 0);
  if (kSpiTransmit == true)
    spi_device_transmit(_spi, &t);
  else
    spi_device_polling_transmit(_spi, &t);
  // gpio_set_level(kGpioCs, 1);
  for (int i = 0; i < len; i++) {
    val[i] = in[i + 1];
  }
  free(out);
  free(in);
}

/**
 * Perform physical reset on the Lora chip
 */
void reset(void) {
  gpio_set_level((gpio_num_t)kGpioReset, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level((gpio_num_t)kGpioReset, 1);
  vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void explicitHeaderMode(void) {
  _implicit = 0;
  writeReg(kRegModemConfig1, readReg(kRegModemConfig1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void implicitHeaderMode(int size) {
  _implicit = 1;
  writeReg(kRegModemConfig1, readReg(kRegModemConfig1) | 0x01);
  writeReg(kRegPayloadLength, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void idle(void) { writeReg(kRegOpMode, kModeLongRangeMode | kModeStdby); }

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void sleep(void) { writeReg(kRegOpMode, kModeLongRangeMode | kModeSleep); }

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void receive(void) {
  writeReg(kRegOpMode, kModeLongRangeMode | kModeRxContinuous);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void setTxPower(int level) {
  // RF9x module uses kPaBoost pin
  if (level < 2)
    level = 2;
  else if (level > 17)
    level = 17;
  writeReg(kRegPaConfig, kPaBoost | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void setFrequency(long frequency) {
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeReg(kRegFrfMsb, (uint8_t)(frf >> 16));
  writeReg(kRegFrfMid, (uint8_t)(frf >> 8));
  writeReg(kRegFrfLsb, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void setSpreadingFactor(int sf) {
  if (sf < 6)
    sf = 6;
  else if (sf > 12)
    sf = 12;

  if (sf == 6) {
    writeReg(kRegDetectionOptimize, 0xc5);
    writeReg(kRegDetectionThreshold, 0x0c);
  } else {
    writeReg(kRegDetectionOptimize, 0xc3);
    writeReg(kRegDetectionThreshold, 0x0a);
  }

  writeReg(
    kRegModemConfig2, (readReg(kRegModemConfig2) & 0x0f) | ((sf << 4) & 0xf0));
  _sf = sf;
}

/**
 * Get spreading factor.
 */
int getSpreadingFactor(void) { return (readReg(kRegModemConfig2) >> 4); }

/**
 * Set Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 * @param mode mode of DIO(0 to 3)
 */
void setDioMapping(int dio, int mode) {
  if (dio < 4) {
    int _mode = readReg(kRegDioMapping1);
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
    writeReg(kRegDioMapping1, _mode);
    ESP_LOGD(kLogTag, "kRegDioMapping1=0x%02x", _mode);
  } else if (dio < 6) {
    int _mode = readReg(kRegDioMapping2);
    if (dio == 4) {
      _mode = _mode & 0x3F;
      _mode = _mode | (mode << 6);
    } else if (dio == 5) {
      _mode = _mode & 0xCF;
      _mode = _mode | (mode << 4);
    }
    ESP_LOGD(kLogTag, "kRegDioMapping2=0x%02x", _mode);
    writeReg(kRegDioMapping2, _mode);
  }
}

/**
 * Get Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 */
int getDioMapping(int dio) {
  if (dio < 4) {
    int _mode = readReg(kRegDioMapping1);
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
    int _mode = readReg(kRegDioMapping2);
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
void setBandwidth(int sbw) {
  if (sbw < 10) {
    writeReg(kRegModemConfig1, (readReg(kRegModemConfig1) & 0x0f) | (sbw << 4));
    _sbw = sbw;
  }
}

/**
 * Get bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
int getBandwidth(void) {
  // int bw;
  // bw = readReg(kRegModemConfig1) & 0xf0;
  // ESP_LOGD(kLogTag, "bw=0x%02x", bw);
  // bw = bw >> 4;
  // return bw;
  return ((readReg(kRegModemConfig1) & 0xf0) >> 4);
}

/**
 * Set coding rate
 * @param cr Coding Rate(1 to 4)
 */
void setCodingRate(int cr) {
  // if (denominator < 5) denominator = 5;
  // else if (denominator > 8) denominator = 8;

  // int cr = denominator - 4;
  if (cr < 1)
    cr = 1;
  else if (cr > 4)
    cr = 4;
  writeReg(kRegModemConfig1, (readReg(kRegModemConfig1) & 0xf1) | (cr << 1));
  _cr = cr;
}

/**
 * Get coding rate
 */
int getCodingRate(void) { return ((readReg(kRegModemConfig1) & 0x0E) >> 1); }

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void setPreambleLength(long length) {
  writeReg(kRegPreambleMsb, (uint8_t)(length >> 8));
  writeReg(kRegPreambleLsb, (uint8_t)(length >> 0));
}

/**
 * Get the size of preamble.
 */
long getPreambleLength(void) {
  long preamble;
  preamble = readReg(kRegPreambleMsb) << 8;
  preamble = preamble + readReg(kRegPreambleLsb);
  return preamble;
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void setSyncWord(int sw) { writeReg(kRegSyncWord, sw); }

/**
 * Enable appending/verifying packet CRC.
 */
void enableCrc(void) {
  writeReg(kRegModemConfig2, readReg(kRegModemConfig2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void disableCrc(void) {
  writeReg(kRegModemConfig2, readReg(kRegModemConfig2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int init(void) {
  esp_err_t ret;

  /*
   * Configure CPU hardware to communicate with the radio chip
   */
  gpio_reset_pin((gpio_num_t)kGpioReset);
  gpio_set_direction((gpio_num_t)kGpioReset, GPIO_MODE_OUTPUT);
  gpio_reset_pin((gpio_num_t)kGpioCs);
  gpio_set_direction((gpio_num_t)kGpioCs, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)kGpioCs, 1);

  spi_bus_config_t bus = {0};
  bus.mosi_io_num = kGpioMosi;
  bus.miso_io_num = kGpioMiso;
  bus.sclk_io_num = kGpioSck;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 0;

  // ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
  ret = spi_bus_initialize(kHostId, &bus, SPI_DMA_CH_AUTO);
  assert(ret == ESP_OK);

  spi_device_interface_config_t dev = {0};
  dev.mode = 0;
  dev.clock_speed_hz = clock_speed;
  dev.spics_io_num = kGpioCs;
  dev.flags = 0;
  dev.queue_size = 7;
  dev.pre_cb = nullptr;
  // ret = spi_bus_add_device(VSPI_HOST, &dev, &_spi);
  ret = spi_bus_add_device(kHostId, &dev, &_spi);
  assert(ret == ESP_OK);

  /*
   * Perform hardware reset.
   */
  reset();

  /*
   * Check version.
   */
  uint8_t version;
  uint8_t i = 0;
  while (i++ < kTimeoutReset) {
    version = readReg(kRegVersion);
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
  sleep();
  writeReg(kRegFifoRxBaseAddr, 0);
  writeReg(kRegFifoTxBaseAddr, 0);
  writeReg(kRegLna, readReg(kRegLna) | 0x03);
  writeReg(kRegModemConfig3, 0x04);
  setTxPower(17);

  idle();
  return 1;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void sendPacket(uint8_t *buf, int size) {
  /*
   * Transfer data to radio.
   */
  idle();
  writeReg(kRegFifoAddrPtr, 0);

  if (kBufferIo == true)
    writeRegBuffer(kRegFifo, buf, size);
  else
    for (int i = 0; i < size; i++) writeReg(kRegFifo, *buf++);

  writeReg(kRegPayloadLength, size);

  /*
   * Start transmission and wait for conclusion.
   */
  writeReg(kRegOpMode, kModeLongRangeMode | kModeTx);
#if 0
   while((readReg(kRegIrqFlags) & kIrqTxDoneMask) == 0)
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
    int irq = readReg(kRegIrqFlags);
    ESP_LOGD(kLogTag, "readReg=0x%x", irq);
    if ((irq & kIrqTxDoneMask) == kIrqTxDoneMask) break;
    loop++;
    if (loop == max_retry) break;
    vTaskDelay(2);
  }
  if (loop == max_retry) {
    _send_packet_lost++;
    ESP_LOGE(kLogTag, "lora_send_packet Fail");
  }
  writeReg(kRegIrqFlags, kIrqTxDoneMask);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int receivePacket(uint8_t *buf, int size) {
  int len = 0;

  /*
   * Check interrupts.
   */
  int irq = readReg(kRegIrqFlags);
  writeReg(kRegIrqFlags, irq);
  if ((irq & kIrqRxDoneMask) == 0) return 0;
  if (irq & kIrqPayloadCrcErrorMask) return 0;

  /*
   * Find packet size.
   */
  if (_implicit)
    len = readReg(kRegPayloadLength);
  else
    len = readReg(kRegRxNbBytes);

  /*
   * Transfer data from radio.
   */
  idle();
  writeReg(kRegFifoAddrPtr, readReg(kRegFifoRxCurrentAddr));
  if (len > size) len = size;
  if (kBufferIo == true)
    readRegBuffer(kRegFifo, buf, len);
  else
    for (int i = 0; i < len; i++) *buf++ = readReg(kRegFifo);

  return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int received(void) {
  if (readReg(kRegIrqFlags) & kIrqRxDoneMask) return 1;
  return 0;
}

/**
 * Returns RegIrqFlags.
 */
int getIrq(void) { return (readReg(kRegIrqFlags)); }

/**
 * Return lost send packet count.
 */
int packetLost(void) { return (_send_packet_lost); }

/**
 * Return last packet's RSSI.
 */
int packetRssi(void) {
  return (readReg(kRegPktRssiValue) - (_frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float packetSnr(void) { return ((int8_t)readReg(kRegPktSnrValue)) * 0.25; }

/**
 * Shutdown hardware.
 */
void close(void) {
  sleep();
  //   close(__spi);  FIXME: end hardware features after close
  //   close(__cs);
  //   close(__rst);
  //   _spi = -1;
  //   __cs = -1;
  //   __rst = -1;
}

void dumpRegisters(void) {
  int i;
  printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
  for (i = 0; i < 0x40; i++) {
    printf("%02X ", readReg(i));
    if ((i & 0x0f) == 0x0f) printf("\n");
  }
  printf("\n");
}

void setPins(
  gpio_num_t rst, gpio_num_t cs, gpio_num_t sck, gpio_num_t miso,
  gpio_num_t mosi) {
  kGpioReset = rst;
  kGpioCs = cs;
  kGpioSck = sck;
  kGpioMiso = miso;
  kGpioMosi = mosi;
}
void setClockSpeed(int speed) { clock_speed = speed; }

void setSpiHost(spi_host_device_t host) { kHostId = host; }

}  // namespace tef::lora