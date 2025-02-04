#include "sx1262.h"

#include <driver/spi_master.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace tef::lora::sx1262 {

static constexpr auto kLogTag = "lora";

// SPI Stuff
static spi_host_device_t kHostId = SPI2_HOST;
static constexpr int SPI_Frequency = 2000000;
static spi_device_handle_t SpiHandle;

// Global Stuff
static uint8_t PacketParams[6];
static bool txActive;
static int txLost = 0;
static bool debug_print;

// GPIO Pins
static gpio_num_t kGpioReset = GPIO_NUM_NC;
static gpio_num_t kGpioCs = GPIO_NUM_NC;
static gpio_num_t kGpioSck = GPIO_NUM_NC;
static gpio_num_t kGpioMiso = GPIO_NUM_NC;
static gpio_num_t kGpioMosi = GPIO_NUM_NC;
static gpio_num_t kGpioBusy = GPIO_NUM_NC;
static gpio_num_t kGpioTxen = GPIO_NUM_NC;
static gpio_num_t kGpioRxen = GPIO_NUM_NC;

void error(int error) {
  if (debug_print) {
    ESP_LOGE(kLogTag, "LoRaErrorDefault=%d", error);
  }
  while (true) {
    vTaskDelay(1);
  }
}

void init(
  gpio_num_t rst, gpio_num_t cs, gpio_num_t sck, gpio_num_t miso,
  gpio_num_t mosi, gpio_num_t busy, gpio_num_t txen, gpio_num_t rxen) {
  kGpioMiso = miso;
  kGpioMosi = mosi;
  kGpioSck = sck;
  kGpioCs = cs;
  kGpioReset = rst;
  kGpioBusy = busy;
  kGpioTxen = txen;
  kGpioRxen = rxen;
  ESP_LOGI(kLogTag, "kGpioMiso=%d", kGpioMiso);
  ESP_LOGI(kLogTag, "kGpioMosi=%d", kGpioMosi);
  ESP_LOGI(kLogTag, "kGpioSck=%d", kGpioSck);
  ESP_LOGI(kLogTag, "kGpioCs=%d", kGpioCs);
  ESP_LOGI(kLogTag, "kGpioReset=%d", kGpioReset);
  ESP_LOGI(kLogTag, "kGpioBusy=%d", kGpioBusy);
  ESP_LOGI(kLogTag, "kGpioTxen=%d", kGpioTxen);
  ESP_LOGI(kLogTag, "kGpioRxen=%d", kGpioRxen);

  txActive = false;
  debug_print = false;

  gpio_reset_pin(kGpioCs);
  gpio_set_direction(kGpioCs, GPIO_MODE_OUTPUT);
  gpio_set_level(kGpioCs, 1);

  gpio_reset_pin(kGpioReset);
  gpio_set_direction(kGpioReset, GPIO_MODE_OUTPUT);

  gpio_reset_pin(kGpioBusy);
  gpio_set_direction(kGpioBusy, GPIO_MODE_INPUT);

  if (kGpioTxen != -1) {
    gpio_reset_pin(kGpioTxen);
    gpio_set_direction(kGpioTxen, GPIO_MODE_OUTPUT);
  }

  if (kGpioRxen != -1) {
    gpio_reset_pin(kGpioRxen);
    gpio_set_direction(kGpioRxen, GPIO_MODE_OUTPUT);
  }

  spi_bus_config_t spi_bus_config = {0};
  spi_bus_config.sclk_io_num = kGpioSck;
  spi_bus_config.mosi_io_num = kGpioMosi;
  spi_bus_config.miso_io_num = kGpioMiso;
  spi_bus_config.quadwp_io_num = -1;
  spi_bus_config.quadhd_io_num = -1;

  esp_err_t ret;
  ret = spi_bus_initialize(kHostId, &spi_bus_config, SPI_DMA_CH_AUTO);
  ESP_LOGI(kLogTag, "spi_bus_initialize=%d", ret);
  assert(ret == ESP_OK);

  spi_device_interface_config_t devcfg;
  memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
  devcfg.clock_speed_hz = SPI_Frequency;
  // It does not work with hardware CS control.
  // devcfg.spics_io_num = kGpioCs;
  // It does work with software CS control.
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 7;
  devcfg.mode = 0;
  devcfg.flags = SPI_DEVICE_NO_DUMMY;

  // spi_device_handle_t handle;
  ret = spi_bus_add_device(kHostId, &devcfg, &SpiHandle);
  ESP_LOGI(kLogTag, "spi_bus_add_device=%d", ret);
  assert(ret == ESP_OK);

#if 0
	pinMode(kGpioCs, OUTPUT);
	pinMode(kGpioReset, OUTPUT);
	pinMode(kGpioBusy, INPUT);
	if (kGpioTxen != -1) pinMode(kGpioTxen, OUTPUT);
	if (kGpioRxen != -1) pinMode(kGpioRxen, OUTPUT);

	SPI.begin();
#endif
}

bool spiWriteByte(uint8_t *Dataout, size_t DataLength) {
  spi_transaction_t SPITransaction;

  if (DataLength > 0) {
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = DataLength * 8;
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = NULL;
    spi_device_transmit(SpiHandle, &SPITransaction);
  }

  return true;
}

bool spiReadByte(uint8_t *Datain, uint8_t *Dataout, size_t DataLength) {
  spi_transaction_t SPITransaction;

  if (DataLength > 0) {
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = DataLength * 8;
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = Datain;
    spi_device_transmit(SpiHandle, &SPITransaction);
  }

  return true;
}

uint8_t spiTransfer(uint8_t address) {
  uint8_t datain[1];
  uint8_t dataout[1];
  dataout[0] = address;
  // spiWriteByte(dataout, 1 );
  spiReadByte(datain, dataout, 1);
  return datain[0];
}

int16_t begin(
  uint32_t frequencyInHz, int8_t txPowerInDbm, float tcxoVoltage,
  bool useRegulatorLDO) {
  if (txPowerInDbm > 22) txPowerInDbm = 22;
  if (txPowerInDbm < -3) txPowerInDbm = -3;

  reset();

  uint8_t wk[2];
  readRegister(SX126X_REG_LORA_SYNC_WORD_MSB, wk, 2);  // 0x0740
  uint16_t syncWord = (wk[0] << 8) + wk[1];
  ESP_LOGI(kLogTag, "syncWord=0x%x", syncWord);
  if (
    syncWord != SX126X_SYNC_WORD_PUBLIC &&
    syncWord != SX126X_SYNC_WORD_PRIVATE) {
    ESP_LOGE(kLogTag, "SX126x error, maybe no SPI connection");
    return ERR_INVALID_MODE;
  }

  ESP_LOGI(kLogTag, "SX126x installed");
  setStandby(SX126X_STANDBY_RC);

  setDio2AsRfSwitchCtrl(true);
  ESP_LOGI(kLogTag, "tcxoVoltage=%f", tcxoVoltage);
  // set TCXO control, if requested
  if (tcxoVoltage > 0.0) {
    setDio3AsTcxoCtrl(
      tcxoVoltage, RADIO_TCXO_SETUP_TIME);  // Configure the radio to use a TCXO
                                            // controlled by DIO3
  }

  calibrate(
    SX126X_CALIBRATE_IMAGE_ON | SX126X_CALIBRATE_ADC_BULK_P_ON |
    SX126X_CALIBRATE_ADC_BULK_N_ON | SX126X_CALIBRATE_ADC_PULSE_ON |
    SX126X_CALIBRATE_PLL_ON | SX126X_CALIBRATE_RC13M_ON |
    SX126X_CALIBRATE_RC64K_ON);

  ESP_LOGI(kLogTag, "useRegulatorLDO=%d", useRegulatorLDO);
  if (useRegulatorLDO) {
    setRegulatorMode(SX126X_REGULATOR_LDO);  // set regulator mode: LDO
  } else {
    setRegulatorMode(SX126X_REGULATOR_DC_DC);  // set regulator mode: DC-DC
  }

  setBufferBaseAddress(0, 0);
#if 0
	// SX1261_TRANCEIVER
	setPaConfig(0x06, 0x00, 0x01, 0x01); // PA Optimal Settings +15 dBm
	// SX1262_TRANCEIVER
	setPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
	// SX1268_TRANCEIVER
	setPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
#endif
  setPaConfig(0x04, 0x07, 0x00, 0x01);  // PA Optimal Settings +22 dBm
  setOvercurrentProtection(60.0);       // current max 60mA for the whole device
  setPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U);  // 0 fuer Empfaenger
  setRfFrequency(frequencyInHz);
  return ERR_NONE;
}

void fixInvertedIQ(uint8_t iqConfig) {
  // fixes IQ configuration for inverted IQ
  // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for
  // details When exchanging LoRa packets with inverted IQ polarity, some packet
  // losses may be observed for longer packets. Workaround: Bit 2 at address
  // 0x0736 must be set to: “0” when using inverted IQ polarity (see the
  // SetPacketParam(...) command) “1” when using standard IQ polarity

  // read current IQ configuration
  uint8_t iqConfigCurrent = 0;
  readRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1);  // 0x0736

  // set correct IQ configuration
  // if(iqConfig == SX126X_LORA_IQ_STANDARD) {
  if (iqConfig == SX126X_LORA_IQ_INVERTED) {
    iqConfigCurrent &= 0xFB;  // using inverted IQ polarity
  } else {
    iqConfigCurrent |= 0x04;  // using standard IQ polarity
  }

  // update with the new value
  writeRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1);  // 0x0736
}

void config(
  uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate,
  uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq) {
  setStopRxTimerOnPreambleDetect(false);
  setLoRaSymbNumTimeout(0);
  setPacketType(SX126X_PACKET_TYPE_LORA);  // SX126x.ModulationParams.PacketType
                                           // : MODEM_LORA
  uint8_t ldro = 0;                        // LowDataRateOptimize OFF
  setModulationParams(spreadingFactor, bandwidth, codingRate, ldro);

  PacketParams[0] = (preambleLength >> 8) & 0xFF;
  PacketParams[1] = preambleLength;
  if (payloadLen) {
    PacketParams[2] = 0x01;  // Fixed length packet (implicit header)
    PacketParams[3] = payloadLen;
  } else {
    PacketParams[2] = 0x00;  // Variable length packet (explicit header)
    PacketParams[3] = 0xFF;
  }

  if (crcOn)
    PacketParams[4] = SX126X_LORA_IQ_INVERTED;
  else
    PacketParams[4] = SX126X_LORA_IQ_STANDARD;

  if (invertIrq)
    PacketParams[5] = 0x01;  // Inverted LoRa I and Q signals setup
  else
    PacketParams[5] = 0x00;  // Standard LoRa I and Q signals setup

  // fixes IQ configuration for inverted IQ
  fixInvertedIQ(PacketParams[5]);

  writeCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6);  // 0x8C

  // Do not use DIO interruptst
  setDioIrqParams(
    SX126X_IRQ_ALL,    // all interrupts enabled
    SX126X_IRQ_NONE,   // interrupts on DIO1
    SX126X_IRQ_NONE,   // interrupts on DIO2
    SX126X_IRQ_NONE);  // interrupts on DIO3

  // Receive state no receive timeoout
  setRx(0xFFFFFF);
}

void debugPrint(bool enable) { debug_print = enable; }

uint8_t receive(uint8_t *pData, int16_t len) {
  uint8_t rxLen = 0;
  uint16_t irqRegs = getIrqStatus();
  // uint8_t status = getStatus();

  if (irqRegs & SX126X_IRQ_RX_DONE) {
    // clearIrqStatus(SX126X_IRQ_RX_DONE);
    clearIrqStatus(SX126X_IRQ_ALL);
    rxLen = readBuffer(pData, len);
  }

  return rxLen;
}

bool send(uint8_t *pData, int16_t len, uint8_t mode) {
  uint16_t irqStatus;
  bool rv = false;

  if (txActive == false) {
    txActive = true;
    PacketParams[2] = 0x00;  // Variable length packet (explicit header)
    PacketParams[3] = len;
    writeCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6);  // 0x8C

    // clearIrqStatus(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);
    clearIrqStatus(SX126X_IRQ_ALL);

    writeBuffer(pData, len);
    setTx(500);

    if (mode & SX126x_TXMODE_SYNC) {
      irqStatus = getIrqStatus();
      while ((!(irqStatus & SX126X_IRQ_TX_DONE)) &&
             (!(irqStatus & SX126X_IRQ_TIMEOUT))) {
        esp_rom_delay_us(1);
        irqStatus = getIrqStatus();
      }
      if (debug_print) {
        ESP_LOGI(kLogTag, "irqStatus=0x%x", irqStatus);
        if (irqStatus & SX126X_IRQ_TX_DONE) {
          ESP_LOGI(kLogTag, "SX126X_IRQ_TX_DONE");
        }
        if (irqStatus & SX126X_IRQ_TIMEOUT) {
          ESP_LOGI(kLogTag, "SX126X_IRQ_TIMEOUT");
        }
      }
      txActive = false;

      setRx(0xFFFFFF);

      if (irqStatus & SX126X_IRQ_TX_DONE) {
        rv = true;
      }
    } else {
      rv = true;
    }
  }
  if (debug_print) {
    ESP_LOGI(kLogTag, "Send rv=0x%x", rv);
  }
  if (rv == false) txLost++;
  return rv;
}

bool receiveMode(void) {
  uint16_t irq;
  bool rv = false;

  if (txActive == false) {
    rv = true;
  } else {
    irq = getIrqStatus();
    if (irq & (SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT)) {
      setRx(0xFFFFFF);
      txActive = false;
      rv = true;
    }
  }

  return rv;
}

void getPacketStatus(int8_t *rssiPacket, int8_t *snrPacket) {
  uint8_t buf[4];
  readCommand(SX126X_CMD_GET_PACKET_STATUS, buf, 4);  // 0x14
  *rssiPacket = (buf[3] >> 1) * -1;
  (buf[2] < 128) ? (*snrPacket = buf[2] >> 2)
                 : (*snrPacket = ((buf[2] - 256) >> 2));
}

void setTxPower(int8_t txPowerInDbm) {
  setPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U);
}

void reset(void) {
  esp_rom_delay_us(10);
  gpio_set_level(kGpioReset, 0);
  esp_rom_delay_us(20);
  gpio_set_level(kGpioReset, 1);
  esp_rom_delay_us(10);
  // ensure BUSY is low (state meachine ready)
  waitForIdle(BUSY_WAIT, const_cast<char *>("reset"), true);
}

void wakeup(void) { getStatus(); }

void setStandby(uint8_t mode) {
  uint8_t data = mode;
  writeCommand(SX126X_CMD_SET_STANDBY, &data, 1);  // 0x80
}

uint8_t getStatus(void) {
  uint8_t rv;
  readCommand(SX126X_CMD_GET_STATUS, &rv, 1);  // 0xC0
  return rv;
}

void setDio3AsTcxoCtrl(float voltage, uint32_t delay) {
  uint8_t buf[4];

  // buf[0] = tcxoVoltage & 0x07;
  if (fabs(voltage - 1.6) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_1_6;
  } else if (fabs(voltage - 1.7) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_1_7;
  } else if (fabs(voltage - 1.8) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_1_8;
  } else if (fabs(voltage - 2.2) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_2_2;
  } else if (fabs(voltage - 2.4) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_2_4;
  } else if (fabs(voltage - 2.7) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_2_7;
  } else if (fabs(voltage - 3.0) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_3_0;
  } else {
    buf[0] = SX126X_DIO3_OUTPUT_3_3;
  }

  uint32_t delayValue = (float)delay / 15.625;
  buf[1] = (uint8_t)((delayValue >> 16) & 0xFF);
  buf[2] = (uint8_t)((delayValue >> 8) & 0xFF);
  buf[3] = (uint8_t)(delayValue & 0xFF);

  writeCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4);  // 0x97
}

void calibrate(uint8_t calibParam) {
  uint8_t data = calibParam;
  writeCommand(SX126X_CMD_CALIBRATE, &data, 1);  // 0x89
}

void setDio2AsRfSwitchCtrl(uint8_t enable) {
  uint8_t data = enable;
  writeCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1);  // 0x9D
}

void setRfFrequency(uint32_t frequency) {
  uint8_t buf[4];
  uint32_t freq = 0;

  calibrateImage(frequency);

  freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
  buf[0] = (uint8_t)((freq >> 24) & 0xFF);
  buf[1] = (uint8_t)((freq >> 16) & 0xFF);
  buf[2] = (uint8_t)((freq >> 8) & 0xFF);
  buf[3] = (uint8_t)(freq & 0xFF);
  writeCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);  // 0x86
}

void calibrateImage(uint32_t frequency) {
  uint8_t calFreq[2];

  if (frequency > 900000000) {
    calFreq[0] = 0xE1;
    calFreq[1] = 0xE9;
  } else if (frequency > 850000000) {
    calFreq[0] = 0xD7;
    calFreq[1] = 0xDB;
  } else if (frequency > 770000000) {
    calFreq[0] = 0xC1;
    calFreq[1] = 0xC5;
  } else if (frequency > 460000000) {
    calFreq[0] = 0x75;
    calFreq[1] = 0x81;
  } else if (frequency > 425000000) {
    calFreq[0] = 0x6B;
    calFreq[1] = 0x6F;
  }
  writeCommand(SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2);  // 0x98
}

void setRegulatorMode(uint8_t mode) {
  uint8_t data = mode;
  writeCommand(SX126X_CMD_SET_REGULATOR_MODE, &data, 1);  // 0x96
}

void setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
  uint8_t buf[2];

  buf[0] = txBaseAddress;
  buf[1] = rxBaseAddress;
  writeCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);  // 0x8F
}

void setPowerConfig(int8_t power, uint8_t rampTime) {
  uint8_t buf[2];

  if (power > 22) {
    power = 22;
  } else if (power < -3) {
    power = -3;
  }

  buf[0] = power;
  buf[1] = (uint8_t)rampTime;
  writeCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2);  // 0x8E
}

void setPaConfig(
  uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut) {
  uint8_t buf[4];

  buf[0] = paDutyCycle;
  buf[1] = hpMax;
  buf[2] = deviceSel;
  buf[3] = paLut;
  writeCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);  // 0x95
}

void setOvercurrentProtection(float currentLimit) {
  if ((currentLimit >= 0.0) && (currentLimit <= 140.0)) {
    uint8_t buf[1];
    buf[0] = (uint8_t)(currentLimit / 2.5);
    writeRegister(SX126X_REG_OCP_CONFIGURATION, buf, 1);  // 0x08E7
  }
}

void setSyncWord(int16_t sync) {
  uint8_t buf[2];

  buf[0] = (uint8_t)((sync >> 8) & 0x00FF);
  buf[1] = (uint8_t)(sync & 0x00FF);
  writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2);  // 0x0740
}

void setDioIrqParams(
  uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
  uint8_t buf[8];

  buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
  buf[1] = (uint8_t)(irqMask & 0x00FF);
  buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
  buf[3] = (uint8_t)(dio1Mask & 0x00FF);
  buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
  buf[5] = (uint8_t)(dio2Mask & 0x00FF);
  buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
  buf[7] = (uint8_t)(dio3Mask & 0x00FF);
  writeCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);  // 0x08
}

void setStopRxTimerOnPreambleDetect(bool enable) {
  ESP_LOGI(kLogTag, "setStopRxTimerOnPreambleDetect enable=%d", enable);
  // uint8_t data = (uint8_t)enable;
  uint8_t data = 0;
  if (enable) data = 1;
  writeCommand(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1);  // 0x9F
}

void setLoRaSymbNumTimeout(uint8_t SymbNum) {
  uint8_t data = SymbNum;
  writeCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1);  // 0xA0
}

void setPacketType(uint8_t packetType) {
  uint8_t data = packetType;
  writeCommand(SX126X_CMD_SET_PACKET_TYPE, &data, 1);  // 0x01
}

void setModulationParams(
  uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate,
  uint8_t lowDataRateOptimize) {
  uint8_t data[4];
  // currently only LoRa supported
  data[0] = spreadingFactor;
  data[1] = bandwidth;
  data[2] = codingRate;
  data[3] = lowDataRateOptimize;
  writeCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 4);  // 0x8B
}

void setCadParams(
  uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin,
  uint8_t cadExitMode, uint32_t cadTimeout) {
  uint8_t data[7];
  data[0] = cadSymbolNum;
  data[1] = cadDetPeak;
  data[2] = cadDetMin;
  data[3] = cadExitMode;
  data[4] = (uint8_t)((cadTimeout >> 16) & 0xFF);
  data[5] = (uint8_t)((cadTimeout >> 8) & 0xFF);
  data[6] = (uint8_t)(cadTimeout & 0xFF);
  writeCommand(SX126X_CMD_SET_CAD_PARAMS, data, 7);  // 0x88
}

void setCad() {
  uint8_t data = 0;
  writeCommand(SX126X_CMD_SET_CAD, &data, 0);  // 0xC5
}

uint16_t getIrqStatus(void) {
  uint8_t data[3];
  readCommand(SX126X_CMD_GET_IRQ_STATUS, data, 3);  // 0x12
  return (data[1] << 8) | data[2];
}

void clearIrqStatus(uint16_t irq) {
  uint8_t buf[2];

  buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
  buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
  writeCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);  // 0x02
}

void setRx(uint32_t timeout) {
  if (debug_print) {
    ESP_LOGI(kLogTag, "----- setRx timeout=%" PRIu32, timeout);
  }
  setStandby(SX126X_STANDBY_RC);
  setRxEnable();
  uint8_t buf[3];
  buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
  buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
  buf[2] = (uint8_t)(timeout & 0xFF);
  writeCommand(SX126X_CMD_SET_RX, buf, 3);  // 0x82

  for (int retry = 0; retry < 10; retry++) {
    if ((getStatus() & 0x70) == 0x50) break;
    esp_rom_delay_us(1);
  }
  if ((getStatus() & 0x70) != 0x50) {
    ESP_LOGE(kLogTag, "setRx Illegal Status");
    error(ERR_INVALID_SETRX_STATE);
  }
}

void setRxEnable(void) {
  if (debug_print) {
    ESP_LOGI(
      kLogTag, "setRxEnable:kGpioTxen=%d kGpioRxen=%d", kGpioTxen, kGpioRxen);
  }
  if ((kGpioTxen != -1) && (kGpioRxen != -1)) {
    gpio_set_level(kGpioRxen, HIGH);
    gpio_set_level(kGpioTxen, LOW);
  }
}

void setTx(uint32_t timeoutInMs) {
  if (debug_print) {
    ESP_LOGI(kLogTag, "----- setTx timeoutInMs=%" PRIu32, timeoutInMs);
  }
  setStandby(SX126X_STANDBY_RC);
  setTxEnable();
  uint8_t buf[3];
  uint32_t tout = timeoutInMs;
  if (timeoutInMs != 0) {
    uint32_t timeoutInUs = timeoutInMs * 1000;
    tout = (uint32_t)(timeoutInUs / 0.015625);
  }
  if (debug_print) {
    ESP_LOGI(
      kLogTag, "setTx timeoutInMs=%" PRIu32 " tout=%" PRIu32, timeoutInMs,
      tout);
  }
  buf[0] = (uint8_t)((tout >> 16) & 0xFF);
  buf[1] = (uint8_t)((tout >> 8) & 0xFF);
  buf[2] = (uint8_t)(tout & 0xFF);
  writeCommand(SX126X_CMD_SET_TX, buf, 3);  // 0x83

  for (int retry = 0; retry < 10; retry++) {
    if ((getStatus() & 0x70) == 0x60) break;
    vTaskDelay(1);
  }
  if ((getStatus() & 0x70) != 0x60) {
    ESP_LOGE(kLogTag, "setTx Illegal Status");
    error(ERR_INVALID_SETTX_STATE);
  }
}

void setTxEnable(void) {
  if (debug_print) {
    ESP_LOGI(
      kLogTag, "setTxEnable:kGpioTxen=%d kGpioRxen=%d", kGpioTxen, kGpioRxen);
  }
  if ((kGpioTxen != -1) && (kGpioRxen != -1)) {
    gpio_set_level(kGpioRxen, LOW);
    gpio_set_level(kGpioTxen, HIGH);
  }
}

int getPacketLost() { return txLost; }

uint8_t getRssiInst() {
  uint8_t buf[2];
  readCommand(SX126X_CMD_GET_RSSI_INST, buf, 2);  // 0x15
  return buf[1];
}

void getRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer) {
  uint8_t buf[3];
  readCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 3);  // 0x13
  *payloadLength = buf[1];
  *rxStartBufferPointer = buf[2];
}

void waitForIdleBegin(unsigned long timeout, char *text) {
  // ensure BUSY is low (state meachine ready)
  bool stop = false;
  for (int retry = 0; retry < 10; retry++) {
    if (retry == 9) stop = true;
    bool ret = waitForIdle(BUSY_WAIT, text, stop);
    if (ret == true) break;
    ESP_LOGW(kLogTag, "waitForIdle fail retry=%d", retry);
    vTaskDelay(1);
  }
}

bool waitForIdle(unsigned long timeout, char *text, bool stop) {
  bool ret = true;
  TickType_t start = xTaskGetTickCount();
  esp_rom_delay_us(1);
  while (xTaskGetTickCount() - start < (timeout / portTICK_PERIOD_MS)) {
    if (gpio_get_level(kGpioBusy) == 0) break;
    esp_rom_delay_us(1);
  }
  if (gpio_get_level(kGpioBusy)) {
    if (stop) {
      ESP_LOGE(
        kLogTag, "waitForIdle Timeout text=%s timeout=%lu start=%" PRIu32, text,
        timeout, start);
      error(ERR_IDLE_TIMEOUT);
    } else {
      ESP_LOGW(
        kLogTag, "waitForIdle Timeout text=%s timeout=%lu start=%" PRIu32, text,
        timeout, start);
      ret = false;
    }
  }
  return ret;
}

uint8_t readBuffer(uint8_t *rxData, int16_t rxDataLen) {
  uint8_t offset = 0;
  uint8_t payloadLength = 0;
  getRxBufferStatus(&payloadLength, &offset);
  if (payloadLength > rxDataLen) {
    ESP_LOGW(
      kLogTag, "readBuffer rxDataLen too small. payloadLength=%d rxDataLen=%d",
      payloadLength, rxDataLen);
    return 0;
  }

  // ensure BUSY is low (state meachine ready)
  waitForIdle(BUSY_WAIT, const_cast<char *>("start readBuffer"), true);

  // start transfer
  gpio_set_level(kGpioCs, LOW);

  spiTransfer(SX126X_CMD_READ_BUFFER);  // 0x1E
  spiTransfer(offset);
  spiTransfer(SX126X_CMD_NOP);
  for (int i = 0; i < payloadLength; i++) {
    rxData[i] = spiTransfer(SX126X_CMD_NOP);
  }

  // stop transfer
  gpio_set_level(kGpioCs, HIGH);

  // wait for BUSY to go low
  waitForIdle(BUSY_WAIT, const_cast<char *>("end readBuffer"), false);

  return payloadLength;
}

void writeBuffer(uint8_t *txData, int16_t txDataLen) {
  // ensure BUSY is low (state meachine ready)
  waitForIdle(BUSY_WAIT, const_cast<char *>("start writeBuffer"), true);

  // start transfer
  gpio_set_level(kGpioCs, LOW);

  spiTransfer(SX126X_CMD_WRITE_BUFFER);  // 0x0E
  spiTransfer(0);                        // offset in tx fifo
  for (int i = 0; i < txDataLen; i++) {
    spiTransfer(txData[i]);
  }

  // stop transfer
  gpio_set_level(kGpioCs, HIGH);

  // wait for BUSY to go low
  waitForIdle(BUSY_WAIT, const_cast<char *>("end writeBuffer"), false);
}

void writeRegister(uint16_t reg, uint8_t *data, uint8_t numBytes) {
  // ensure BUSY is low (state meachine ready)
  waitForIdle(BUSY_WAIT, const_cast<char *>("start writeRegister"), true);

  if (debug_print) {
    ESP_LOGI(kLogTag, "writeRegister: REG=0x%02x", reg);
  }
  // start transfer
  gpio_set_level(kGpioCs, LOW);

  // send command byte
  spiTransfer(SX126X_CMD_WRITE_REGISTER);  // 0x0D
  spiTransfer((reg & 0xFF00) >> 8);
  spiTransfer(reg & 0xff);

  for (uint8_t n = 0; n < numBytes; n++) {
    uint8_t in = spiTransfer(data[n]);
    (void)in;
    if (debug_print) {
      ESP_LOGI(kLogTag, "%02x --> %02x", data[n], in);
      // ESP_LOGI(kLogTag, "DataOut:%02x ", data[n]);
    }
  }

  // stop transfer
  gpio_set_level(kGpioCs, HIGH);

  // wait for BUSY to go low
  waitForIdle(BUSY_WAIT, const_cast<char *>("end writeRegister"), false);
#if 0
	if(waitForBusy) {
		waitForIdle(BUSY_WAIT);
	}
#endif
}

void readRegister(uint16_t reg, uint8_t *data, uint8_t numBytes) {
  // ensure BUSY is low (state meachine ready)
  waitForIdle(BUSY_WAIT, const_cast<char *>("start readRegister"), true);

  if (debug_print) {
    ESP_LOGI(kLogTag, "readRegister: REG=0x%02x", reg);
  }

  // start transfer
  gpio_set_level(kGpioCs, LOW);

  // send command byte
  spiTransfer(SX126X_CMD_READ_REGISTER);  // 0x1D
  spiTransfer((reg & 0xFF00) >> 8);
  spiTransfer(reg & 0xff);
  spiTransfer(SX126X_CMD_NOP);

  for (uint8_t n = 0; n < numBytes; n++) {
    data[n] = spiTransfer(SX126X_CMD_NOP);
    if (debug_print) {
      ESP_LOGI(kLogTag, "DataIn:%02x ", data[n]);
    }
  }

  // stop transfer
  gpio_set_level(kGpioCs, HIGH);

  // wait for BUSY to go low
  waitForIdle(BUSY_WAIT, const_cast<char *>("end readRegister"), false);
#if 0
	if(waitForBusy) {
		waitForIdle(BUSY_WAIT);
	}
#endif
}

// writeCommand with retry
void writeCommand(uint8_t cmd, uint8_t *data, uint8_t numBytes) {
  uint8_t status;
  for (int retry = 1; retry < 10; retry++) {
    status = writeCommand2(cmd, data, numBytes);
    ESP_LOGD(kLogTag, "status=%02x", status);
    if (status == 0) break;
    ESP_LOGW(kLogTag, "writeCommand2 status=%02x retry=%d", status, retry);
  }
  if (status != 0) {
    ESP_LOGE(kLogTag, "SPI Transaction error:0x%02x", status);
    error(ERR_SPI_TRANSACTION);
  }
}

uint8_t writeCommand2(uint8_t cmd, uint8_t *data, uint8_t numBytes) {
  // ensure BUSY is low (state meachine ready)
  waitForIdle(BUSY_WAIT, const_cast<char *>("start writeCommand2"), true);

  // start transfer
  gpio_set_level(kGpioCs, LOW);

  // send command byte
  if (debug_print) {
    ESP_LOGI(kLogTag, "writeCommand: CMD=0x%02x", cmd);
  }
  spiTransfer(cmd);

  // variable to save error during SPI transfer
  uint8_t status = 0;

  // send/receive all bytes
  for (uint8_t n = 0; n < numBytes; n++) {
    uint8_t in = spiTransfer(data[n]);
    if (debug_print) {
      ESP_LOGI(kLogTag, "%02x --> %02x", data[n], in);
    }

    // check status
    if (
      ((in & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT) ||
      ((in & 0b00001110) == SX126X_STATUS_CMD_INVALID) ||
      ((in & 0b00001110) == SX126X_STATUS_CMD_FAILED)) {
      status = in & 0b00001110;
      break;
    } else if (in == 0x00 || in == 0xFF) {
      status = SX126X_STATUS_SPI_FAILED;
      break;
    }
  }

  // stop transfer
  gpio_set_level(kGpioCs, HIGH);

  // wait for BUSY to go low
  waitForIdle(BUSY_WAIT, const_cast<char *>("end writeCommand2"), false);
#if 0
	if(waitForBusy) {
		waitForIdle(BUSY_WAIT);
	}
#endif

#if 0
	if (status != 0) {
		ESP_LOGE(kLogTag, "SPI Transaction error:0x%02x", status);
		error(ERR_SPI_TRANSACTION);
	}
#endif
  return status;
}

void readCommand(uint8_t cmd, uint8_t *data, uint8_t numBytes) {
  // ensure BUSY is low (state meachine ready)
  // waitForIdle(BUSY_WAIT, "start readCommand", true);
  waitForIdleBegin(BUSY_WAIT, const_cast<char *>("start readCommand"));

  // start transfer
  gpio_set_level(kGpioCs, LOW);

  // send command byte
  if (debug_print) {
    ESP_LOGI(kLogTag, "readCommand: CMD=0x%02x", cmd);
  }
  spiTransfer(cmd);

  // send/receive all bytes
  for (uint8_t n = 0; n < numBytes; n++) {
    data[n] = spiTransfer(SX126X_CMD_NOP);
    if (debug_print) {
      ESP_LOGI(kLogTag, "DataIn:%02x", data[n]);
    }
  }

  // stop transfer
  gpio_set_level(kGpioCs, HIGH);

  // wait for BUSY to go low
  vTaskDelay(1);
  waitForIdle(BUSY_WAIT, const_cast<char *>("end readCommand"), false);
#if 0
	if(waitForBusy) {
		waitForIdle(BUSY_WAIT);
	}
#endif
}
}  // namespace tef::lora::sx1262