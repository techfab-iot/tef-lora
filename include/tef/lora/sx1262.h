#pragma once

#include <driver/gpio.h>

#include "driver/spi_master.h"
#include "tef/lora_types.h"

namespace tef::lora::sx1262 {

// Public function
void init(
  gpio_num_t rst, gpio_num_t cs, gpio_num_t sck, gpio_num_t miso,
  gpio_num_t mosi, gpio_num_t busy, gpio_num_t dio1, gpio_num_t txen,
  gpio_num_t rxen);
int16_t begin(
  uint32_t frequencyInHz, int8_t txPowerInDbm, float tcxoVoltage,
  bool useRegulatorLDO);
void config(
  uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate,
  uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq);
void config(
  uint8_t spreadingFactor, tef::lora::Bandwidth bandwidth,
  tef::lora::CodingRate codingRate, uint16_t preambleLength,
  uint8_t payloadLen, bool crcOn, bool invertIrq);
uint8_t receive(uint8_t* pData, int16_t len);
bool send(uint8_t* pData, int16_t len, uint8_t mode);
bool send(uint8_t* pData, int16_t len, tef::lora::TxMode mode);
void debugPrint(bool enable);

// Private function
bool spiWriteByte(uint8_t* Dataout, size_t DataLength);
bool spiReadByte(uint8_t* Datain, uint8_t* Dataout, size_t DataLength);
uint8_t spiTransfer(uint8_t address);

bool receiveMode(void);
void getPacketStatus(int8_t* rssiPacket, int8_t* snrPacket);
void setTxPower(int8_t txPowerInDbm);

void fixInvertedIQ(uint8_t iqConfig);
void setDio3AsTcxoCtrl(float voltage, uint32_t delay);
void setDio2AsRfSwitchCtrl(uint8_t enable);
void reset(void);
void setStandby(uint8_t mode);
void setRfFrequency(uint32_t frequency);
void calibrate(uint8_t calibParam);
void calibrateImage(uint32_t frequency);
void setRegulatorMode(uint8_t mode);
void setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
void setPowerConfig(int8_t power, uint8_t rampTime);
void setOvercurrentProtection(float currentLimit);
void setSyncWord(int16_t sync);
void setPaConfig(
  uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);
void setDioIrqParams(
  uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
void setStopRxTimerOnPreambleDetect(bool enable);
void setLoRaSymbNumTimeout(uint8_t SymbNum);
void setPacketType(uint8_t packetType);
void setModulationParams(
  uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate,
  uint8_t lowDataRateOptimize);
void setCadParams(
  uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin,
  uint8_t cadExitMode, uint32_t cadTimeout);
void setCad();
uint8_t getStatus(void);
uint16_t getIrqStatus(void);
void clearIrqStatus(uint16_t irq);
uint16_t getDeviceErrors(void);
void clearDeviceErrors(void);
void setTxEnable(void);
void setRxEnable(void);
void setRx(uint32_t timeout);
void setTx(uint32_t timeoutInMs);
int getPacketLost();
uint8_t getRssiInst();
void getRxBufferStatus(uint8_t* payloadLength, uint8_t* rxStartBufferPointer);
void wakeup(void);
void waitForIdleBegin(unsigned long timeout, char* text);
bool waitForIdle(unsigned long timeout, char* text, bool stop);
uint8_t readBuffer(uint8_t* rxData, int16_t rxDataLen);
void writeBuffer(uint8_t* txData, int16_t txDataLen);
void writeRegister(uint16_t reg, uint8_t* data, uint8_t numBytes);
void readRegister(uint16_t reg, uint8_t* data, uint8_t numBytes);
void writeCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes);
uint8_t writeCommand2(uint8_t cmd, uint8_t* data, uint8_t numBytes);
void readCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes);
void spiTransfer(
  uint8_t cmd, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes,
  bool waitForBusy);
void error(int error);
}
