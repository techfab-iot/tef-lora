#pragma once
#include <string>

#include "driver/gpio.h"
#include "hal/spi_types.h"
#include "tef/lora_types.h"
namespace tef::lora::sx1276 {

void reset(void);
void explicitHeaderMode(void);
void implicitHeaderMode(int size);
void idle(void);
void sleep(void);
void receive(void);
int getIrq(void);
void setTxPower(int level);
void setFrequency(long frequency);
void setSpreadingFactor(int sf);
int getSpreadingFactor(void);
void setDioMapping(int dio, int mode);
int getDioMapping(int dio);
void setBandwidth(int sbw);
void setBandwidth(tef::lora::Bandwidth bw);
int getBandwidth(void);
void setCodingRate(int cr);
void setCodingRate(tef::lora::CodingRate cr);
int getCodingRate(void);
void setPreambleLength(long length);
long getPreambleLength(void);
void setSyncWord(int sw);
void enableCrc(void);
void disableCrc(void);
void enableLdro(void);
void disableLdro(void);
int init(void);
int init(
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
uint8_t receive(uint8_t *pData, int16_t len);
bool send(uint8_t *pData, int16_t len, uint8_t mode);
bool send(uint8_t *pData, int16_t len, tef::lora::TxMode mode);
void debugPrint(bool enable);
void getPacketStatus(int8_t *rssiPacket, int8_t *snrPacket);
void sendPacket(uint8_t *buf, int size);
void sendMessage(std::string &message);
int receivePacket(uint8_t *buf, int size);
int received(void);
int packetLost(void);
int packetRssi(void);
float packetSnr(void);
void close(void);
void dumpRegisters(void);
void setClockSpeed(int speed);
void setSpiHost(spi_host_device_t host);

}  // namespace tef::lora::sx1276
