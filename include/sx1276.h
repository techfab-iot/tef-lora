#pragma once
#include <string>

#include "driver/gpio.h"
#include "hal/spi_types.h"
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
int getBandwidth(void);
void setCodingRate(int cr);
int getCodingRate(void);
void setPreambleLength(long length);
long getPreambleLength(void);
void setSyncWord(int sw);
void enableCrc(void);
void disableCrc(void);
int init(void);
void sendPacket(uint8_t *buf, int size);
void sendMessage(std::string &message);
int receivePacket(uint8_t *buf, int size);
int received(void);
int packetLost(void);
int packetRssi(void);
float packetSnr(void);
void close(void);
void dumpRegisters(void);
void setPins(
  gpio_num_t rst, gpio_num_t cs, gpio_num_t sck, gpio_num_t miso,
  gpio_num_t mosi);
void setClockSpeed(int speed);
void setSpiHost(spi_host_device_t host);

}  // namespace tef::lora::sx1276
