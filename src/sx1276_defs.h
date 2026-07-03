#pragma once

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
