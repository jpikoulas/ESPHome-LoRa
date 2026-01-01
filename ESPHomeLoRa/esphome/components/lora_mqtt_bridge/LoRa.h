#ifndef LORA_WRAPPER_H
#define LORA_WRAPPER_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// Default pin definitions (same as original)
#define LORA_DEFAULT_SPI           SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6
#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
#define LORA_DEFAULT_DIO0_PIN      2
#define LORA_DEFAULT_DIO1_PIN      -1

#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

// Chip type selection
enum LoRaChipType {
  CHIP_SX1276,
  CHIP_SX1277,
  CHIP_SX1278,
  CHIP_SX1279,
  CHIP_SX1262,
  CHIP_SX1268,
  CHIP_SX1280
};

class LoRaClass : public Stream {
public:
  LoRaClass();

  // Set the chip type before calling begin()
  void setChipType(LoRaChipType type);

  int begin(long frequency);
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket(bool async = false);

  int parsePacket(int size = 0);
  int packetRssi();
  float packetSnr();
  long packetFrequencyError();

  int rssi();

  // from Print
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  void onReceive(void(*callback)(int));
  void onCadDone(void(*callback)(boolean));
  void onTxDone(void(*callback)());

  void receive(int size = 0);
  void channelActivityDetection(void);

  void idle();
  void sleep();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();
  void enableInvertIQ();
  void disableInvertIQ();

  void setOCP(uint8_t mA);
  void setGain(uint8_t gain);

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  byte random();

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN,
               int dio0 = LORA_DEFAULT_DIO0_PIN, int dio1 = LORA_DEFAULT_DIO1_PIN);
  void setSPI(SPIClass& spi);
  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);

private:
  void handleDio0Rise();
  void handleDio1Rise();
  bool isTransmitting();

  int getSpreadingFactor();
  long getSignalBandwidth();

  static void onDio0Rise();
  static void onDio1Rise();

  void explicitHeaderMode();
  void implicitHeaderMode();

private:
  SPISettings _spiSettings;
  SPIClass* _spi;
  int _ss;
  int _reset;
  int _dio0;
  int _dio1;
  long _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
  void (*_onCadDone)(boolean);
  void (*_onTxDone)();

  LoRaChipType _chipType;

  // RadioLib module pointers (only one will be used based on chip type)
  SX1276* _sx127x;
  SX1262* _sx1262;

  // Receive buffer for compatibility with Stream interface
  static const int RX_BUFFER_SIZE = 256;
  uint8_t _rxBuffer[RX_BUFFER_SIZE];
  int _rxBufferLen;

  // Transmission buffer
  static const int TX_BUFFER_SIZE = 256;
  uint8_t _txBuffer[TX_BUFFER_SIZE];
  int _txBufferLen;

  // Last packet info
  int _lastRssi;
  float _lastSnr;
  long _lastFreqError;

  // Current settings (RadioLib doesn't provide getters)
  int _currentSpreadingFactor;
  long _currentBandwidth;

  bool _initialized;
};

extern LoRaClass LoRa;

#endif
