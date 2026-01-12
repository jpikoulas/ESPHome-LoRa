#include "LoRa.h"

#if (ESP8266 || ESP32)
    #define ISR_PREFIX ICACHE_RAM_ATTR
#else
    #define ISR_PREFIX
#endif

LoRaClass::LoRaClass() :
  _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
  _spi(&LORA_DEFAULT_SPI),
  _ss(LORA_DEFAULT_SS_PIN),
  _reset(LORA_DEFAULT_RESET_PIN),
  _dio0(LORA_DEFAULT_DIO0_PIN),
  _dio1(LORA_DEFAULT_DIO1_PIN),
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL),
  _onCadDone(NULL),
  _onTxDone(NULL),
  _chipType(CHIP_SX1276),
  _sx127x(NULL),
  _sx1262(NULL),
  _rxBufferLen(0),
  _txBufferLen(0),
  _lastRssi(0),
  _lastSnr(0),
  _lastFreqError(0),
  _currentSpreadingFactor(7),
  _currentBandwidth(125000),
  _initialized(false)
{
  setTimeout(0);
}

void LoRaClass::setChipType(LoRaChipType type) {
  _chipType = type;
}

int LoRaClass::begin(long frequency) {
  // Start SPI
  _spi->begin();

  // Create the appropriate radio module based on chip type
  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    Module* mod = new Module(_ss, _dio0, _reset, _dio1, *_spi, _spiSettings);
    _sx1262 = new SX1262(mod);

    // Initialize SX1262
    int state = _sx1262->begin(frequency / 1000000.0);
    if (state != RADIOLIB_ERR_NONE) {
      delete _sx1262;
      delete mod;
      _sx1262 = NULL;
      return 0;
    }

    // Configure TCXO for Wio-SX1262 module (1.8V, 5ms startup delay)
    // This is required for the Seeedstudio Wio-SX1262 module which uses an external TCXO
    state = _sx1262->setTCXO(1.8);
    if (state != RADIOLIB_ERR_NONE) {
      // TCXO setup failed, but continue anyway - some modules may not need it
    }

    // Enable DIO2 as RF switch control (for antenna switching)
    state = _sx1262->setDio2AsRfSwitch(true);
    if (state != RADIOLIB_ERR_NONE) {
      // RF switch setup failed, but continue anyway
    }

    // Set default parameters
    _sx1262->setSpreadingFactor(7);
    _sx1262->setBandwidth(125.0);
    _sx1262->setCodingRate(5);
    _sx1262->setOutputPower(17);
    _sx1262->setPreambleLength(8);

  } else {
    // SX127x series (SX1276, SX1277, SX1278, SX1279)
    Module* mod = new Module(_ss, _dio0, _reset, _dio1, *_spi, _spiSettings);
    _sx127x = new SX1276(mod);

    // Initialize SX127x
    int state = _sx127x->begin(frequency / 1000000.0);
    if (state != RADIOLIB_ERR_NONE) {
      delete _sx127x;
      delete mod;
      _sx127x = NULL;
      return 0;
    }

    // Set default parameters
    _sx127x->setSpreadingFactor(7);
    _sx127x->setBandwidth(125.0);
    _sx127x->setCodingRate(5);
    _sx127x->setOutputPower(17);
    _sx127x->setPreambleLength(8);
  }

  _frequency = frequency;
  _initialized = true;
  return 1;
}

void LoRaClass::end() {
  if (_sx127x) {
    delete _sx127x;
    _sx127x = NULL;
  }
  if (_sx1262) {
    delete _sx1262;
    _sx1262 = NULL;
  }
  _spi->end();
  _initialized = false;
}

int LoRaClass::beginPacket(int implicitHeader) {
  if (!_initialized) return 0;

  _txBufferLen = 0;
  _implicitHeaderMode = implicitHeader;

  return 1;
}

int LoRaClass::endPacket(bool async) {
  if (!_initialized) return 0;

  int state;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    if (async) {
      state = _sx1262->startTransmit(_txBuffer, _txBufferLen);
    } else {
      state = _sx1262->transmit(_txBuffer, _txBufferLen);
    }
  } else {
    if (async) {
      state = _sx127x->startTransmit(_txBuffer, _txBufferLen);
    } else {
      state = _sx127x->transmit(_txBuffer, _txBufferLen);
    }
  }

  _txBufferLen = 0;
  return (state == RADIOLIB_ERR_NONE) ? 1 : 0;
}

bool LoRaClass::isTransmitting() {
  if (!_initialized) return false;

  // RadioLib doesn't have a direct isTransmitting check
  // We'll return false as transmissions are handled by endPacket
  return false;
}

int LoRaClass::parsePacket(int size) {
  if (!_initialized) return 0;

  _rxBufferLen = 0;
  _packetIndex = 0;

  int state;
  size_t packetLen;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    // For RadioLib, getPacketLength() must be called BEFORE readData()
    packetLen = _sx1262->getPacketLength();
    if (packetLen == 0 || packetLen > RX_BUFFER_SIZE) {
      return 0;
    }
    state = _sx1262->readData(_rxBuffer, packetLen);
    if (state == RADIOLIB_ERR_NONE) {
      _rxBufferLen = packetLen;
      _lastRssi = _sx1262->getRSSI();
      _lastSnr = _sx1262->getSNR();
      _lastFreqError = _sx1262->getFrequencyError();
    }
  } else {
    // For RadioLib, getPacketLength() must be called BEFORE readData()
    packetLen = _sx127x->getPacketLength();
    if (packetLen == 0 || packetLen > RX_BUFFER_SIZE) {
      return 0;
    }
    state = _sx127x->readData(_rxBuffer, packetLen);
    if (state == RADIOLIB_ERR_NONE) {
      _rxBufferLen = packetLen;
      _lastRssi = _sx127x->getRSSI();
      _lastSnr = _sx127x->getSNR();
      _lastFreqError = _sx127x->getFrequencyError();
    }
  }

  return _rxBufferLen;
}

int LoRaClass::packetRssi() {
  return _lastRssi;
}

float LoRaClass::packetSnr() {
  return _lastSnr;
}

long LoRaClass::packetFrequencyError() {
  return _lastFreqError;
}

int LoRaClass::rssi() {
  if (!_initialized) return 0;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    return _sx1262->getRSSI();
  } else {
    return _sx127x->getRSSI();
  }
}

size_t LoRaClass::write(uint8_t byte) {
  return write(&byte, sizeof(byte));
}

size_t LoRaClass::write(const uint8_t *buffer, size_t size) {
  if (_txBufferLen + size > TX_BUFFER_SIZE) {
    size = TX_BUFFER_SIZE - _txBufferLen;
  }

  memcpy(_txBuffer + _txBufferLen, buffer, size);
  _txBufferLen += size;

  return size;
}

int LoRaClass::available() {
  return _rxBufferLen - _packetIndex;
}

int LoRaClass::read() {
  if (!available()) {
    return -1;
  }

  return _rxBuffer[_packetIndex++];
}

int LoRaClass::peek() {
  if (!available()) {
    return -1;
  }

  return _rxBuffer[_packetIndex];
}

void LoRaClass::flush() {
  // Nothing to do for LoRa
}

void LoRaClass::onReceive(void(*callback)(int)) {
  _onReceive = callback;

  if (callback) {
    // Use setPacketReceivedAction - RadioLib's high-level API that handles all IRQ setup
    if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
      _sx1262->setPacketReceivedAction(LoRaClass::onDio0Rise);
    } else {
      _sx127x->setPacketReceivedAction(LoRaClass::onDio0Rise);
    }
  } else {
    if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
      _sx1262->clearPacketReceivedAction();
    } else {
      _sx127x->clearPacketReceivedAction();
    }
  }
}

void LoRaClass::onCadDone(void(*callback)(boolean)) {
  _onCadDone = callback;
  // CAD callbacks are handled through the same DIO interrupt as onReceive
}

void LoRaClass::onTxDone(void(*callback)()) {
  _onTxDone = callback;
  // TX done callbacks are handled through the same DIO interrupt as onReceive
}

void LoRaClass::receive(int size) {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    // For SX1262, we need to specify IRQ flags for interrupt-based receive
    _sx1262->startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_SX126X_IRQ_RX_DONE, RADIOLIB_SX126X_IRQ_RX_DONE);
  } else {
    _sx127x->startReceive();
  }
}

void LoRaClass::channelActivityDetection(void) {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->startChannelScan();
  } else {
    _sx127x->startChannelScan();
  }
}

void LoRaClass::idle() {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->standby();
  } else {
    _sx127x->standby();
  }
}

void LoRaClass::sleep() {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->sleep();
  } else {
    _sx127x->sleep();
  }
}

void LoRaClass::setTxPower(int level, int outputPin) {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->setOutputPower(level);
  } else {
    _sx127x->setOutputPower(level);
  }
}

void LoRaClass::setFrequency(long frequency) {
  if (!_initialized) return;

  _frequency = frequency;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->setFrequency(frequency / 1000000.0);
  } else {
    _sx127x->setFrequency(frequency / 1000000.0);
  }
}

void LoRaClass::setSpreadingFactor(int sf) {
  if (!_initialized) return;

  if (sf < 6) sf = 6;
  if (sf > 12) sf = 12;

  _currentSpreadingFactor = sf;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->setSpreadingFactor(sf);
  } else {
    _sx127x->setSpreadingFactor(sf);
  }
}

int LoRaClass::getSpreadingFactor() {
  return _currentSpreadingFactor;
}

void LoRaClass::setSignalBandwidth(long sbw) {
  if (!_initialized) return;

  _currentBandwidth = sbw;
  float bw_khz = sbw / 1000.0;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->setBandwidth(bw_khz);
  } else {
    _sx127x->setBandwidth(bw_khz);
  }
}

long LoRaClass::getSignalBandwidth() {
  return _currentBandwidth;
}

void LoRaClass::setCodingRate4(int denominator) {
  if (!_initialized) return;

  if (denominator < 5) denominator = 5;
  if (denominator > 8) denominator = 8;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->setCodingRate(denominator);
  } else {
    _sx127x->setCodingRate(denominator);
  }
}

void LoRaClass::setPreambleLength(long length) {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->setPreambleLength(length);
  } else {
    _sx127x->setPreambleLength(length);
  }
}

void LoRaClass::setSyncWord(int sw) {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    // SX1262 uses a 2-byte sync word; we'll use the same byte twice
    uint8_t syncWord[2] = {(uint8_t)sw, (uint8_t)sw};
    _sx1262->setSyncWord(syncWord, 2);
  } else {
    _sx127x->setSyncWord(sw);
  }
}

void LoRaClass::enableCrc() {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->setCRC(true);
  } else {
    _sx127x->setCRC(true);
  }
}

void LoRaClass::disableCrc() {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->setCRC(false);
  } else {
    _sx127x->setCRC(false);
  }
}

void LoRaClass::enableInvertIQ() {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->invertIQ(true);
  } else {
    _sx127x->invertIQ(true);
  }
}

void LoRaClass::disableInvertIQ() {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->invertIQ(false);
  } else {
    _sx127x->invertIQ(false);
  }
}

void LoRaClass::setOCP(uint8_t mA) {
  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->setCurrentLimit(mA);
  } else {
    _sx127x->setCurrentLimit(mA);
  }
}

void LoRaClass::setGain(uint8_t gain) {
  // RadioLib handles gain automatically, this is a no-op for compatibility
}

byte LoRaClass::random() {
  if (!_initialized) return 0;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    return _sx1262->randomByte();
  } else {
    return _sx127x->randomByte();
  }
}

void LoRaClass::setPins(int ss, int reset, int dio0, int dio1) {
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
  _dio1 = dio1;
}

void LoRaClass::setSPI(SPIClass& spi) {
  _spi = &spi;
}

void LoRaClass::setSPIFrequency(uint32_t frequency) {
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void LoRaClass::dumpRegisters(Stream& out) {
  // RadioLib doesn't provide direct register access in the same way
  out.println("Register dump not available with RadioLib");
}

void LoRaClass::explicitHeaderMode() {
  _implicitHeaderMode = 0;

  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->explicitHeader();
  } else {
    _sx127x->explicitHeader();
  }
}

void LoRaClass::implicitHeaderMode() {
  _implicitHeaderMode = 1;

  if (!_initialized) return;

  if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
    _sx1262->implicitHeader(255);
  } else {
    _sx127x->implicitHeader(255);
  }
}

void LoRaClass::handleDio0Rise() {
  if (!_initialized) return;

  int packetLength = parsePacket();

  if (packetLength > 0 && _onReceive) {
    _onReceive(packetLength);
  }

  if (_onTxDone) {
    _onTxDone();
  }

  // Restart receive mode for continuous reception
  if (_onReceive) {
    if (_chipType == CHIP_SX1262 || _chipType == CHIP_SX1268) {
      _sx1262->startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_SX126X_IRQ_RX_DONE, RADIOLIB_SX126X_IRQ_RX_DONE);
    } else {
      _sx127x->startReceive();
    }
  }
}

void LoRaClass::handleDio1Rise() {
  // DIO1 is used for RxTimeout and other events
}

ISR_PREFIX void LoRaClass::onDio0Rise() {
  LoRa.handleDio0Rise();
}

ISR_PREFIX void LoRaClass::onDio1Rise() {
  LoRa.handleDio1Rise();
}

LoRaClass LoRa;
