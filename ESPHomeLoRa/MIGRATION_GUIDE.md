# LoRa Component Migration Guide - RadioLib Support

## Overview

The LoRa MQTT components have been updated to use the RadioLib library instead of the arduino-LoRa library. This provides support for newer LoRa chips including the SX1262, SX1268, and SX1280, while maintaining backward compatibility with SX127x series chips.

## Key Changes

### Library Replacement
- **Old**: arduino-LoRa library (https://github.com/sandeepmistry/arduino-LoRa)
- **New**: RadioLib (https://github.com/jgromes/RadioLib)

### Supported Chips
- **SX127x series**: SX1276, SX1277, SX1278, SX1279 (existing support maintained)
- **SX126x series**: SX1262, SX1268 (NEW)
- **SX128x series**: SX1280 (NEW)

## Configuration Changes

### New Configuration Options

1. **chip_type** (optional, default: "SX1276")
   - Specifies which LoRa chip you're using
   - Valid values: `SX1276`, `SX1277`, `SX1278`, `SX1279`, `SX1262`, `SX1268`, `SX1280`

2. **dio1_pin** (optional, default: GPIO33)
   - Required for SX1262/SX1268 chips
   - Used for RxTimeout and other interrupts

### Example Configuration for SX1276 (backward compatible)

```yaml
lora_mqtt:
  cs_pin: GPIO18
  reset_pin: GPIO14
  dio_pin: GPIO26
  frequency: 915000000
  bandwidth: 125E3
  spread: 7
  coding: 5
  sync: 0x12
```

### Example Configuration for SX1262 (NEW)

```yaml
lora_mqtt:
  chip_type: SX1262
  cs_pin: GPIO18
  reset_pin: GPIO14
  dio_pin: GPIO26      # DIO0
  dio1_pin: GPIO33     # DIO1 (required for SX1262)
  frequency: 915000000
  bandwidth: 125E3
  spread: 7
  coding: 5
  sync: 0x12
```

### Example Configuration for LoRa MQTT Bridge with SX1268

```yaml
lora_mqtt_bridge:
  chip_type: SX1268
  cs_pin: GPIO5
  reset_pin: GPIO14
  dio_pin: GPIO26
  dio1_pin: GPIO33
  frequency: 868000000  # EU frequency
  bandwidth: 125E3
  spread: 7
  coding: 5
  sync: 0x12
```

## Migration Steps

### For Existing SX127x Users (No Changes Required)

If you're currently using SX1276/1277/1278/1279, your existing configuration will continue to work without any changes. The default `chip_type` is `SX1276`.

### For New SX1262/SX1268 Users

1. Add the `chip_type` parameter to your configuration
2. Add the `dio1_pin` parameter (required for SX126x chips)
3. Ensure RadioLib is available in your build environment

## Technical Details

### API Compatibility

The wrapper maintains the same API as the original arduino-LoRa library, so existing code using the `LoRa` object will continue to work:

```cpp
LoRa.setPins(cs_pin, reset_pin, dio0_pin, dio1_pin);
LoRa.setChipType(CHIP_SX1262);  // NEW
LoRa.begin(frequency);
LoRa.setSpreadingFactor(7);
LoRa.setSignalBandwidth(125000);
LoRa.setCodingRate4(5);
LoRa.setSyncWord(0x12);

// Sending
LoRa.beginPacket();
LoRa.print("Hello");
LoRa.endPacket();

// Receiving
int packetSize = LoRa.parsePacket();
if (packetSize) {
  while (LoRa.available()) {
    char c = LoRa.read();
  }
}
```

### Pin Mapping

#### SX127x Series
- **CS** (Chip Select): SPI chip select
- **RESET**: Hardware reset
- **DIO0**: RxDone, TxDone, CadDone interrupts
- **DIO1**: Not used (optional)

#### SX1262/SX1268 Series
- **CS** (Chip Select): SPI chip select
- **RESET**: Hardware reset
- **DIO1**: RxDone, TxDone interrupts (replaces DIO0 functionality)
- **BUSY**: Can be connected to DIO0 pin

### Library Dependencies

You'll need to install RadioLib in your ESPHome environment. Add this to your YAML:

```yaml
esphome:
  libraries:
    - "jgromes/RadioLib@^6.6.0"
```

## Known Differences

1. **Register Dump**: The `dumpRegisters()` function is not available with RadioLib (returns a message indicating this)
2. **Gain Control**: `setGain()` is a no-op with RadioLib as it handles gain automatically
3. **Sync Word**: For SX1262/SX1268, the sync word is internally converted to a 2-byte format

## Troubleshooting

### Issue: LoRa initialization fails
- Verify correct chip_type is set
- Check pin connections, especially RESET and CS
- For SX1262/SX1268, ensure DIO1 is connected and configured

### Issue: No packets received
- Ensure both sender and receiver use the same chip settings (frequency, bandwidth, spreading factor, coding rate, sync word)
- For SX1262/SX1268, verify DIO1 interrupt is working

### Issue: Compilation errors
- Ensure RadioLib is installed
- Check that all pin configurations are valid GPIO pins for your board

## Performance Notes

- RadioLib provides better performance and more features than arduino-LoRa
- SX1262/SX1268 chips offer improved sensitivity and lower power consumption compared to SX127x
- The wrapper maintains the same memory footprint as the original implementation

## Additional Resources

- [RadioLib Documentation](https://github.com/jgromes/RadioLib)
- [SX1262 Datasheet](https://www.semtech.com/products/wireless-rf/lora-core/sx1262)
- [ESPHome Custom Components](https://esphome.io/custom/custom_component.html)
