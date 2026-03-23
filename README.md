# V11_Dyson_BMS

Aftermarket firmware for Dyson V11/V15 Battery Management Systems.

Based on https://github.com/davidmpye/V10_Dyson_BMS

By using this project, you acknowledge and agree to the following:

The author and contributors are NOT responsible for any damage, injury,
loss, or legal consequences resulting from the use or misuse of this project.

This project is provided for educational, experimental, and research purposes only.

Improper battery management can result in thermal runaway, fire, toxic fumes,
or explosion.

IF YOU DO NOT FULLY UNDERSTAND THE RISKS OF LITHIUM BATTERIES, DO NOT USE THIS PROJECT.

## Supported Hardware

### MCU

| Parameter | Value |
|-----------|-------|
| Part | Microchip ATSAMD20E15 |
| Core | ARM Cortex-M0+ |
| Flash | 32 KB (1 KB reserved for EEPROM emulation) |
| RAM | 4 KB |

### BMS Frontend

| Parameter | Value |
|-----------|-------|
| Part | Texas Instruments BQ7693003 |
| Interface | I2C (address 0x08) |
| Cells | 7S |
| Features | Cell voltage monitoring, coulomb counter, charge/discharge FET control, OV/UV/OCD/SCD protection |

### Supported Vacuum Models

- Dyson V11 (click-in and screw-type motor head)
- Dyson V15

The firmware implements the Dyson serial protocol with TLV-based communication, supporting both the standard data protocol (SRC=0x01) and V11 screw-type extended protocol (SRC=0x02, SRC=0x03).

## Build Toolchain

### Requirements

- `arm-none-eabi-gcc` (GCC ARM Embedded toolchain)
- `cmake` >= 3.20
- `make`
- `openocd` with J-Link support (for flashing)

### Building

```bash
cd V11_BMS

# Configure and build (Debug)
make all

# Or Release build
make all BUILD_TYPE=Release

# Flash via OpenOCD + J-Link SWD
make flash

# Unlock a firmware-protected Dyson pack (full chip erase)
make unlock

# Clean
make clean
```

Alternatively, open `V11_BMS.atsln` in Microchip/Atmel Studio 7.

### Flashing

Requires a J-Link debug probe connected via SWD. OpenOCD configuration is in `openocd_samd20.cfg`.

## License

GNU GPL v3 or later
