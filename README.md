# J1850 VPW Arduino Library

This Arduino library allows the user to receive and send messages on Chrysler's PCI-bus.
It uses interrupts to decode and encode J1850 VPW protocol.

## Supported Hardware

- **Arduino Uno** (ATmega168/ATmega328P)
- **Arduino Mega** (ATmega1280/ATmega2560)
- **Teensy 4.0** (iMXRT1062)
- **Teensy 4.1** (iMXRT1062)
- **Teensy MicroMod** (iMXRT1062)

## Pin Requirements

**RX-pin must be external interrupt capable:**
- Arduino Uno: pins 2 or 3
- Arduino Mega: pins 2, 3, 18, 19, 20, or 21 (pins 71 and 72 are not broken out to the connector)
- Teensy 4.x: any digital pin (all pins support interrupts)

**TX-pin can be any unused pin.**

Message reception and transmission are handled simultaneously.

## Timer Usage

**Arduino Uno/Mega:**
- Bus-idle condition is measured with Timer 2
- Protocol encoder uses Timer 1 (Uno) or Timer 4 (Mega) for TX pin state changes
- Make sure that these timers are not used by other functions

**Teensy 4.x:**
- Uses 2 IntervalTimer instances (out of 4 available)
  - One for protocol encoder (TX timing)
  - One for bus-idle detection
- Leaves 2 IntervalTimer instances available for user code
- All other Teensy timers (FlexPWM, QuadTimer, GPT) remain free for other uses

Without a suitable transceiver circuit/chip and live PCI-bus network the library is easy to test with the loop-back method: short selected RX/TX pins, upload J1850VPWWrite example, open Arduino Serial Monitor and observe the repeated test message.

![IMG](https://chryslerccdsci.files.wordpress.com/2021/04/pci-bus_transceiver_schematic_03.png)

To be used with the [CCD/PCI-bus Transceiver Development Boards.](https://github.com/laszlodaniel/CCDPCIBusTransceiver)
