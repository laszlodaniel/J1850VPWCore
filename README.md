# J1850 VPW Arduino Library

This Arduino library allows the user to receive and send messages on a vehicle diagnostic bus commonly known as PCI-bus.  
It uses interrupts to decode and encode VPW protocol.  

RX-pin must be external interrupt capable:
- Arduino Uno: 2 or 3
- Arduino Mega: 2, 3, 18, 19, 20 or 21 (pins 71 and 72 are not broken out to the connector)

TX-pin must be a 16-bit timer output pin:
- Arduino Uno: 9 or 10
- Arduino Mega: 2, 3, 5, 6, 7, 8, 11, 12, 13, 44, 45 or 46

Writing is not yet supported!

The library is based on [Ryzhkov Ilya's J1850 library](https://github.com/iiryzhkov/j1850_arduino) which serves as a basis for [Stepan Matafonov's J1850 VPW Arduino Transceiver Library](https://github.com/matafonoff/J1850-VPW-Arduino-Transceiver-Library), that is a more advanced library utilizing pin change interrupts to achieve nearly non-blocking operation.

![IMG](https://chryslerccdsci.files.wordpress.com/2021/04/pci-bus_transceiver_schematic_03.png)

To be used with the [CCD/PCI-bus Transceiver Development Boards.](https://github.com/laszlodaniel/CCDPCIBusTransceiver)
