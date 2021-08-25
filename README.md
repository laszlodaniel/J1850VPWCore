# J1850 VPW Arduino Library

This Arduino library allows the user to receive and send messages on Chrysler's PCI-bus.  
It uses interrupts to decode and encode J1850 VPW protocol.  
Only Uno and Mega boards are supported.

RX-pin must be external interrupt capable:
- Arduino Uno: 2 or 3
- Arduino Mega: 2, 3, 18, 19, 20 or 21 (pins 71 and 72 are not broken out to the connector)

TX-pin can be any unused pin.

Message reception and transmission is handled at the same time.

Without a live PCI-bus network the library is easy to test with the loop-back method: short selected RX/TX pins, upload J1850VPWWrite example, open Arduino Serial Monitor and observe the repeated test message.

![IMG](https://chryslerccdsci.files.wordpress.com/2021/04/pci-bus_transceiver_schematic_03.png)

To be used with the [CCD/PCI-bus Transceiver Development Boards.](https://github.com/laszlodaniel/CCDPCIBusTransceiver)
