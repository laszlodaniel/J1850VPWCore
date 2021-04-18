# J1850 VPW Arduino Library

This Arduino library allows you to receive and send messages on a vehicle diagnostic bus commonly known as PCI-bus.  
It uses blocking methods to read and write messages, therefore other time sensitive operations may be compromised.  
The library is based on [Ryzhkov Ilya's J1850 library](https://github.com/iiryzhkov/j1850_arduino) which serves as a basis for [Stepan Matafonov's J1850 VPW Arduino Transceiver Library](https://github.com/matafonoff/J1850-VPW-Arduino-Transceiver-Library), that is a more advanced library utilizing pin change interrupts to achieve nearly non-blocking operation.

![IMG](https://chryslerccdsci.files.wordpress.com/2021/04/pci-bus_transceiver_schematic_03.png)

To be used with my upcoming [CCD/PCI-bus Transceiver Development Boards.](https://github.com/laszlodaniel/CCDPCIBusTransceiver)

## API

### Include

`#include <J1850VPWCore.h>`

Create an instance of the class named 'VPW'.

`J1850VPWCore VPW;`

### Initializing

`void J1850VPWCore::init(uint8_t rxPin, uint8_t txPin, uint8_t activeLevel = HIGH, bool debug = false);`

- rxPin - pin for incoming signal
- txPin - pin for outgoing signal
- activeLevel - it depends on the transceiver circuit which logic state drives the bus to high voltage (~7V)
- debug - enable / disable debugging mode

MC33390 transceiver IC: ACTIVE_HIGH  
Other custom circuits involving inverted logic: ACTIVE_LOW

Example:

`VPW.init(10, 11, ACTIVE_HIGH, DEBUG_ON);`


#### Debugging mode
When enabled it allows printing incoming messages, outgoing messages, status codes to the Serial Monitor and running tests.  
To change between modes send a single numeric character with new line ending via the Serial Monitor:
- 1 - print incoming and outgoing messages (default)
- 2 - print status codes
- 3 - print incoming messages
- 4 - print outgoing messages
- 5 - run IO test to see if PCI-bus is operational

### Read messages

`bool J1850VPWCore::accept(uint8_t *msg_buf, bool crc_c = true)`

- msg_buf - buffer array for received bytes
- crc_c - verify CRC (default true)

Returns true if valid PCI-bus message is received, otherwise returns false.

Example:

Before receiving messages, a receive buffer must be created. The J1850 VPW protocol allows messages sizes up to **12 bytes**.

`uint8_t rx_buf[12];`

The receiver functions must be called as often as possible to catch all PCI-bus messages.

```
void loop()
{
    if (VPW.accept(rx_buf))
    {
        // Process message here.
        Serial.print("Number of bytes received: ");
        Serial.println(VPW.rx_nbyte);
    }
}
```

The class variable **rx_nbyte** contains the number of bytes in the receive buffer.

### Sending a message

There are two methods to send a message:

`bool J1850VPWCore::send(uint8_t *tx_buf, uint8_t nbytes)`

- tx_buf - transmit buffer array
- nbytes - number of bytes to be sent (max. 11 bytes + 1 CRC byte automatically added)

Returns true if PCI-bus message transmission is successful, otherwise returns false.

Example:

```
uint8_t tx_buf[6] = { 0x48, 0x69, 0x2D, 0x4D, 0x32, 0x21 };

if (VPW.send(tx_buf, sizeof(tx_buf)))
{
    // Do something when transmission is successful.
}
```

and

`bool J1850VPWCore::easy_send(uint8_t size, ...)`

- size - number of bytes to be sent
- series of bytes to be transmitted

Returns true if PCI-bus message transmission is successful, otherwise returns false.

Example:

```
if (VPW.easy_send(6, 0x48, 0x69, 0x2D, 0x4D, 0x32, 0x21))
{
    Serial.print("Number of bytes transmitted: ");
    Serial.println(VPW.tx_nbyte);
}
```

The CRC byte is automatically added to the end of each transmitted message.  
The class variable **tx_nbyte** contains the number of bytes transmitted on the bus.

### Status codes
The **message** class variable contains event codes:

- 1 - TX_OK
- 2 - RX_OK
- 3 - ERROR_MESSAGE_TOO_LONG
- 4 - ERROR_NO_RESPONSE_WITHIN_100US
- 5 - ERROR_SOF_TIMEOUT
- 6 - ERROR_SYMBOL_WAS_NOT_SOF
- 7 - ERROR_SYMBOL_WAS_TOO_SHORT
- 8 - ERROR_CRC_FAILURE

Example, to check why a message transmission may have failed:

```
if (VPW.easy_send(6, 0x48, 0x69, 0x2D, 0x4D, 0x32, 0x21))
{
    Serial.print("Number of bytes transmitted: ");
    Serial.println(VPW.tx_nbyte);
}
else
{
    switch (VPW.message)
    {
        case ERROR_MESSAGE_TOO_LONG:
        {
            Serial.println("Error: transmitted message is too long.");
            break;
        }
        ...
        default:
        {
            Serial.println("Unknown error occured.");
            break;
        }
    }
}
```