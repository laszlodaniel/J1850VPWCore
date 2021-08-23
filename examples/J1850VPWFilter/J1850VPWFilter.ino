/*
 * J1850VPWFilter.ino (https://github.com/laszlodaniel/J1850VPWCore)
 * Copyright (C) 2021, Daniel Laszlo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <J1850VPWCore.h>

#define J1850VPW_RX 2 // INT4 on Mega
#define J1850VPW_TX 6 // OC4A on Mega
#define SLEEP       5
#define _4XLOOP     7

uint8_t messageFilter[] = { 0x10, 0x14, 0xC0 }; // ID-bytes to filter

void PCIMessageReceived(uint8_t* message, uint8_t messageLength)
{
    for (uint8_t i = 0; i < messageLength; i++)
    {
        if (message[i] < 16) Serial.print("0"); // print leading zero
        Serial.print(message[i], HEX); // print message byte in hexadecimal format on the serial monitor
        Serial.print(" "); // insert whitespace between bytes
    }

    Serial.println(); // add new line
}

void PCIHandleError(J1850VPW_Operations op, J1850VPW_Errors err)
{
    if (err == J1850VPW_OK) return;

    String s = op == J1850VPW_Read ? "READ " : "WRITE ";

    switch (err)
    {
        case J1850VPW_ERR_TX_DISABLED:
        {
            Serial.println(s + "J1850VPW_ERR_TX_DISABLED");
            break;
        }
        case J1850VPW_ERR_INVALID_MESSAGE_LENGTH:
        {
            Serial.println(s + "J1850VPW_ERR_INVALID_MESSAGE_LENGTH");
            break;
        }
        case J1850VPW_ERR_BUS_IS_BUSY:
        {
            Serial.println(s + "J1850VPW_ERR_BUS_IS_BUSY");
            break;
        }
        case J1850VPW_ERR_BUS_ERROR:
        {
            Serial.println(s + "J1850VPW_ERR_BUS_ERROR");
            break;
        }
        case J1850VPW_ERR_ARBITRATION_LOST:
        {
            Serial.println(s + "J1850VPW_ERR_ARBITRATION_LOST");
            break;
        }
        case J1850VPW_ERR_IFR_NOT_SUPPORTED:
        {
            Serial.println(s + "J1850VPW_ERR_IFR_NOT_SUPPORTED");
            break;
        }
        case J1850VPW_ERR_CRC:
        {
            Serial.println(s + "J1850VPW_ERR_CRC");
            break;
        }
        default: // unknown error
        {
            Serial.println(s + "ERR: " + String(err, HEX));
            break;
        }
    }
}

void setup()
{
    Serial.begin(250000);

    // MC33390 settings.
    pinMode(SLEEP, OUTPUT);
    digitalWrite(SLEEP, HIGH); // enable PCI-bus transmitter (receiver is always enabled), HIGH: enable, LOW: disable
    //pinMode(_4XLOOP, INPUT); // PCI-bus loopback mode: transmitted message is immediately received (for testing only)
    pinMode(_4XLOOP, OUTPUT);
    digitalWrite(_4XLOOP, LOW); // PCI-bus waveshaping - LOW: enabled, HIGH: disabled
    VPW.onMessageReceived(PCIMessageReceived); // subscribe to the message received event and call this function when a PCI-bus message is received
    VPW.onError(PCIHandleError);
    VPW.begin(J1850VPW_RX, J1850VPW_TX, ACTIVE_HIGH);
    VPW.listen(messageFilter); // display selected messages only
    //VPW.ignore(messageFilter); // ignore selected messages
    //VPW.listenAll(); // disable message filtering
    //VPW.ignoreAll(); // don't let any message through
}

void loop()
{
    // Empty.
}
