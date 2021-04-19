/*
 * J1850_VPW_transmitter.ino (https://github.com/laszlodaniel/J1850VPWCore)
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

J1850VPWCore VPW;

#define J1850_VPW_RX (10)
#define J1850_VPW_TX (11)
#define SLEEP        (5)
#define _4XLOOP      (6)

uint8_t tx_buf[6] = { 0x48, 0x69, 0x2D, 0x4D, 0x32, 0x21 };

void setup()
{
    Serial.begin(250000);

    // MC33390 settings
    pinMode(SLEEP, OUTPUT);
    digitalWrite(SLEEP, HIGH); // enable PCI-bus transmitter (receiver is always enabled), HIGH: enable, LOW: disable
    //pinMode(6, INPUT); // PCI-bus loopback mode: transmitted message is immediately received (for testing only)
    pinMode(_4XLOOP, OUTPUT);
    digitalWrite(_4XLOOP, LOW); // PCI-bus waveshaping - LOW: enabled, HIGH: disabled

    VPW.init(J1850_VPW_RX, J1850_VPW_TX, ACTIVE_HIGH, DEBUG_ON);
}

void loop()
{
    if (VPW.send(tx_buf, sizeof(tx_buf)))
    {

    }

    delay(500);
}
