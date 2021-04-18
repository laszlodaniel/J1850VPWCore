/*
 * J1850VPWCore (https://github.com/laszlodaniel/J1850VPWCore)
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

#ifndef J1850VPWCore_H
#define J1850VPWCore_H

#include <Arduino.h>

#define ACTIVE_HIGH HIGH
#define ACTIVE_LOW LOW

#define WAIT_100us 100
#define TX_SHORT 64
#define TX_LONG 128
#define TX_SOF 200
#define TX_EOF 280
#define RX_SHORT_MIN 34
#define RX_SHORT_MAX 96
#define RX_LONG_MIN 96
#define RX_LONG_MAX 163
#define RX_SOF_MIN 163
#define RX_SOF_MAX 239
#define RX_EOD_MIN 163
#define RX_IFS_MIN 280

#define TX_OK 1
#define RX_OK 2
#define ERROR_MESSAGE_TOO_LONG 3
#define ERROR_NO_RESPONSE_WITHIN_100US 4
#define ERROR_SOF_TIMEOUT 5
#define ERROR_SYMBOL_WAS_NOT_SOF 6
#define ERROR_SYMBOL_WAS_TOO_SHORT 7
#define ERROR_CRC_FAILURE 8

#define DEBUG_ON 1
#define DEBUG_OFF 0

class J1850VPWCore
{
    public:
        J1850VPWCore();
	void init(uint8_t rxPin, uint8_t txPin, uint8_t activeLevel = HIGH, bool debug = false);
	bool accept(uint8_t *msg_buf, bool crc_c = true);
	bool send(uint8_t *msg_buf, uint8_t nbytes);
	bool easy_send(uint8_t size, ...);
	uint8_t message;
	uint8_t rx_nbyte = 0;
	uint8_t tx_nbyte = 0;

    private:
        uint8_t _rxPin;
        uint8_t _txPin;
        uint8_t _activeLevel = HIGH;
        uint8_t _passiveLevel = LOW;
	bool _debug = false;
	bool _if_init = false;
	uint8_t _mode = 0;
	uint8_t *_rx_msg_buf;
	uint8_t *_tx_msg_buf;
	uint32_t _time_tmp = 0;

	void start_timer(void);
	void monitor(void);
	void sendToUART(const char *, uint8_t, uint8_t *);
	void tests(void);
	void active(void);
	void passive(void);
	bool is_active(void);
	bool recv_msg(uint8_t *);
	bool send_msg(uint8_t *, uint8_t);
	uint32_t read_timer(void);
	uint8_t crc(uint8_t *, uint8_t);
};

#endif