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

#include <J1850VPWCore.h>

J1850VPWCore::J1850VPWCore()
{

}

void J1850VPWCore::init(uint8_t rxPin, uint8_t txPin, uint8_t activeLevel, bool debug)
{
    _rxPin = rxPin;
    _txPin = txPin;
    _activeLevel = activeLevel;
    _passiveLevel = !_activeLevel;
    _debug = debug;

    if (_debug)
    {
        _mode = 1;
    }

    pinMode(_rxPin, INPUT);
    pinMode(_txPin, OUTPUT);
    passive();

    _if_init = true;
}

bool J1850VPWCore::accept(uint8_t *msg_buf, bool crc_c)
{
    if (!_if_init) return false;

    bool f = recv_msg(msg_buf);

    if ((crc_c) && (f))
    {
        if (msg_buf[rx_nbyte - 1] != crc(msg_buf, rx_nbyte - 1))
        {
            f = false;
            message = ERROR_CRC_FAILURE;
        }
    }

    if (_debug)
    {
        if (Serial.available() >= 2) _mode = Serial.parseInt();
        monitor();
    }

    return f;
}

bool J1850VPWCore::easy_send(uint8_t size, ...)
{
    if (size == 0) return false;

    uint8_t *buffer = new uint8_t[size];
    bool result = false;
    va_list ap;

    va_start(ap, size);

    for (uint8_t i = 0; i < size; i++) buffer[i] = va_arg(ap, int);

    va_end(ap);

    result = send(buffer, size);
    delete[] buffer;
    return result;
}

bool J1850VPWCore::send(uint8_t *msg_buf, uint8_t nbytes)
{
    if (!_if_init) return false;

    msg_buf[nbytes] = crc(msg_buf, nbytes);
    nbytes++;

    bool f = send_msg(msg_buf, nbytes);

    if (_debug) monitor();

    return f;
}

bool J1850VPWCore::recv_msg(uint8_t *msg_buf)
{
    uint8_t nbits, nbytes;
    bool bit_state;
    _rx_msg_buf = msg_buf;

    start_timer();
    while (!is_active())
    {
        if (read_timer() > WAIT_100us)
        {
            message = ERROR_NO_RESPONSE_WITHIN_100US;
            return false;
        }
    }

    start_timer();

    while (is_active())
    {
        if (read_timer() > RX_SOF_MAX)
        {
            message = ERROR_SOF_TIMEOUT;
            return false;
        }
    }

    if (read_timer() < RX_SOF_MIN)
    {
        message = ERROR_SYMBOL_WAS_NOT_SOF;
        return false;
    }

    bit_state = is_active();
    start_timer();

    for (nbytes = 0; nbytes < 12; ++nbytes)
    {
        nbits = 8;
        do
        {
            *msg_buf <<= 1;
            while (is_active() == bit_state)
            {
                if (read_timer() > RX_EOD_MIN)
                {
                    rx_nbyte = nbytes;
                    message = RX_OK;
                    return true;
                }
            }

            bit_state = is_active();
            uint32_t tcnt1_buf = read_timer();
            start_timer();

            if (tcnt1_buf < RX_SHORT_MIN)
            {
                message = ERROR_SYMBOL_WAS_TOO_SHORT;
                return false;
            }

            if ((tcnt1_buf < RX_SHORT_MAX) && !is_active()) *msg_buf |= 1;
            if ((tcnt1_buf > RX_LONG_MIN) && (tcnt1_buf < RX_LONG_MAX) && is_active()) *msg_buf |= 1;

        } while (--nbits);
        ++msg_buf;
    }

    rx_nbyte = nbytes;
    message = RX_OK;
    return true;
}

bool J1850VPWCore::send_msg(uint8_t *msg_buf, uint8_t nbytes)
{
    uint8_t nbits;
    uint8_t temp_byte;
    _tx_msg_buf = msg_buf;
    tx_nbyte = nbytes;

    if (nbytes > 12)
    {
        message = ERROR_MESSAGE_TOO_LONG;
        return false;
    }

    start_timer();

    while (read_timer() < RX_IFS_MIN)
    {
        if (is_active()) start_timer();
    }

    active();
    delayMicroseconds(TX_SOF);

    do
    {
        temp_byte = *msg_buf;
        nbits = 8;

        while (nbits--)
        {
            if (nbits & 1)
            {
                passive();
                delayMicroseconds((temp_byte & 0x80) ? TX_LONG : TX_SHORT);
            }
            else
            {
                active();
                delayMicroseconds((temp_byte & 0x80) ? TX_SHORT : TX_LONG);
            }

            temp_byte <<= 1;
        }
        ++msg_buf;
    } while (--nbytes);

    passive();
    delayMicroseconds(TX_EOF);
    message = TX_OK;
    return true;
}

void J1850VPWCore::monitor(void)
{
    static uint8_t old_message;
	
    switch (_mode)
    {
        //tests
        case 5:
            tests();
            _mode = 1; //default mode
            break;

        //RX
        case 4:
            if (message == RX_OK) sendToUART("RX: ", rx_nbyte, _rx_msg_buf);
            break;

        //TX
        case 3:
            if (message == TX_OK) sendToUART("TX: ", tx_nbyte, _tx_msg_buf);
            break;

        //status codes
        case 2:
            if (old_message != message)
            {
                Serial.println(message);
                old_message = message;
            }
            break;

        //RX\TX
        case 1:
            if (message == TX_OK) sendToUART("TX: ", tx_nbyte, _tx_msg_buf);
            if (message == RX_OK) sendToUART("RX: ", rx_nbyte, _rx_msg_buf);
            break;

        default:
            break;
    }
}

void J1850VPWCore::sendToUART(const char *header, uint8_t rx_nbyte, uint8_t *msg_buf)
{
    Serial.print(header);

    for (uint8_t i = 0; i < rx_nbyte; i++)
    {
        if (msg_buf[i] < 0x10) Serial.print(0);

        Serial.print(msg_buf[i], HEX);

        if (i == (rx_nbyte - 1))
        {
            Serial.print("\n");
        }
        else
        {
            Serial.print(" ");
        }
    }
}

void J1850VPWCore::tests(void)
{
    char fail[] = "Test failure!\n";
    char ok[] = "Test success!\n";

    Serial.print("----Start I/O test----\n");
    if (!is_active())
    {
        active();
        delayMicroseconds(32);

        if (is_active())
        {
            Serial.print(ok);
        }
        else
        {
            Serial.print(fail);
        }

        passive();

    }
    else
    {
        Serial.print(fail);
    }

    Serial.print("----End I/O test----\n\n");
}

void J1850VPWCore::active(void)
{
    digitalWrite(_txPin, _activeLevel);
}

void J1850VPWCore::passive(void)
{
    digitalWrite(_txPin, _passiveLevel);
}

bool J1850VPWCore::is_active(void)
{
    return digitalRead(_rxPin);
}

uint8_t J1850VPWCore::crc(uint8_t *msg_buf, uint8_t nbytes)
{
    uint8_t crc = 0xFF;

    while (nbytes--)
    {
        crc ^= *msg_buf++;
        for (uint8_t i = 0; i < 8; i++) crc = crc & 0x80 ? (crc << 1) ^ 0x1D : crc << 1;
    }

    crc ^= 0xFF;
    return crc;
}

void J1850VPWCore::start_timer(void)
{
    _time_tmp = micros();
}

uint32_t J1850VPWCore::read_timer(void)
{
    return micros() - _time_tmp;
}
