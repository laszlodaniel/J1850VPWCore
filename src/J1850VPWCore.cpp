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
#include <util/atomic.h>
#include <util/delay.h>

J1850VPWCore VPW;

J1850VPWCore::J1850VPWCore()
{
    // Empty class constructor.
}

J1850VPWCore::~J1850VPWCore()
{
    // Empty class destructor.
}

static void isrProtocolDecoder()
{
    VPW.protocolDecoder();
}

bool J1850VPWCore::begin(uint8_t rxPin, uint8_t txPin, bool activeLevel)
{
    _rxPin = rxPin;
    _txPin = txPin;
    _readOnly = false;
    _activeLevel = activeLevel;
    _passiveLevel = !_activeLevel;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
    if ((_rxPin != 2) && (_rxPin != 3)) return false; // Pin 2 or 3 must be selected as RX-pin!
    if ((_txPin != 9) && (_txPin != 10)) return false; // Pin 9 or 10 must be selected as TX-pin!
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
    if ((_rxPin != 2)  && (_rxPin != 3)  && (_rxPin != 18) && (_rxPin != 19) && (_rxPin != 20) && (_rxPin != 21) && (_rxPin != 71) && (_rxPin != 72)) return false; // Pin 2, 3, 18, 19, 20, 21, 71 or 72 must be selected as RX-pin!
    if ((_txPin != 11) && (_txPin != 12) && (_txPin != 13) && (_txPin != 2)  && (_txPin != 3)  && (_txPin != 5)  && (_txPin != 6)  && (_txPin != 7)  && (_txPin != 8)  && (_txPin != 44) && (_txPin != 45) && (_txPin != 46)) return false; // Pin 2, 3, 5, 6, 7, 8, 11, 12, 13, 44, 45 or 46 must be selected as TX-pin!
#endif

    pinMode(_txPin, OUTPUT); // set TX pin as output (must be a 16-bit timer pin)
    if (_activeLevel) digitalWrite(_txPin, LOW); // pull TX pin low
    else digitalWrite(_txPin, HIGH); // pull TX pin high

    protocolEncoderInit();
    pinMode(_rxPin, INPUT); // set RX pin as input (must be external interrupt capable)
    attachInterrupt(digitalPinToInterrupt(_rxPin), isrProtocolDecoder, CHANGE);
    busIdleTimerInit();
    listenAll();

    return true;
}

bool J1850VPWCore::begin(uint8_t rxPin, bool activeLevel)
{
    _rxPin = rxPin;
    _readOnly = true;
    _activeLevel = activeLevel;
    _passiveLevel = !_activeLevel;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
    if ((_rxPin != 2) && (_rxPin != 3)) return false; // Pin 2 or 3 must be selected as RX-pin!
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
    if ((_rxPin != 2)  && (_rxPin != 3)  && (_rxPin != 18) && (_rxPin != 19) && (_rxPin != 20) && (_rxPin != 21) && (_rxPin != 71) && (_rxPin != 72)) return false; // Pin 2, 3, 18, 19, 20, 21, 71 or 72 must be selected as RX-pin!
#endif

    pinMode(_rxPin, INPUT); // set RX pin as input (must be external interrupt capable)
    attachInterrupt(digitalPinToInterrupt(_rxPin), isrProtocolDecoder, CHANGE);
    busIdleTimerInit();
    listenAll();

    return true;
}

void J1850VPWCore::protocolEncoderInit()
{
    // Setup Timer 4 to toggle PCI_TX pin in accordance with J1850 VPW timing requirements.
    // The top value to count (OCR4A) and prescaler (TCCR4B) registers will be modified on-the-fly.
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        TCCR4A = 0; // clear register
        TCCR4B = 0; // clear register
        TCNT4 = 0; // clear counter
        TCCR4A |= (1 << COM4A0); // toggle OC4A on compare match
        TIMSK4 |= (1 << OCIE4A); // enable Output Compare Match A Interrupt
        OCR4A = 0; // top value to count
        TCCR4B |= (1 << WGM42); // CTC mode, stop timer
    }
}

ISR(TIMER4_COMPA_vect)
{
    VPW.protocolEncoder();
}

void J1850VPWCore::protocolEncoder()
{
    // TODO
}

void J1850VPWCore::write(uint8_t* buffer, uint8_t bufferLength)
{
    if (_readOnly)
    {
        handleErrorsInternal(J1850VPW_Write, J1850VPW_ERR_TX_DISABLED);
        return;
    }

    if ((bufferLength == 0) || (bufferLength > MAX_DATA_LENGTH))
    {
        handleErrorsInternal(J1850VPW_Write, J1850VPW_ERR_INVALID_MESSAGE_LENGTH);
        return;
    }

    for (uint8_t i = 0; i < bufferLength; i++) _txBuffer[i] = buffer[i]; // copy message bytes to the transmit buffer

    _txBuffer[bufferLength] = CRC(_txBuffer, bufferLength); // calculate CRC and append to the message
    _txBufferPos = 0; // reset transmit buffer position
    _txLength = bufferLength + 1; // save message length (+1 CRC byte)

    // TODO: trigger message transmission

    handleErrorsInternal(J1850VPW_Write, J1850VPW_ERR_TX_DISABLED);
}

void J1850VPWCore::protocolDecoder()
{
    uint32_t now = micros();
    uint32_t diff = (uint32_t)(now - _lastChange);
    _lastChange = now;
    _busIdle = false;

    if (digitalRead(_rxPin)) // RX pin transitioned from low to high
    {
        _lastState = 0; // save last state
    }
    else // RX pin transitioned from high to low
    {
        _lastState = 1; // save last state
    }

    if (diff < J1850VPW_RX_SRT_MIN) // too short to be a valid pulse
    {
        _sofRead = false;
        return;
    }

    if (!_sofRead)
    {
        if ((_sofRead = ((_lastState == _activeLevel) && IS_BETWEEN(diff, J1850VPW_RX_SOF_MIN, J1850VPW_RX_SOF_MAX))))
        {
            _bitPos = 0;
            _rxBufferPos = 0;
            _IFRDetected = false;
        }

        return;
    }

    if (_lastState == _passiveLevel)
    {
        busIdleTimerStart();
        
        if (!_IFRDetected && IS_BETWEEN(diff, J1850VPW_RX_EOD_MIN, J1850VPW_RX_EOD_MAX)) // data ended and IFR detected
        {
            _IFRDetected = true; // set flag to ignore the incoming IFR
            processMessage();
            handleErrorsInternal(J1850VPW_Read, J1850VPW_ERR_IFR_NOT_SUPPORTED); // report error
            return;
        }

        if (!_IFRDetected && IS_BETWEEN(diff, J1850VPW_RX_LNG_MIN, J1850VPW_RX_LNG_MAX))
        {
            _rxBuffer[_rxBufferPos] |= (1 << (7 - _bitPos)); // save passive 1 bit
        }
    }
    else // active level
    {
        if (!_IFRDetected && (diff <= J1850VPW_RX_SRT_MAX))
        {
            _rxBuffer[_rxBufferPos] |= (1 << (7 - _bitPos)); // save active 1 bit
        }
    }

    if(!_IFRDetected)
    {
        _bitPos++;

        if (_bitPos == 8)
        {
            _rxBufferPos++;
            _bitPos = 0;
        }
    }

    if (_rxBufferPos == MAX_FRAME_LENGTH)
    {
        processMessage();
    }
}

void J1850VPWCore::processMessage()
{
    if ((_rxBufferPos > 1) && (_ignoreList[_rxBuffer[0]] == 0))
    {
        if (_rxBuffer[_rxBufferPos - 1] == CRC(_rxBuffer, _rxBufferPos - 1)) // CRC ok
        {
            for (uint8_t i = 0; i < _rxBufferPos; i++) _message[i] = _rxBuffer[i]; // copy receive buffer to message buffer

            _messageLength = _rxBufferPos; // save message length
            memset(_rxBuffer, 0, sizeof(_rxBuffer)); // clear receive buffer

            handleMessagesInternal(_message, _messageLength);
        }
        else // CRC error
        {
            handleErrorsInternal(J1850VPW_Read, J1850VPW_ERR_CRC);
        }
    }

    _sofRead = false;
}

void J1850VPWCore::busIdleTimerInit()
{
    // Setup Timer 2 to measure ~282 us time interval to decide if bus is idle.
    _calculatedOCR2AValue = (uint8_t)(roundf(((float)F_CPU / ((1000000.0/(float)J1850VPW_RX_IFS_MIN) * 32.0)) - 1.0));

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        TCCR2A = 0; // clear register
        TCCR2B = 0; // clear register
        TCNT2 = 0; // clear counter
        OCR2A = _calculatedOCR2AValue; // top value to count
        TCCR2A |= (1 << WGM21); // CTC mode
        TIMSK2 |= (1 << OCIE2A); // enable Output Compare Match A Interrupt
    }
}

void J1850VPWCore::busIdleTimerStart()
{
    TCNT2 = 0; // clear counter
    TCCR2B |= (1 << CS21) | (1 << CS20); // prescaler 32, start timer
}

void J1850VPWCore::busIdleTimerStop()
{
    TCCR2B &= ~(1 << CS21) & ~(1 << CS20); // clear prescaler to stop timer
    TCNT2 = 0; // clear counter
}

ISR(TIMER2_COMPA_vect)
{
    VPW.busIdleTimerHandler();
}

void J1850VPWCore::busIdleTimerHandler()
{
    busIdleTimerStop();
    _busIdle = true;
    processMessage();
}

void J1850VPWCore::listenAll()
{
    memset(_ignoreList, 0, sizeof(_ignoreList));
}

void J1850VPWCore::listen(uint8_t* ids)
{
    ignoreAll();

    while (*ids)
    {
        _ignoreList[*ids] = 0; // clear ignore flag
        ids++;
    }
}

void J1850VPWCore::ignoreAll()
{
    memset(_ignoreList, 1, sizeof(_ignoreList));
}

void J1850VPWCore::ignore(uint8_t* ids)
{
    listenAll();

    while (*ids)
    {
        _ignoreList[*ids] = 1; // set ignore flag
        ids++;
    }
}

uint8_t J1850VPWCore::CRC(uint8_t* _buff, uint8_t _nbytes)
{
    uint8_t crc = 0xFF;

    while (_nbytes--)
    {
        crc ^= *_buff++;
        for (uint8_t i = 0; i < 8; i++) crc = crc & 0x80 ? (crc << 1) ^ 0x1D : crc << 1;
    }

    crc ^= 0xFF;
    return crc;
}

void J1850VPWCore::handleMessagesInternal(uint8_t* message, uint8_t messageLength)
{
    onJ1850VPWMessageReceivedHandler msgHandler = _msgHandler;

    if (msgHandler)
    {
        msgHandler(message, messageLength); // raise event
    }
}

void J1850VPWCore::onMessageReceived(onJ1850VPWMessageReceivedHandler msgHandler)
{
    _msgHandler = msgHandler;
}

void J1850VPWCore::handleErrorsInternal(J1850VPW_Operations _op, J1850VPW_Errors _err)
{
    if (_err != J1850VPW_OK)
    {
        onJ1850VPWErrorHandler errHandler = _errHandler;

        if (errHandler)
        {
            errHandler(_op, _err); // raise event
        }
    }
}

void J1850VPWCore::onError(onJ1850VPWErrorHandler errHandler)
{
    _errHandler = errHandler;
}
