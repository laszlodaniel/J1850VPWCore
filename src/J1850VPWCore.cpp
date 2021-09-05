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

static void isrVPWProtocolDecoder()
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
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
    if ((_rxPin != 2)  && (_rxPin != 3)  && (_rxPin != 18) && (_rxPin != 19) && (_rxPin != 20) && (_rxPin != 21) && (_rxPin != 71) && (_rxPin != 72)) return false; // Pin 2, 3, 18, 19, 20, 21, 71 or 72 must be selected as RX-pin!
#endif

    protocolEncoderInit();
    pinMode(_txPin, OUTPUT); // set TX pin as output
    digitalWrite(_txPin, _passiveLevel); // keep bus at passive level

    // Calculate top values to count for timers.
    _timerOCRValue_J1850VPW_TX_SOF = (uint16_t)(((float)F_CPU / ((1000000.0/(float)J1850VPW_TX_SOF) * 64.0)) - 1.0);
    _timerOCRValue_J1850VPW_TX_SRT = (uint16_t)(((float)F_CPU / ((1000000.0/(float)J1850VPW_TX_SRT) * 64.0)) - 1.0);
    _timerOCRValue_J1850VPW_TX_LNG = (uint16_t)(((float)F_CPU / ((1000000.0/(float)J1850VPW_TX_LNG) * 64.0)) - 1.0);

    pinMode(_rxPin, INPUT); // set RX pin as input (must be external interrupt capable)
    busIdleTimerInit();
    busIdleTimerStart();
    listenAll();
    attachInterrupt(digitalPinToInterrupt(_rxPin), isrVPWProtocolDecoder, CHANGE);

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
    busIdleTimerInit();
    busIdleTimerStart();
    listenAll();
    attachInterrupt(digitalPinToInterrupt(_rxPin), isrVPWProtocolDecoder, CHANGE);

    return true;
}

void J1850VPWCore::protocolEncoderInit()
{
    // Setup Timer 1 (Uno) or Timer 4 (Mega) to signal TX pin state changes in accordance with J1850 VPW timing requirements.
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
        TCCR1A = 0; // clear register
        TCCR1B = 0; // clear register
        TCNT1 = 0; // clear counter
        TIMSK1 |= (1 << OCIE1A); // enable Output Compare Match A Interrupt
        OCR1A = 0; // top value to count
        TCCR1B |= (1 << WGM12); // CTC mode, stop timer
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
        TCCR4A = 0; // clear register
        TCCR4B = 0; // clear register
        TCNT4 = 0; // clear counter
        TIMSK4 |= (1 << OCIE4A); // enable Output Compare Match A Interrupt
        OCR4A = 0; // top value to count
        TCCR4B |= (1 << WGM42); // CTC mode, stop timer
#endif
    }
}

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
ISR(TIMER1_COMPA_vect)
{
    VPW.protocolEncoder();
}
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
ISR(TIMER4_COMPA_vect)
{
    VPW.protocolEncoder();
}
#endif

void J1850VPWCore::protocolEncoder()
{
    _busIdle = false;
    
    if (_sofWrite)
    {
        _sofWrite = false;
        _bitWrite = true;
        _currentState = _activeLevel;

        digitalWrite(_txPin, _activeLevel);

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
        OCR1A = _timerOCRValue_J1850VPW_TX_SOF; // 200 us
        TCNT1 = 0; // reset counter;
        TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler 64, start timer
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
        OCR4A = _timerOCRValue_J1850VPW_TX_SOF; // 200 us
        TCNT4 = 0; // reset counter;
        TCCR4B |= (1 << CS41) | (1 << CS40); // prescaler 64, start timer
#endif

        return;
    }

    if (_bitWrite)
    {
        _currentState = !_currentState; // alternate between passive and active states
        _currentTxBit = _txBuffer[_txBufferPos] & (1 << (7 - _bitNumWrite));

        if (_currentState != _lastState)
        {
            _bitWrite = false;
            digitalWrite(_txPin, _passiveLevel);
            busIdleTimerStart(); // look for bus-idle

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
            TCCR1B &= ~(1 << CS11) & ~(1 << CS10); // clear prescaler to stop timer
            TCNT1 = 0; // reset counter;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
            TCCR4B &= ~(1 << CS41) & ~(1 << CS40); // clear prescaler to stop timer
            TCNT4 = 0; // reset counter;
#endif

            handleErrorsInternal(J1850VPW_Write, J1850VPW_ERR_ARBITRATION_LOST); // report error
            return;
        }

        if (_currentState == _passiveLevel) // passive bus
        {
            digitalWrite(_txPin, _passiveLevel);

            if (_currentTxBit) // logic 1
            {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
                OCR1A = _timerOCRValue_J1850VPW_TX_LNG; // 128 us
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
                OCR4A = _timerOCRValue_J1850VPW_TX_LNG; // 128 us
#endif
            }
            else // logic 0
            {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
                OCR1A = _timerOCRValue_J1850VPW_TX_SRT; // 64 us
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
                OCR4A = _timerOCRValue_J1850VPW_TX_SRT; // 64 us
#endif
            }
        }
        else // active bus
        {
            digitalWrite(_txPin, _activeLevel);

            if (_currentTxBit) // logic 1
            {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
                OCR1A = _timerOCRValue_J1850VPW_TX_SRT; // 64 us
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
                OCR4A = _timerOCRValue_J1850VPW_TX_SRT; // 64 us
#endif
            }
            else // logic 0
            {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
                OCR1A = _timerOCRValue_J1850VPW_TX_LNG; // 128 us
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
                OCR4A = _timerOCRValue_J1850VPW_TX_LNG; // 128 us
#endif
            }
        }

        _bitNumWrite++;

        if (_bitNumWrite == 8)
        {
            _bitNumWrite = 0;
            _txBufferPos++;

            if (_txBufferPos == _txLength)
            {
                _bitWrite = false;
                _eofWrite = true;
            }
        }

        return;
    }

    if (_eofWrite)
    {
        _eofWrite = false;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino Uno
        TCCR1B &= ~(1 << CS11) & ~(1 << CS10); // clear prescaler to stop timer
        TCNT1 = 0; // reset counter;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
        TCCR4B &= ~(1 << CS41) & ~(1 << CS40); // clear prescaler to stop timer
        TCNT4 = 0; // reset counter;
#endif

        digitalWrite(_txPin, _passiveLevel);

        if (_lastState == _activeLevel)
        {
            _bitWrite = false;
            handleErrorsInternal(J1850VPW_Write, J1850VPW_ERR_ARBITRATION_LOST); // report error
        }
    }
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

    bool timeout = false;
    uint32_t timeoutStart = millis();

    while (!_busIdle && !timeout)
    {
        if ((uint32_t)(millis() - timeoutStart) >= 1000) timeout = true;
    }

    if (timeout)
    {
        handleErrorsInternal(J1850VPW_Write, J1850VPW_ERR_BUS_IS_BUSY);
        return;
    }

    _sofWrite = true;
    _bitWrite = false;
    _bitNumWrite = 0;
    _eofWrite = false;

    protocolEncoder();
}

void J1850VPWCore::protocolDecoder()
{
    uint32_t now = micros();
    uint32_t diff = (uint32_t)(now - _lastChange);
    _lastChange = now;
    _lastState = !digitalRead(_rxPin);

    if (!_sofRead && (_lastState == _activeLevel))
    {
        _busIdle = false;
        busIdleTimerStart(); // look for bus-idle
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
            _IFRDetected = false;
        }

        return;
    }

    if (_lastState == _passiveLevel)
    {
        busIdleTimerStart(); // look for the end of the frame
        
        if (!_IFRDetected && IS_BETWEEN(diff, J1850VPW_RX_EOD_MIN, J1850VPW_RX_EOD_MAX)) // data ended and IFR detected
        {
            _IFRDetected = true; // set flag to ignore incoming IFR
            processMessage();
            handleErrorsInternal(J1850VPW_Read, J1850VPW_ERR_IFR_NOT_SUPPORTED); // report error
            return;
        }

        if (!_IFRDetected)
        {
            if (IS_BETWEEN(diff, J1850VPW_RX_LNG_MIN, J1850VPW_RX_LNG_MAX))
            {
                _rxBuffer[_rxBufferPos] |= (1 << (7 - _bitPos)); // save passive 1 bit
                _currentRxBit = 1;
            }
            else if (diff <= J1850VPW_RX_SRT_MAX)
            {
                _currentRxBit = 0;
            }
        }
    }
    else // active level
    {
        if (!_IFRDetected)
        {
            if (diff <= J1850VPW_RX_SRT_MAX)
            {
                _rxBuffer[_rxBufferPos] |= (1 << (7 - _bitPos)); // save active 1 bit
                _currentRxBit = 1;
            }
            else if (IS_BETWEEN(diff, J1850VPW_RX_LNG_MIN, J1850VPW_RX_LNG_MAX))
            {
                _currentRxBit = 0;
            }
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
    if (_rxBufferPos > 1) // at least 2 bytes are necessary for a valid frame
    {
        uint8_t *pByte, bit;

        pByte = getBit(_rxBuffer[0], &bit); // check if ID-byte is on ignore list
        
        if (pByte && (*pByte & (1 << bit)) == 0) // ID-byte is not on ignore list
        {
            if (_rxBuffer[_rxBufferPos - 1] == CRC(_rxBuffer, _rxBufferPos - 1)) // CRC ok
            {
                for (uint8_t i = 0; i < _rxBufferPos; i++) _message[i] = _rxBuffer[i]; // copy receive buffer to message buffer

                _messageLength = _rxBufferPos; // save message length
                handleMessagesInternal(_message, _messageLength); // raise event
            }
            else // CRC error
            {
                handleErrorsInternal(J1850VPW_Read, J1850VPW_ERR_CRC); // report error
            }
        }
    }

    memset(_rxBuffer, 0, sizeof(_rxBuffer)); // clear receive buffer
    _rxBufferPos = 0;
    _sofRead = false;
}

void J1850VPWCore::busIdleTimerInit()
{
    // Setup Timer 2 to measure ~282 us time interval to decide if bus is idle.
    uint8_t _calculatedOCR2AValue = (uint8_t)(roundf(((float)F_CPU / ((1000000.0/(float)J1850VPW_RX_IFS_MIN) * 32.0)) - 1.0));

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
    if (_sofRead) processMessage();
}

void J1850VPWCore::listenAll()
{
    memset(_ignoreList, 0, sizeof(_ignoreList));
}

void J1850VPWCore::listen(uint8_t* ids)
{
    uint8_t *pByte, bit;

    while (*ids)
    {
        pByte = getBit(*ids, &bit);

        if (pByte)
        {
            *pByte &= ~(1 << bit);
        }

        ids++;
    }
}

void J1850VPWCore::ignoreAll()
{
    memset(_ignoreList, 0xFF, sizeof(_ignoreList));
}

void J1850VPWCore::ignore(uint8_t* ids)
{
    uint8_t *pByte, bit;

    while (*ids)
    {
        pByte = getBit(*ids, &bit);

        if (pByte)
        {
            *pByte |= 1 << bit;
        }

        ids++;
    }
}

uint8_t* J1850VPWCore::getBit(uint8_t _id, uint8_t* _pBit)
{
    if (!_id)
    {
        *_pBit = 0xFF;
        return NULL;
    }

    *_pBit = _id % 8;
    return &(_ignoreList[_id / 8]);
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
    onVPWMessageReceivedHandler msgHandler = _msgHandler;

    if (msgHandler)
    {
        msgHandler(message, messageLength); // raise event
    }
}

void J1850VPWCore::onMessageReceived(onVPWMessageReceivedHandler msgHandler)
{
    _msgHandler = msgHandler;
}

void J1850VPWCore::handleErrorsInternal(J1850VPW_Operations _op, J1850VPW_Errors _err)
{
    if (_err != J1850VPW_OK)
    {
        onVPWErrorHandler errHandler = _errHandler;

        if (errHandler)
        {
            errHandler(_op, _err); // raise event
        }
    }
}

void J1850VPWCore::onError(onVPWErrorHandler errHandler)
{
    _errHandler = errHandler;
}
