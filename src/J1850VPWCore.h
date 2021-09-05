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

#define MAX_FRAME_LENGTH 12
#define MAX_DATA_LENGTH (MAX_FRAME_LENGTH - 1)

#define ACTIVE_HIGH HIGH
#define ACTIVE_LOW LOW

#define MAX(a, b) ((a) < (b) ? (b) : (a))
#define IS_BETWEEN(x, min, max) ((x >= min) && (x <= max))

enum J1850VPW_Operations
{
    J1850VPW_Read,
    J1850VPW_Write
};

// J1850 VPW Network Elements and Structure:
// - with IFR:    {IDLE}[SOF][DATA_0][DATA_1]...[DATA_N][CRC][EOD][NB][IFR_0][IFR_1]...[IFR_N][CRC][(EOF)IFS]{IDLE}
// - without IFR: {IDLE}[SOF][DATA_0][DATA_1]...[DATA_N][CRC][(EOF)IFS]{IDLE}

enum J1850VPW_Symbols // time intervals in microseconds (us)
{
    J1850VPW_TX_SRT = 64,  // Short
    J1850VPW_TX_LNG = 128, // Long
    J1850VPW_TX_SOF = 200, // Start of Frame
    J1850VPW_TX_EOD = 200, // End of Data, only when IFR (Inter-Frame Response) is used
    J1850VPW_TX_EOF = 280, // End of Frame
    J1850VPW_TX_BRK = 300, // Break
    J1850VPW_TX_IFS = 300, // Inter-Frame Separation
    J1850VPW_RX_SRT_MIN = 35,
    J1850VPW_RX_SRT_MAX = 96,
    J1850VPW_RX_LNG_MIN = 97,
    J1850VPW_RX_LNG_MAX = 163,
    J1850VPW_RX_SOF_MIN = 164,
    J1850VPW_RX_SOF_MAX = 239,
    J1850VPW_RX_EOD_MIN = 164,
    J1850VPW_RX_EOD_MAX = 239,
    J1850VPW_RX_EOF_MIN = 240,
    J1850VPW_RX_BRK_MIN = 240,
    J1850VPW_RX_BRK_MAX = 1000000,
    J1850VPW_RX_IFS_MIN = 281
};

enum J1850VPW_Errors
{
    J1850VPW_OK                         = 0x00,
    J1850VPW_ERR_TX_DISABLED            = 0x80,
    J1850VPW_ERR_INVALID_MESSAGE_LENGTH = 0x81,
    J1850VPW_ERR_BUS_IS_BUSY            = 0x82,
    J1850VPW_ERR_BUS_ERROR              = 0x83,
    J1850VPW_ERR_ARBITRATION_LOST       = 0x84,
    J1850VPW_ERR_IFR_NOT_SUPPORTED      = 0x85,
    J1850VPW_ERR_CRC                    = 0x86
};

typedef void (*onVPWMessageReceivedHandler)(uint8_t* message, uint8_t messageLength);
typedef void (*onVPWErrorHandler)(J1850VPW_Operations op, J1850VPW_Errors err);

class J1850VPWCore
{
    public:
        J1850VPWCore();
        ~J1850VPWCore();
        bool begin(uint8_t rxPin, uint8_t txPin, bool activeLevel);
        bool begin(uint8_t rxPin, bool activeLevel);
        void write(uint8_t* buffer, uint8_t bufferLength);
        void listenAll();
        void listen(uint8_t* ids);
        void ignoreAll();
        void ignore(uint8_t* ids);
        void protocolEncoder();
        void protocolDecoder();
        void busIdleTimerHandler();
        void onMessageReceived(onVPWMessageReceivedHandler msgHandler);
        void onError(onVPWErrorHandler errHandler);

    private:
        uint8_t _rxPin;
        uint8_t _txPin;
        bool _readOnly;
        bool _activeLevel;
        bool _passiveLevel;
        uint8_t _message[MAX_FRAME_LENGTH];
        uint8_t _messageLength;
        uint8_t _rxBuffer[MAX_FRAME_LENGTH];
        uint8_t _rxBufferPos;
        uint8_t _txBuffer[MAX_FRAME_LENGTH];
        uint8_t _txBufferPos;
        uint8_t _txLength;
        volatile bool _busIdle;
        volatile uint32_t _lastChange;
        volatile bool _lastState;
        volatile bool _currentRxBit;
        volatile bool _sofRead;
        volatile uint8_t _bitPos;
        volatile bool _IFRDetected;
        volatile bool _sofWrite;
        volatile bool _bitWrite;
        volatile uint8_t _bitNumWrite;
        volatile bool _currentState;
        volatile bool _currentTxBit;
        volatile bool _eofWrite;
        uint16_t _timerOCRValue_J1850VPW_TX_SOF;
        uint16_t _timerOCRValue_J1850VPW_TX_SRT;
        uint16_t _timerOCRValue_J1850VPW_TX_LNG;
        uint8_t _ignoreList[32];
        volatile onVPWMessageReceivedHandler _msgHandler;
        volatile onVPWErrorHandler _errHandler;
        void protocolEncoderInit();
        void protocolDecoderInit();
        void processMessage();
        void busIdleTimerInit();
        void busIdleTimerStart();
        void busIdleTimerStop();
        uint8_t* getBit(uint8_t _id, uint8_t* _pBit);
        uint8_t CRC(uint8_t* _buff, uint8_t _nbytes);
        void handleMessagesInternal(uint8_t* _message, uint8_t _messageLength);
        void handleErrorsInternal(J1850VPW_Operations _op, J1850VPW_Errors _err);
};

extern J1850VPWCore VPW;

#endif