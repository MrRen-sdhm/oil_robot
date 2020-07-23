//
// Created by sdhm on 7/5/19.
//

#ifndef MODBUSADAPTER_H
#define MODBUSADAPTER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <algorithm>

#include "modbus.h"

//#define LIB_MODBUS_DEBUG_OUTPUT
#define DEBUG 0

using namespace std;

class ModbusAdapter
{
public:
    explicit ModbusAdapter();
    ~ModbusAdapter();

    void modbusConnectRTU(string port, int baud, char parity, int dataBits, int stopBits, int RTS, int timeOut=1);
    int modbusConnectTCP(const string ip, int port, int timeOut=1); // ip example: "127.0.0.1"
    void modbusDisConnect();

    void modbusReadData(int slave, int functionCode, int startAddress, int noOfItems);
    int modbusReadHoldReg(int slave, int startAddress, int noOfItems, uint16_t* data);

    int modbusWriteData(int slave, int functionCode, int startAddress, int noOfItems, const uint16_t *value);

    bool isConnected();

    void setSlave(int slave);
    void setFunctionCode(int functionCode);
    void setStartAddr(int addr);
    void setNumOfRegs(int num);
    void addItems();

    void setScanRate(int scanRate);
    void setTimeOut(int timeOut);
    int packets();
    int errors();

    modbus_t * m_modbus;

    uint8_t *dest;
    uint16_t *dest16;

private:
    string libmodbus_strerror(int errnum);

    bool m_connected;
    int m_slave;
    int m_functionCode;
    int m_startAddr;
    int m_numOfRegs;
    int m_scanRate;
    int m_packets;
    int m_errors;

};

#endif // MODBUSADAPTER_H
