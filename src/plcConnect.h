#ifndef PLC_CONNECT_H
#define PLC_CONNECT_H

#define LINUX
#include "openSocket.h"
#define LINUX
#include <nodave.h>

#include <string>

int plcConnect(std::string m_strIp);
int dataRead(int DBnum, int start, int len, void *buffer);
int dataWrite(int DBnum, int start, int len, void *buffer);

#endif