#include"plcConnect.h"

static _daveOSserialType _fds;
static daveInterface *_di;
static daveConnection *_dc;

static bool plcConnectStatus = false;
static int m_nPort = 120;
static int m_nRack = 1;
static int m_nSlot = 0;

int plcConnect(std::string m_strIp)
{
    _fds.rfd = openSocket(120, m_strIp.c_str());
    _fds.wfd = _fds.rfd;
    int useProtocol = daveProtoISOTCP;

    printf("connect……\n");
    if (_fds.rfd > 0)
    {
        _di = daveNewInterface(_fds, "IF1", 0, useProtocol, daveSpeed187k);
        daveSetTimeout(_di, 5000000);
        daveInitAdapter(_di);
        int Mpi = 2;
        _dc = daveNewConnection(_di, Mpi, m_nRack, m_nSlot); // insert your rack and slot here
        char szconnlog[256] = {0};
        snprintf(szconnlog, 255, "daveNewConnection:  Mpi=%d, nrack=%d, slot=%d\n",  Mpi, m_nRack, m_nSlot);
        if (0 == daveConnectPLC(_dc))
        {
            printf("plc connect success, ip: %s, port: %d", m_strIp.c_str(), m_nPort);
            char szlog[1024] = {0};
            snprintf(szlog, sizeof(szlog) - 1, "connect plc success，ip: %s, port: %d\n", m_strIp.c_str(), m_nPort);
            plcConnectStatus = false;
            return 0;
        }
    }
    else
    {
        printf("plc connect failed, ip: %s, port: %d\n", m_strIp.c_str(), m_nPort);
    }
    plcConnectStatus = false;
    return -1;
}

int dataRead(int DBnum, int start, int len, void *buffer)
{
    int res = daveReadManyBytes(_dc, daveDB, DBnum, start, len, buffer);
    return res;
}

int dataWrite(int DBnum, int start, int len, void *buffer)
{
    int res = daveReadManyBytes(_dc, daveDB, DBnum, start, len, buffer);
    return res;
}
