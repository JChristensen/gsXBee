// XBee class for GroveStreams wireless sensor network.
//
// "GroveStreams Sensor Node"
// by Jack Christensen is licensed under CC BY-SA 4.0,
// http://creativecommons.org/licenses/by-sa/4.0/
//
//CompID_ssmmnnww where ss=second to transmit, mm=transmit interval in minutes, nn=transmit offset in minutes, ww=warmup time in seconds

#ifndef _GSXBEE_H
#define _GSXBEE_H
#include <Arduino.h>
#include <avr/wdt.h>
#include <Streaming.h>              //http://arduiniana.org/libraries/streaming/
#include <XBee.h>                   //http://code.google.com/p/xbee-arduino/

const uint8_t PAYLOAD_LEN(80);
enum xbeeReadStatus_t
{
    NO_TRAFFIC, READ_TIMEOUT, TX_ACK, TX_FAIL, COMMAND_RESPONSE, AI_CMD_RESPONSE, DA_CMD_RESPONSE,
    NI_CMD_RESPONSE, MODEM_STATUS, RX_NO_ACK, RX_DATA, RX_ERROR, RX_UNKNOWN, UNKNOWN_FRAME
};

class gsXBee : public XBee
{
public:
    gsXBee(void);
    xbeeReadStatus_t waitFor(xbeeReadStatus_t stat, uint32_t timeout);
    xbeeReadStatus_t read(void);
    void sendCommand(uint8_t* cmd);
    void sendData(char* data);
    void mcuReset(uint32_t dly);

    char compID[10];          //our component ID
    uint8_t txSec;            //transmit on this second, 0 <= txSec < 60
    uint8_t txInterval;       //transmission interval in minutes, 0 <= txInterval < 100
    uint8_t txOffset;         //minute offset to transmit, 0 <= txOffset < txInterval
    uint8_t txWarmup;         //seconds to wake before transmission time, to allow sensors to produce data, etc.
    uint8_t assocStatus;
    int8_t rss;               //received signal strength, dBm
    bool disassocReset;       //flag to reset MCU when XBee disassociation occurs
    char atCmdRecd[4];        //the AT command responded to in an AT Command Response packet (two chars with zero terminator)
    char sendingCompID[10];         //sender's component ID from received packet
    char payload[PAYLOAD_LEN];

private:
    bool parsePacket(void);
    void buildDataPayload(void);
    void getRSS(void);
    void parseNodeID(char* nodeID);

    uint32_t msTX;                       //last XBee transmission time from millis()
    char packetType;                     //D = data packet
    XBeeAddress64 sendingAddr;           //address of node that sent packet
    XBeeAddress64 destAddr;              //destination address
    ZBTxStatusResponse zbStat;
    AtCommandResponse atResp;
    ModemStatusResponse zbMSR;
    ZBRxResponse zbRX;
    ZBTxRequest zbTX;
};

#endif
