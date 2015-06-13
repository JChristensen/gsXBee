//To do: Add packet types: S-time sync request, T-current time (response to request)
//       Add a bool property to indicate whether to operate as a time server (default false),
//       don't respond to T-packets unless set to true.

// Arduino XBee Library for GroveStreams Wireless Sensor Network.
//
// This work by Jack Christensen is licensed under CC BY-SA 4.0,
// http://creativecommons.org/licenses/by-sa/4.0/
//
// A naming convention is used for the XBee Node Identifier (NI command) to
// identify the node to GroveStreams and also specify the timing and
// frequency of data transmission.
//
// The XBee NI must be set as follows. No validity checking is done;
// failure to follow the format exactly will result in undefined behaviour.
//
// Node Identifier Format: CompID_ssmmnnww
//
// Where CompID is a one- to eight-character name*,
//       _      is a single underscore character,
//       ss     gives the second to transmit on, 0 <= ss < 60,
//       mm     is the transmission interval in minutes, 1 <= mm < 100,
//       nn     is the number of minutes to offset transmissions, 0 <= nn < mm,
//       ww     is the number of seconds to wake a sleeping node before the
//              transmit time to read sensors, etc., 0 <= ww < 100.
//
// *This name is used as the GroveStreams Component ID.
// Each XBee on a given network (PAN ID) should have a unique CompID name.
//
// The parameters ss, nn and ww are intended for nodes whose transmission is precisely
// controlled by an accurate time source such as an RTC, NTP, GPS, etc.
//
// For example, nodeA_10010000 will transmit once a minute at ten seconds after the minute.
//              nodeB_10050000 will transmit once every five minutes at ten seconds after the minute,
//                             e.g. 00:00:10, 00:05:10, 00:10:10, etc.
//              nodeC_10050100 will transmit once every five minutes at ten seconds after the minute,
//                             but offset one minute from nodeB, e.g. 00:01:10, 00:06:10, 00:11:10, etc.

#ifndef _GSXBEE_H
#define _GSXBEE_H
#include <Arduino.h>
#include <avr/wdt.h>
#include <Streaming.h>              //http://arduiniana.org/libraries/streaming/
#include <XBee.h>                   //http://github.com/andrewrapp/xbee-arduino

const uint8_t PAYLOAD_LEN(80);
const char SOH = 0x01;                   //start of header
const char STX = 0x02;                   //start of text
enum xbeeReadStatus_t
{
    NO_TRAFFIC, READ_TIMEOUT, TX_ACK, TX_FAIL, COMMAND_RESPONSE, AI_CMD_RESPONSE, DA_CMD_RESPONSE,
    NI_CMD_RESPONSE, VR_CMD_RESPONSE, MODEM_STATUS, RX_NO_ACK, RX_DATA, RX_TIMESYNC, RX_ERROR,
    RX_UNKNOWN, UNKNOWN_FRAME
};

//a union between a 32-bit integer and a 4-byte array
union charInt_t {
    char c[4];
    uint32_t i;
};

class gsXBee : public XBee
{
public:
    gsXBee(void);
    bool begin(Stream &serial, bool forceDisassoc = true);
    xbeeReadStatus_t waitFor(xbeeReadStatus_t stat, uint32_t timeout);
    xbeeReadStatus_t read(void);
    void sendCommand(uint8_t* cmd);
    void sendData(char* data);
    void requestTimeSync(uint32_t utc);
    void sendTimeSync(uint32_t utc);
    void setSyncCallback( void (*fcn)(uint32_t) );  //set the time sync callback function
    void mcuReset(uint32_t dly = 0 );

    char compID[10];            //our component ID
    uint8_t txSec;              //transmit on this second, 0 <= txSec < 60
    uint8_t txInterval;         //transmission interval in minutes, 0 <= txInterval < 100
    uint8_t txOffset;           //minute offset to transmit, 0 <= txOffset < txInterval
    uint8_t txWarmup;           //seconds to wake before transmission time, to allow sensors to produce data, etc.
    uint8_t assocStatus;        //association status as returned in response from the AI command
    int8_t rss;                 //received signal strength, dBm
    bool disassocReset;         //flag to reset MCU when XBee disassociation occurs
    bool isTimeServer;          //if server, responds to requests for current time; else uses callback function to set time. 
    char packetType;            //D = data packet, S = time sync packet
    XBeeAddress64 sendingAddr;  //address of node that sent packet
    XBeeAddress64 destAddr;     //destination address
    uint16_t firmwareVersion;
    char atCmdRecd[4];          //the AT command responded to in an AT Command Response packet (two chars with zero terminator)
    char sendingCompID[10];     //sender's component ID from received packet
    char payload[PAYLOAD_LEN];  //XBee payload

private:
    bool parsePacket(void);
    void buildDataPayload(void);
    void getRSS(void);
    void parseNodeID(char* nodeID);
    void copyToBuffer(char* dest, uint32_t source);
    uint32_t getFromBuffer(char* source);

    uint32_t msTX;                          //last XBee transmission time from millis()
    void (*timeSyncCallback)(uint32_t);     //user function called to set the time when a sync packet is received (if not a time server)
    char tsCompID[10];                      //time sync requestor's component ID
    ZBTxStatusResponse zbStat;
    AtCommandResponse atResp;
    ModemStatusResponse zbMSR;
    ZBRxResponse zbRX;
    ZBTxRequest zbTX;
};

#endif
