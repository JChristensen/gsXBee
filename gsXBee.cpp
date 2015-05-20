// Arduino XBee Library for GroveStreams Wireless Sensor Network.
//
// This work by Jack Christensen is licensed under CC BY-SA 4.0,
// http://creativecommons.org/licenses/by-sa/4.0/

#include <gsXBee.h>

gsXBee::gsXBee(void) : destAddr(0x0, 0x0)   //coordinator is default destination
{
}

//read the XBee until a certain type of message is received, or until a certain amount of time has passed.
xbeeReadStatus_t gsXBee::waitFor(xbeeReadStatus_t stat, uint32_t timeout)
{
    uint32_t msStart = millis();
    while ( millis() - msStart < timeout )
    {
        xbeeReadStatus_t s = read();
        if ( s == stat ) return s;
    }
    return READ_TIMEOUT;
}

//check the XBee for incoming traffic and process it
xbeeReadStatus_t gsXBee::read(void)
{
    readPacket();
    if ( getResponse().isAvailable() )
    {
        uint32_t ms = millis();

        switch (getResponse().getApiId())             //what kind of packet did we get?
        {
        case ZB_TX_STATUS_RESPONSE:                   //transmit status for packets we've sent
            {
                ZBTxStatusResponse zbStat;
                getResponse().getZBTxStatusResponse(zbStat);
                uint8_t delyStatus = zbStat.getDeliveryStatus();
                uint8_t dscyStatus = zbStat.getDiscoveryStatus();
                uint8_t txRetryCount = zbStat.getTxRetryCount();
                switch (delyStatus)
                {
                case SUCCESS:
                    Serial << ms << F(" XB TX OK ") << ms - msTX << F("ms R=");
                    Serial << txRetryCount << F(" DSCY=") << dscyStatus << endl;
                    return TX_ACK;
                    break;
                default:
                    Serial << ms << F(" XB TX FAIL ") << ms - msTX << F("ms R=");
                    Serial << txRetryCount << F(" DELY=") << delyStatus << F(" DSCY=") << dscyStatus << endl;
                    return TX_FAIL;
                    break;
                }
            }
            break;

        case AT_COMMAND_RESPONSE:                          //response to an AT command
            {
                AtCommandResponse atResp;
                getResponse().getAtCommandResponse(atResp);
                if (atResp.isOk())
                {
                    uint8_t* p = atResp.getCommand();          //get the command
                    atCmdRecd[0] = *p++;
                    atCmdRecd[1] = *p++;
                    atCmdRecd[2] = 0;
                    uint16_t cmd = ( atCmdRecd[0] << 8 ) + atCmdRecd[1];    //quick way to compare two characters
                    uint8_t respLen = atResp.getValueLength();
                    p = atResp.getValue();
                    switch ( cmd )
                    {
                    case 0x4149:                        //AI command
                        assocStatus = *p;
                        return AI_CMD_RESPONSE;
                        break;
                    case 0x4441:                        //DA command
                        return DA_CMD_RESPONSE;
                        break;
                    case 0x4E49:                        //NI command
                        {
                            char nodeID[20];
                            char *n = nodeID;
                            for (uint8_t i=0; i<respLen; ++i) {
                                *n++ = *p++;
                            }
                            *n++ = 0;                   //string terminator
                            Serial << ms << F(" XB NI=") << nodeID << endl;
                            parseNodeID(nodeID);
                            return NI_CMD_RESPONSE;
                            break;
                        }
                    default:
                        Serial << ms << F(" UNK CMD RESP ") << atCmdRecd << endl;
                        return COMMAND_RESPONSE;
                        break;
                    }
                }
                else
                {
                    Serial << ms << F(" AT CMD FAIL\n");
                    return COMMAND_RESPONSE;
                }
            }
            break;

        case MODEM_STATUS_RESPONSE:                   //XBee administrative messages
            {
                ModemStatusResponse zbMSR;
                getResponse().getModemStatusResponse(zbMSR);
                uint8_t msrResponse = zbMSR.getStatus();
                Serial << ms << ' ';
                switch (msrResponse)
                {
                case HARDWARE_RESET:
                    Serial << F("XB HW RST\n");
                    break;
                case ASSOCIATED:
                    Serial << F("XB ASC\n");
                    assocStatus = 0x00;
                    break;
                case DISASSOCIATED:
                    Serial << F("XB DISASC\n");
                    assocStatus = 0xFF;
                    if (disassocReset) mcuReset();         //restart and hope to reassociate
                    break;
                default:
                    Serial << F("XB MDM STAT 0x") << _HEX(msrResponse) << endl;
                    break;
                }
            }
            return MODEM_STATUS;
            break;

        case ZB_RX_RESPONSE:                               //rx data packet
            getResponse().getZBRxResponse(zbRX);           //get the received data
            switch (zbRX.getOption() & 0x01)               //check ack bit only
            {
            case ZB_PACKET_ACKNOWLEDGED:
                //process the received data
                Serial << ms << F(" XB RX/ACK\n");
                if ( parsePacket() )
                {
                    switch (packetType)                    //what type of packet
                    {
                    case 'D':                              //data headed for the web
                        getRSS();                          //get the received signal strength
                        return RX_DATA;
                        break;
                    default:                               //not expecting anything else
                        Serial << endl << ms << F(" XB unknown packet type\n");
                        return RX_UNKNOWN;
                        break;
                    }
                }
                else
                {
                    uint8_t *d = zbRX.getData();
                    uint8_t nChar = zbRX.getDataLength();
                    Serial << ms << F(" Malformed packet: /");
                    for ( uint8_t i = 0; i < nChar; ++i ) Serial << (char)*d++;
                    Serial << '/' << nChar << endl;
                    return RX_ERROR;
                }
                break;

            default:
                Serial << F("XB RX no ACK\n");             //packet received and not ACKed
                return RX_NO_ACK;
                break;
            }
            break;

        default:                                           //something else we were not expecting
            Serial << F("XB UNEXP TYPE\n");                //unexpected frame type
            return UNKNOWN_FRAME;
            break;
        }
    }
    else
    {
        return NO_TRAFFIC;
    }
}

//send an AT command to the XBee.
//response is processed in read().
void gsXBee::sendCommand(uint8_t* cmd)
{
    AtCommandRequest atCmdReq = AtCommandRequest(cmd);
    send(atCmdReq);
    Serial << endl << millis() << F(" XB CMD ") << (char*)cmd << endl;
}

//Build & send an XBee data packet.
//The data packet is defined as follows:
//Byte  0:       SOH character (Start of header, 0x01)
//Byte  1:       Packet type, D=data, S=time sync request
//Bytes 2-m:     (m <= 9) GroveStreams component ID, 1-8 characters.
//Byte  m+1:     STX character (0x02), delimiter between header and data.
//Bytes m+2-n:   (D packet) Data to be sent to GroveStreams, in
//               GroveStreams' PUT feed API/URL format, terminated by a
//               zero byte. The sending unit must format the data, e.g.:
//               &streamID1=value1&streamID2=value2...&streamIDn=valuen
//
//The maximum XBee packet size is set by PAYLOAD_LEN at the top of this
//file. Note there is an upper limit, see the XBee ATNP command.
void gsXBee::sendData(char* data)
{
    const char SOH = 0x01;                   //start of header
    const char STX = 0x02;                   //start of text

    char *p = payload;
    *p++ = SOH;
    *p++ = 'D';                              //data packet
    char *c = compID;
    while ( (*p++ = *c++) );                 //copy in component ID
    *(p - 1) = STX;                          //overlay the string terminator
    strcpy(p, data);                         //copy in the data
    uint8_t len = strlen(payload);
    zbTX.setAddress64(destAddr);             //build the tx request packet
    zbTX.setAddress16(0xFFFE);
    zbTX.setPayload( (uint8_t*)payload );
    zbTX.setPayloadLength(len);
    send(zbTX);
    msTX = millis();
    Serial << endl << msTX << F(" XB TX ") << len << endl;
}

//parse a received packet; check format, extract GroveStreams component ID and data.
//returns false if there is an error in the format, else true.
bool gsXBee::parsePacket(void)
{
    uint8_t *d = zbRX.getData();
    uint8_t len = zbRX.getDataLength();
    if ( *d++ != 0x01 ) return false;          //check for SOH start character
    packetType = *d++;                         //save the packet type
    char *c = sendingCompID;                   //now parse the component ID
    uint8_t nChar = 0;
    char ch;
    while ( (ch = *d++) != 0x02 )              //look for STX
    {
        if ( ++nChar > 8 ) return false;       //missing
        *c++ = ch;
    }
    *c++ = 0;                                  //string terminator
    char *p = payload;                         //now copy the rest of the payload data
    for (uint8_t i = nChar+2; i < len; ++i )
    {
        *p++ = *d++;
    }
    *p++ = 0;                                  //string terminator
    sendingAddr = zbRX.getRemoteAddress64();   //save the sender's address
    Serial << millis() << F(" XB RX ") << sendingCompID << ' ' << len << endl;
    return true;
}

//returns received signal strength value for the last RF data packet.
void gsXBee::getRSS(void)
{
    uint8_t atCmd[] = { 'D', 'B' };
    AtCommandRequest atCmdReq = AtCommandRequest(atCmd);
    send(atCmdReq);
    if (readPacket(10))
    {
        if (getResponse().getApiId() == AT_COMMAND_RESPONSE)
        {
            AtCommandResponse atResp;
            getResponse().getAtCommandResponse(atResp);
            if (atResp.isOk())
            {
                uint8_t respLen = atResp.getValueLength();
                if (respLen == 1)
                {
                    uint8_t* resp = atResp.getValue();
                    rss = -resp[0];
                }
                else
                {
                    Serial << F("RSS LEN ERR\n");    //unexpected length
                }
            }
            else
            {
                Serial << F("RSS ERR\n");            //status not ok
            }
        }
        else
        {
            Serial << F("RSS UNEXP RESP\n");         //expecting AT_COMMAND_RESPONSE, got something else
        }
    }
    else
    {
        Serial << F("RSS NO RESP\n");                //timed out
    }
}

//parse Node ID in format compID_ssmmnnww.
//compID must be 1-8 characters, the remainder must be exactly "_" followed by 8 digits.
//no checking for proper format is done; improper format will cause undefined behavior.
void gsXBee::parseNodeID(char* ni)
{
    char* p = ni;              //copy the pointer to preserve ni pointing at the start of the string
    p += strlen(p) - 2;        //point at ww
    txWarmup = atoi(p);
    *p = 0;
    p -= 2;                    //put in terminator and back up to point at nn
    txOffset = atoi(p);
    *p = 0;
    p -= 2;                    //mm
    txInterval = atoi(p);
    *p = 0;
    p -= 2;                    //ss
    txSec = atoi(p);
    *(--p) = 0;                //terminator after component ID
    strcpy(compID, ni);        //save the component ID
}

//reset the mcu
void GroveStreams::mcuReset(uint32_t dly)
{
    if ( dly > 4000 ) delay(dly - 4000);
    Serial << millis() << F(" Reset in");
    wdt_enable(WDTO_4S);
    int countdown = 4;
    while (1)
    {
        Serial << ' ' << countdown--;
        delay(1000);
    }
}

