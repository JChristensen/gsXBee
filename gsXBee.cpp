// Arduino XBee Library for GroveStreams Wireless Sensor Network.
//
// This work by Jack Christensen is licensed under CC BY-SA 4.0,
// http://creativecommons.org/licenses/by-sa/4.0/

#include <gsXBee.h>

//constructor. coordinator is default destination.
gsXBee::gsXBee(void) : destAddr(0x0, 0x0), timeSyncCallback(NULL)
{
    tsCompID[0] = 0;
}

//optionally reset the XBee. check to see if it's associated, if not, wait.
//after it's associated, get its Node ID and version information.
bool gsXBee::begin(Stream &serial, bool resetXBee)
{
    XBee::begin(serial);

    //initialization state machine establishes communication with the XBee and
    //ensures that it is associated.
    const uint32_t ASSOC_TIMEOUT(60000);                    //milliseconds to wait for XBee to associate
    enum BEGIN_STATES_t                                     //state machine states
    {
        RESET_XBEE, REQ_AI, WAIT_ASSOC, GET_NI, GET_VR, INIT_COMPLETE, INIT_FAIL
    };
    BEGIN_STATES_t BEGIN_STATE = RESET_XBEE;

    while ( 1 )
    {
        uint32_t stateTimer;

        switch (BEGIN_STATE)
        {
        case RESET_XBEE:
            delay(1000);                                    //allow some time for the XBee POR
            while ( read() != NO_TRAFFIC );                 //handle any incoming traffic
            if (resetXBee)
            {
                uint8_t cmd[] = "FR";                       //firmware reset
                sendCommand(cmd);
                if ( waitFor(FR_CMD_RESPONSE, 1000) == READ_TIMEOUT )
                {
                    BEGIN_STATE = INIT_FAIL;
                    Serial << millis() << F("\tThe XBee did not respond\n");
                }
                else if ( waitFor(MODEM_STATUS, 3000) == READ_TIMEOUT )     //FR takes 2+ seconds, responds with modem status (WDT reset)
                {
                    BEGIN_STATE = INIT_FAIL;
                    Serial << millis() << F("\tThe XBee did not respond\n");
                }
                else
                {
                    BEGIN_STATE = REQ_AI;
                }
            }
            else
            {
                BEGIN_STATE = REQ_AI;
            }
            break;
            
        case REQ_AI:                                        //get association indicator
            {
                uint8_t cmd[] = "AI";
                sendCommand(cmd);
                if ( waitFor(AI_CMD_RESPONSE, 1000) == READ_TIMEOUT )
                {
                    BEGIN_STATE = INIT_FAIL;
                    Serial << millis() << F("\tThe XBee did not respond\n");
                }
                BEGIN_STATE = WAIT_ASSOC;
                stateTimer = millis();
            }
            break;
            
        case WAIT_ASSOC:                                    //wait for the XBee to associate
            read();
            if ( assocStatus == 0 ) {                       //zero means associated
                BEGIN_STATE = GET_NI;
                disassocReset = true;                       //any further disassociations are unexpected
            }
            else if (millis() - stateTimer >= ASSOC_TIMEOUT) {
                BEGIN_STATE = INIT_FAIL;
                Serial << millis() << F("\tXBee associate fail\n");
            }
            break;
            
        case GET_NI:
            {
                uint8_t cmd[] = "NI";                       //ask for the node ID
                sendCommand(cmd);
                if ( waitFor(NI_CMD_RESPONSE, 1000) == READ_TIMEOUT )
                {
                    BEGIN_STATE = INIT_FAIL;
                    Serial << millis() << F("\tThe XBee did not respond\n");
                }
                else
                {
                    BEGIN_STATE = GET_VR;
                }
            }
            break;

        case GET_VR:
            {
                uint8_t cmd[] = "VR";                       //ask for firmware version
                sendCommand(cmd);
            }
            if ( waitFor(VR_CMD_RESPONSE, 1000) == READ_TIMEOUT )
            {
                BEGIN_STATE = INIT_FAIL;
                Serial << millis() << F("\tXBee VR fail\n");
            }
            else
            {
                BEGIN_STATE = INIT_COMPLETE;
            }
            break;

        case INIT_COMPLETE:
            return true;
            break;

        case INIT_FAIL:
            return false;
            break;
        }
    }
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

        uint8_t apiID = getResponse().getApiId();
        switch (apiID)                                      //what kind of packet did we get?
        {
        case ZB_TX_STATUS_RESPONSE:                         //transmit status for packets we've sent
            {
                ZBTxStatusResponse zbStat;
                getResponse().getZBTxStatusResponse(zbStat);
                uint8_t delyStatus = zbStat.getDeliveryStatus();
                uint8_t dscyStatus = zbStat.getDiscoveryStatus();
                uint8_t txRetryCount = zbStat.getTxRetryCount();
                switch (delyStatus)
                {
                case SUCCESS:
                    Serial << ms << F("\tXB TX OK\t") << ms - msTX << F("ms R=");
                    Serial << txRetryCount << F(" DSCY=") << dscyStatus << endl;
                    return TX_ACK;
                    break;
                default:
                    Serial << ms << F("\tXB TX FAIL\t") << ms - msTX << F("ms R=");
                    Serial << txRetryCount << F(" DELY=") << delyStatus << F(" DSCY=") << dscyStatus << endl;
                    return TX_FAIL;
                    break;
                }
            }
            break;

        case AT_COMMAND_RESPONSE:                           //response to an AT command
            {
                AtCommandResponse atResp;
                getResponse().getAtCommandResponse(atResp);
                if (atResp.isOk())
                {
                    uint8_t* p = atResp.getCommand();       //get the command
                    atCmdRecd[0] = *p++;
                    atCmdRecd[1] = *p++;
                    atCmdRecd[2] = 0;
                    uint16_t cmd = ( atCmdRecd[0] << 8 ) + atCmdRecd[1];    //quick way to compare two characters
                    uint8_t respLen = atResp.getValueLength();
                    p = atResp.getValue();
                    switch ( cmd )
                    {
                    case 0x4149:                            //AI command (association indication)
                        assocStatus = *p;
                        return AI_CMD_RESPONSE;
                        break;
                    case 0x4441:                            //DA command (force disassociation)
                        return DA_CMD_RESPONSE;
                        break;
                    case 0x4652:                            //FR command (firmware reset)
                        return FR_CMD_RESPONSE;
                        break;
                    case 0x4E49:                            //NI command (node identifier)
                        {
                            char nodeID[20];
                            char *n = nodeID;
                            for (uint8_t i=0; i<respLen; ++i) {
                                *n++ = *p++;
                            }
                            *n++ = 0;                       //string terminator
                            parseNodeID(nodeID);
                            return NI_CMD_RESPONSE;
                            break;
                        }
                    case 0x5652:                            //VR command (firmware version)
                        firmwareVersion = ( *p << 8 ) + *(p + 1);
                        return VR_CMD_RESPONSE;
                        break;
                    default:
                        Serial << ms << F("\tUNK CMD RESP\t") << atCmdRecd << endl;
                        return COMMAND_RESPONSE;
                        break;
                    }
                }
                else
                {
                    Serial << ms << F("\tAT CMD FAIL\n");
                    return COMMAND_RESPONSE;
                }
            }
            break;

        case MODEM_STATUS_RESPONSE:                         //XBee administrative messages
            {
                ModemStatusResponse zbMSR;
                getResponse().getModemStatusResponse(zbMSR);
                uint8_t msrResponse = zbMSR.getStatus();
                Serial << ms << '\t';
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
                    if (disassocReset) mcuReset();          //restart and hope to reassociate
                    break;
                default:
                    Serial << F("XB MDM STAT\t0x") << _HEX(msrResponse) << endl;
                    break;
                }
            }
            return MODEM_STATUS;
            break;

        case ZB_RX_RESPONSE:                                //rx data packet
            getResponse().getZBRxResponse(zbRX);            //get the received data
            switch (zbRX.getOption() & 0x01)                //check ack bit only
            {
            case ZB_PACKET_ACKNOWLEDGED:
                //process the received data
                Serial << ms << F("\tXB RX/ACK\n");
                if ( parsePacket() )
                {
                    switch (packetType)                     //what type of packet
                    {
                    case 'D':                               //data headed for the web
                        getRSS();                           //get the received signal strength
                        return RX_DATA;
                        break;

                    case 'S':                               //time sync packet
                        if ( isTimeServer )                 //queue the request
                        {
                            if (tsCompID[0] == 0)           //can only queue one request, ignore if one is already queued
                            {
                                strcpy(tsCompID, sendingCompID);    //save the sender's node ID
                            }
                        }
                        else if (timeSyncCallback != NULL)  //call the user's time sync function if they gave one
                        {
                            uint32_t utc = getFromBuffer(payload);
                            timeSyncCallback(utc);
                        }
                        return RX_TIMESYNC;
                        break;

                    default:                                //not expecting anything else
                        Serial << endl << ms << F("\tXB unknown packet type\n");
                        return RX_UNKNOWN;
                        break;
                    }
                }
                else
                {
                    uint8_t *d = zbRX.getData();
                    uint8_t nChar = zbRX.getDataLength();
                    Serial << ms << F("\tMalformed packet:\t/");
                    for ( uint8_t i = 0; i < nChar; ++i ) Serial << (char)*d++;
                    Serial << '/' << nChar << endl;
                    return RX_ERROR;
                }
                break;

            default:
                Serial << ms << F("\tXB RX no ACK\n");              //packet received and not ACKed
                return RX_NO_ACK;
                break;
            }
            break;

        default:                                            //something else we were not expecting
            Serial << ms << F("\tXB UNEXP TYPE 0x") << _HEX(apiID) << endl; //unexpected frame type
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
    Serial << endl << millis() << F("\tXB CMD\t") << (char*)cmd << endl;
}

//Build & send an XBee data packet containing a character string destined for GroveStreams.
//The data packet is defined as follows:
//Byte  0:       SOH character (Start of header, 0x01)
//Byte  1:       Packet type, D=data
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
    char *p = payload;
    *p++ = SOH;
    *p++ = 'D';                                             //data packet
    char *c = compID;
    while ( (*p++ = *c++) );                                //copy in component ID
    *(p - 1) = STX;                                         //overlay the string terminator
    strcpy(p, data);                                        //copy in the data
    uint8_t len = strlen(payload);
    zbTX.setAddress64(destAddr);                            //build the tx request packet
    zbTX.setAddress16(0xFFFE);
    zbTX.setPayload( (uint8_t*)payload );
    zbTX.setPayloadLength(len);
    send(zbTX);
    msTX = millis();
    Serial << endl << msTX << F("\tXB TX\t") << len << endl;
}

//Build & send an XBee data packet containing binary data, typically to another node.
void gsXBee::sendData(char packetType, uint8_t* data, uint8_t dataLen)
{
    char *p = payload;
    *p++ = SOH;
    *p++ = packetType;
    char *c = compID;
    while ( (*p++ = *c++) );                                //copy in component ID
    *(p - 1) = STX;                                         //overlay the string terminator
    for ( uint8_t i = 0; i < dataLen; ++i )                     //copy in the data
    {
        *p++ = *data++;
    }
    uint8_t len = dataLen + strlen(compID) + 3;
    zbTX.setAddress64(destAddr);                            //build the tx request packet
    zbTX.setAddress16(0xFFFE);
    zbTX.setPayload( (uint8_t*)payload );
    zbTX.setPayloadLength(len);
    send(zbTX);
    msTX = millis();
    Serial << endl << msTX << F("\tXB TX\t") << len << endl;
}

//parse a received packet; check format, extract GroveStreams component ID and data.
//returns false if there is an error in the format, else true.
bool gsXBee::parsePacket(void)
{
    uint8_t *d = zbRX.getData();
    uint8_t len = zbRX.getDataLength();
    if ( *d++ != SOH ) return false;                        //check for SOH start character
    packetType = *d++;                                      //save the packet type
    char *c = sendingCompID;                                //now parse the component ID
    uint8_t nChar = 0;
    char ch;
    while ( (ch = *d++) != STX )                            //look for STX
    {
        if ( ++nChar > 8 ) return false;                    //missing
        *c++ = ch;
    }
    *c++ = 0;                                               //string terminator
    char *p = payload;                                      //now copy the rest of the payload data
    for (uint8_t i = nChar + 3; i < len; ++i )              //SOH + STX + packet type = 3 chars
    {
        *p++ = *d++;
    }
    *p++ = 0;                                               //string terminator
    sendingAddr = zbRX.getRemoteAddress64();                //save the sender's address
    Serial << millis() << F("\tXB RX\t") << sendingCompID << ' ' << len << F("b\n");
    return true;
}

//returns received signal strength value for the last RF data packet.
void gsXBee::getRSS()
{
    uint8_t atCmd[] = { 'D', 'B' };
    AtCommandRequest atCmdReq = AtCommandRequest(atCmd);
    send(atCmdReq);
    uint32_t ms = millis();
    if (readPacket(20)) {
        Serial << endl;
        if (getResponse().getApiId() == AT_COMMAND_RESPONSE) {
            AtCommandResponse atResp;
            getResponse().getAtCommandResponse(atResp);
            if (atResp.isOk()) {
                uint8_t respLen = atResp.getValueLength();
                if (respLen == 1) {
                    uint8_t* resp = atResp.getValue();
                    rss = -resp[0];
                }
                else {
                    Serial << ms << F("\tRSS LEN ERR\n");   //unexpected length
                }
            }
            else {
                Serial << ms << F("\tRSS ERR\n");           //status not ok
            }
        }
        else {
            Serial << ms << F("\tRSS UNEXP RESP\n");        //expecting AT_COMMAND_RESPONSE, got something else
        }
    }
    else {
        Serial << millis() << F("\tRSS NO RESP\n");         //timed out
    }
}

//parse Node ID in format compID_ssmmnnww.
//compID must be 1-8 characters, the remainder must be exactly "_" followed by 8 digits.
//no checking for proper format is done; improper format will cause undefined behavior.
void gsXBee::parseNodeID(char* ni)
{
    char* p = ni;                                           //copy the pointer to preserve ni pointing at the start of the string
    p += strlen(p) - 2;                                     //point at ww
    txWarmup = atoi(p);
    *p = 0;
    p -= 2;                                                 //put in terminator and back up to point at nn
    txOffset = atoi(p);
    *p = 0;
    p -= 2;                                                 //mm
    txInterval = atoi(p);
    *p = 0;
    p -= 2;                                                 //ss
    txSec = atoi(p);
    *(--p) = 0;                                             //terminator after component ID
    strcpy(compID, ni);                                     //save the component ID
}

//ask for the current time, utc is the current time of the requestor (not currently used)
void gsXBee::requestTimeSync(uint32_t utc)
{
    char *p = payload;
    *p++ = SOH;
    *p++ = 'S';                                             //time sync packet
    char *c = compID;
    while ( (*p++ = *c++) );                                //copy in component ID
    *(p - 1) = STX;                                         //overlay the string terminator
    copyToBuffer(p, utc);                                   //send our current time

    uint8_t len = strlen(compID) + 7;                       //build the tx request
    zbTX.setAddress64(destAddr);
    zbTX.setAddress16(0xFFFE);
    zbTX.setPayload((uint8_t*)payload);
    zbTX.setPayloadLength(len);
    send(zbTX);
    msTX = millis();
    Serial << endl << msTX << F("\tTime sync\t") << len << endl;
}

//respond to a previously queued time sync request
//utc can be a time_t value (same as uint32_t).
//should be called immediately after second rollover
void gsXBee::sendTimeSync(uint32_t utc)
{
    if (tsCompID[0] != 0) {                                 //is there a request queued?
        char *p = payload;
        *p++ = SOH;
        *p++ = 'S';                                         //time sync packet
        char *c = compID;
        while ( (*p++ = *c++) );                            //copy in component ID
        *(p - 1) = STX;                                     //overlay the string terminator
        copyToBuffer(p, utc);                               //send current UTC

        uint8_t len = strlen(compID) + 7;                   //build the tx request
        zbTX.setAddress64(sendingAddr);
        zbTX.setAddress16(0xFFFE);
        zbTX.setPayload((uint8_t*)payload);
        zbTX.setPayloadLength(len);
        send(zbTX);
        msTX = millis();
        Serial << endl << millis() << F("\tTime sync\t") << tsCompID << ' ' << len << endl;
        tsCompID[0] = 0;                                    //request was serviced, none queued
    }
}

void gsXBee::setSyncCallback( void (*fcn)(uint32_t t) )
{
    timeSyncCallback = fcn;
}

//reset the mcu
void gsXBee::mcuReset(uint32_t dly)
{
    if ( dly > 4000 ) delay(dly - 4000);
    Serial << millis() << F("\tReset in");
    wdt_enable(WDTO_4S);
    int countdown = 4;
    while (1)
    {
        Serial << ' ' << countdown--;
        delay(1000);
    }
}

//copy a four-byte integer to the designated offset in the buffer
void gsXBee::copyToBuffer(char* dest, uint32_t source)
{
    charInt_t data;

    data.i = source;
    dest[0] = data.c[0];
    dest[1] = data.c[1];
    dest[2] = data.c[2];
    dest[3] = data.c[3];
}

//get a four-byte integer from the buffer starting at the designated offset
uint32_t gsXBee::getFromBuffer(char* source)
{
    charInt_t data;

    data.c[0] = source[0];
    data.c[1] = source[1];
    data.c[2] = source[2];
    data.c[3] = source[3];

    return data.i;
}
