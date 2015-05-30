#Arduino XBee Library for GroveStreams Wireless Sensor Network
http://github.com/JChristensen/gsXBee  
ReadMe file  
Jack Christensen May 2015  

![CC BY-SA](http://mirrors.creativecommons.org/presskit/buttons/80x15/png/by-sa.png)

"Arduino XBee Library for GroveStreams Wireless Sensor Network" by Jack Christensen is licensed under CC BY-SA 4.0.

##Description
This library is derived from Andrew Rapp's XBee library to provide additional functionality for a GroveStreams wireless sensor network. See my [GroveStreams library](https://github.com/JChristensen/GroveStreams) for more information, example sketches and schematics for gateway and sensor nodes.

**Prerequisites**  
Andrew Rapp's XBee library  
http://github.com/andrewrapp/xbee-arduino

Mikal Hart's Streaming library  
http://arduiniana.org/libraries/streaming/

## Installation ##
To use the **gsXBee** library:  
- Go to https://github.com/JChristensen/gsXBee, click the **Download ZIP** button and save the ZIP file to a convenient location on your PC.
- Uncompress the downloaded file.  This will result in a folder containing all the files for the library, that has a name that includes the branch name, usually **gsXBee-master**.
- Rename the folder to just **gsXBee**.
- Copy the renamed folder to the Arduino **sketchbook\libraries** folder.

## Constructor ##

###gsXBee(void)
#####Description
Instantiates a gsXBee object.
#####Syntax
`gsXBee myXBee;`
#####Parameters
None.

## Methods ##
###begin(Stream &serial, bool forceDisassoc)
#####Description
Initializes the gsXBee library. Verifies serial communication with the XBee, gets its Node ID, ensures that it's associated, and optionally forces disassociation for end devices (this causes initialization to take several seconds longer but allows an end device to associate with an optimal parent, e.g. if it was moved). If communication with the XBee fails, the initialization routine waits one minute, then resets the microcontroller.
#####Syntax
`myXBee.begin(Serial, false);`

**serial:** Serial port to use for XBee communications *(Stream&)*.

**forceDisassoc:** Optional argument that defaults to *true*. For end devices only, causes the XBee to disassociate during the initialization sequence. Coordinators and routers are not affected *(bool)*.
#####Returns
None.
###read(void)
#####Description
Checks the XBee for incoming traffic and processes it.
#####Syntax
`myXBee.read();`
#####Parameters
None.
#####Returns
Type of traffic received, or various error indications. See the `xbeeReadStatus_t` enumeration in the [gsXBee.h file](https://github.com/JChristensen/gsXBee/blob/master/gsXBee.h) *(xbeeReadStatus_t)*.
#####Example
```c++
gsXBee myXBee;
xbeeReadStatus_t xbStat;
xbStat = myXBee.read();
```
###waitFor(xbeeReadStatus_t stat, uint32_t timeout)
#####Description
Reads the XBee until a certain status is returned, or a certain amount of time elapses, whichever occurs first.
#####Syntax
`myXBee.waitFor(stat, timeout);`
#####Parameters
**stat:** The status to wait for. See the `xbeeReadStatus_t` enumeration in the gsXBee.h file *(xbeeReadStatus_t)*.

**timeout:** Time to wait in milliseconds. *(unsigned long)*.
#####Returns
Type of traffic received, or various error indications. See the `xbeeReadStatus_t` enumeration in the [gsXBee.h file](https://github.com/JChristensen/gsXBee/blob/master/gsXBee.h) *(xbeeReadStatus_t)*.
#####Example
```c++
gsXBee myXBee;
xbeeReadStatus_t xbStat;
xbStat = myXBee.waitFor(TX_ACK, 100);	//wait 100 ms for transmission to be acknowledged
```
###sendCommand(uint8_t* cmd)
#####Description
Sends an AT command to the local XBee. The response is processed by `read()` or `waitFor()`.
#####Syntax
`myXBee.sendCommand(cmd);`
#####Parameters
**cmd:** A two-byte array containing the ASCII command characters. _(uint8_t*)_.
#####Returns
None.
#####Example
```c++
gsXBee myXBee;
uint8_t atCmd[] = { 'D', 'B' };
myXBee.sendCommand(atCmd);		//request rss for last packet received
```
###sendData(char* data)
#####Description
Sends data to a remote node.
#####Syntax
`myXBee.send(data);`
#####Parameters
**data:** Zero-terminated char array containing the data to be transmitted _(char*)_.
#####Returns
None.
#####Example
```c++
gsXBee myXBee;
char someData[] = "Hello, world!";
myXBee.sendData(someData);
```
###mcuReset(uint32_t dly)
#####Description
Resets the microcontroller after a given number of milliseconds. The minimum is 4 seconds (4000 ms). If a number less than 4000 is given, the delay will be approximately 4 seconds.
#####Syntax
`myXBee.mcuReset(dly);`
#####Parameters
**dly:** Delay in milliseconds before reset *(unsigned long)*.

#####Returns
None.
#####Example
```c++
gsXBee myXBee;
myXBee.resetMCU(60000);		//reset after a minute
```
