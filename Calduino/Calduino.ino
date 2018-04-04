/***************************************************
This sketch decodes/encodes messages through EMS BUS (Buderus / Nefit Boilers)
and sends them in XML format using a WiFly module (RN-XV  RN-171)

last edit	:  07 MAY 2017

01 APR 2018 : Included operations to configure programs and set switch points
29 MAR 2018	: Short corrections
07 MAY 2017	: GitHub Version 1.0
01 MAY 2017 : Included tags to skip operations when an error is detected
15 APR 2017 : Include timestamp in every XML file when the EMS operation returns correctly 
14 APR 2017 : Added a timeout in EMS reader function to force the end of the function when nothing is received
1  APR 2017 : Redefined WiFly operations

author		:	dani.macias.perea@gmail.com

Get RC Time				- calduino/?op=09
Get UBA Monitor			- calduino/?op=01
Get UBA WW Monitor		- calduino/?op=02
Get RC35_HC1			- calduino/?op=03
Get RC35_HC2			- calduino/?op=04
Get General Monitor		- calduino/?op=05
Get Program				- calduino/?op=06?pr=X the packet id of the user program to be read (0 - 0x38 - DHW, 1 - 0x39 - DHW Pump, 2 - 0x3F - HK1User1, 3 - 0x42 - HK1User2, 4 - 0x49 - HK2User1, 5 - 0x4C - HK2User2)
Get All Monitors	    - calduino/?op=08

Set RC35 Working Mode	- calduino/?op=10?hc=X?wm=Y (hc - Heating Circuit 1 Ground Floor / Heath Circuit 2 Top Floor) (wm - Working Mode 0 = night, 1 = day, 2 = auto)
Set RC35 Temperature	- calduino/?op=11?hc=X?wm=Y?tp=ZZ (hc - Heath Circuit 1 Ground Floor / Heath Circuit 2 Top Floor) (wm - Working Mode 0 = night, 1 = day, 2 = holiday) (tp - Temperature x 2)
Set WW Temperature		- calduino/?op=12?tp=ZZ (tp - Temperature)
Set WW Working Mode		- calduino/?op=13?wm=Y (wm - Working Mode 0-off, 1-on, 2-auto)
Set WW One Time			- calduino/?op=14?wm=Y (wm - Working Mode 0-off, 1-on)
Set RC35 Program		- calduino/?op=15?hc=X?pr=YY (hc - Heating Circuit 1 Ground Floor / Heath Circuit 2 Top Floor) (pr - Program 0x00 = User 1, 0x01 = Family, 0x02 = Morning, 0x03 = Early morning, 0x04 = Evening, 0x05 = Midmorning, 0x06 = Afternoon, 0x07 = Midday, 0x08 = Single, 0x09 = Senioren, 0x0A User2)
Set WW Program			- calduino/?op=16?pr=YYY (0 = like the heating circuit, 255 = own program)
Set Switch Point		- calduino/?op=17?pr=Y?sp=XX?on=Z?d=D?h=HH?m=MM Y - the packet id of the user program to be read (0 - 0x38 - DHW, 1 - 0x39 - DHW Pump, 2 - 0x3F - HK1User1, 3 - 0x42 - HK1User2, 4 - 0x49 - HK2User1, 5 - 0x4C - HK2User2); sp= 0 to 41; op = 0 off, 1 = on, 7 = undef; d = day 0=monday,...,6=sunday

Reboot Calduino			- calduino/?op=20
Reconfigure Calduino	- calduino/?op=21?tz=X (tz - Time Zone - 1=Summer / 0=Winter)
Get Calduino stats basic- calduino/?op=22
Get Calduino stats advan- calduino/?op=23
Reset Calduino stats    - calduino/?op=24
Set EMS timeout			- calduino/?op=25?to=XX (XX is milliseconds/100) 11 = 1100 milliseconds


****************************************************/

#include <WiFlyHQ.h>
#include <EMSSerial.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include <stdint.h>
#include <time.h>

#pragma region defines

#undef DEBUG 1
//#define DEBUG 1 // When enabled Arduino sends the decoded values over the Serial port (eMSSerial) 

#define RESET_PIN 40 // Pin to reset the WiFly module

#define IN_EMS_BUFFER_SIZE 32
#define OUT_EMS_BUFFER_SIZE 7
#define HTTP_BUFFER_SIZE 80
#define EMS_MAX_READ 40

#define NTP_UPDATE_TIME 30	//	NTP Refresh time in minutes
#define IDLE_TIME 0 // Seconds to maintain the TCP connection opened
#define MAIN_LOOP_WAIT_TIME 1000 // Arduino Loop time in seconds
#define WIFLY_TIMEOUT 300000 // Timeout for WiFly module in milliseconds (5 min = 5 min * 60 sec/min * 1000 milliseconds/sec = 300000)
#define CALDUINO_BASIC_STADISTICS 1 
#define CALDUINO_FULL_STADISTICS 0
#define EMS_BUS_SPEED 9700
#define WIFLY_SPEED 9600
#define MAX_RETRIES 10

#define NO_OPERATION 0xFF

#define PC_ID 0x0B
#define RC35_ID 0x10
#define MC10_ID 0x08
#define RCTIME 0x06
#define UBA_WORKING_HOURS 0x14
#define UBA_MONITOR_FAST 0x18
#define UBA_MONITOR_SLOW 0x19
#define UBA_PARAMETER_WW 0X33
#define UBA_MONITOR_WW 0x34
#define UBA_FLAG_WW 0x35
#define UBA_WORKINGMODE_WW 0X37
#define UBA_PROGRAM_WW 0X38
#define UBA_PROGRAM_WWPUMP 0X39
#define RC35_MONITOR_HC1 0x3E
#define RC35_MONITOR_HC2 0x48
#define RC35_WORKINGMODE_HC1 0x3D
#define RC35_PROGRAM1_HC1 0x3F
#define RC35_PROGRAM2_HC1 0x42
#define RC35_WORKINGMODE_HC2 0x47
#define RC35_PROGRAM1_HC2 0x49
#define RC35_PROGRAM2_HC2 0x4C

#define GET_UBA_MONITOR 1
#define GET_UBA_MONITOR_WW 2
#define GET_RC35_MONITOR_HC1 3
#define GET_RC35_MONITOR_HC2 4
#define GET_GENERAL_MONITOR 5
#define GET_PROGRAM 6
#define GET_ALL_MONITORS 8
#define GET_RCTIME 9
#define SET_RC35_WORKING_MODE 10
#define SET_RC35_TEMPERATURE 11
#define SET_WW_TEMPERATURE 12
#define SET_WW_WORKING_MODE 13
#define SET_WW_ONE_TIME 14
#define SET_RC35_PROGRAM 15
#define SET_WW_PROGRAM 16
#define SET_SWITCH_POINT 17
#define EMS_OPERATIONS 20
#define REBOOT_CALDUINO 20
#define RECONFIGURE_CALDUINO 21
#define GET_CALDUINO_BASIC 22
#define GET_CALDUINO_FULL 23
#define RESET_CALDUINO_STATS 24
#define SET_EMS_TIMEOUT 25
#define WINTER_TIMEZONE 23
#define SUMMER_TIMEZONE 22
#define SUMMER_TIME 1
#define MAX_WW_TEMPERATURE 80
#define MIN_WW_TEMPERATURE 30
#define MAX_TEMPERATURE 29
#define MIN_TEMPERATURE 6
#define WW_ONETIME_ON  35
#define WW_ONETIME_OFF 3
#define SWITCHING_POINTS 42

#pragma endregion defines

#pragma region variables

// Buffers
char inEMSBuffer[IN_EMS_BUFFER_SIZE];
char outEMSBuffer[OUT_EMS_BUFFER_SIZE];
char httpBuffer[HTTP_BUFFER_SIZE];

// WiFly  vars
WiFly wifly;
const char* nTPServer = "129.6.15.30";
const uint16_t nTPServerPort = 123;
const uint16_t nTPUpdateTime = 600;
long EMSMaxAnswerTime = 1000; // Timeout for a EMS poll/request in milliseconds
uint8_t nTPTimeZone = 1;
const char* mySSID = "";
const char* myPassword = "";
const char* myDeviceID = "Calduino";

// Stats vars
uint32_t startedArduinoRTC = 0;
uint32_t lastEMSOperationRTC = 0;
uint32_t wiflyTimeout = 0;
uint32_t EMSTimeout = 0;
int operationsReceived = 0;
int operationsOK = 0;
int operationsNOK = 0;

// Common variables
long timeout;
boolean returnStatus;

// UBAMonitor Variables
char bufferFloat[4];
byte selImpTemp;
float curImpTemp;
float retTemp;
byte selBurnPow;
byte curBurnPow;
float flameCurr;
float sysPress;
float extTemp;
float boilTemp;
byte pumpMod;
boolean burnGas;
boolean fanWork;
boolean ignWork;
boolean heatPmp;
boolean wWHeat;
boolean wWcirc;
char srvCode[4] = { ' ', '-', ' ', '\0' };
int errCode;
unsigned long burnStarts;
unsigned long burnWorkMin;
unsigned long heatWorkMin;
unsigned long uBAWorkMin;

// UBAMonitorWW Variables
float wWCurTmp;
unsigned long wWWorkM;
unsigned long wWStarts;
uint8_t wWSelTemp;
uint8_t wWOneTime;
uint8_t wWWorkMod;
uint8_t wWPumpWorkMod;
uint8_t wWProgram;
uint8_t wWPumpProgram;

// RCMonitor Variables
float hCSelTemp[2];
boolean hCDayMod[2];
boolean hCSumMod[2];
char xmlTag[20];

//RC35 Working Mode Variables
float hCSelNightTemp[2];
float hCSelDayTemp[2];
byte hCWorkMod[2];
uint8_t hcProgram[2];

#pragma endregion variables

#pragma region EMSOperations

/**
Calculate the CRC code of the buffer passed as parameter.

@buffer buffer for which the CRC is calculated.
@len length of the buffer. 
@return CRC code.
*/
uint8_t crcCalculator(char * buffer, int len)
{
	uint8_t i, crc = 0x0;
	uint8_t d;
	for(i = 0; i < len - 2; i++)
	{
		d = 0;
		if(crc & 0x80)
		{
			crc ^= 12;
			d = 1;
		}
		crc = (crc << 1) & 0xfe;
		crc |= d;
		crc ^= buffer[i];
	}
	return crc;
}

/**
Check if the CRC code calculated for the buffer corresponds with the
CRC value received in the buffer (second to last position). 

@buffer buffer for which the CRC is calculated.
@len length of the buffer.
@return if the CRC received and the calculated are equal.
*/
boolean crcCheckOK(char * buffer, int len)
{
	int crc = crcCalculator(buffer, len);
	boolean crcOK = (crc == (uint8_t)buffer[len - 2]);
	return crcOK;
}

/**
Read one bus frame and return number of read bytes.
Includes a timeout in order not to block the program if there is no communication with the EMS Bus

@inEMSBuffer buffer where the income data will be stored.
@return number of read bytes.
*/
int readBytes(char * inEMSBuffer)
{
	int ptr = 0;

	// while there is available data
	while(eMSSerial3.available())
	{

		if((long)millis() > EMSTimeout) break;
		// if the first byte to be read is 0x00
		if((uint8_t)eMSSerial3.peek() == 0)
		{
			// skip zero's
			uint8_t temp = eMSSerial3.read();
		}
		else
		{
			// break the loop
			break;
		}
	}
	
	// read data until frame-error, max bytes are read or timeout
	while((!eMSSerial3.frameError()) && ptr < (EMS_MAX_READ))
	{
		if((long)millis() > EMSTimeout) break;
		if(eMSSerial3.available())
		{
			// store the information in buffer and update the number of bytes read
			inEMSBuffer[ptr] = eMSSerial3.read();
			ptr++;
		}
	}
	// flush the possible pending information left to be read (garbage)
	eMSSerial3.flush();
	// return the number of bytes read
	return ptr;
}

/**
Send a data frame to the EMS BUS.

@outEMSBuffer buffer where the output data is stored.
@len length of the buffer.
*/
void sendBuffer(char * outEMSBuffer, int len)
{
	char j;
	for(j = 0; j < len - 1; j++)
	{
		eMSSerial3.write(outEMSBuffer[j]);
		delay(3);
	}
	// write a EMS end - of - frame character to the port
	eMSSerial3.writeEOF();
	delay(2);
	eMSSerial3.flush();
}

/**
Reset the content of the EMS BUS.

@inEMSBuffer input Buffer.
*/
void resetBuffer(char *inEMSBuffer)
{
	memset(inEMSBuffer, 0, IN_EMS_BUFFER_SIZE);
}

/**
Wait until Calduino is polled and send the content of the outEMSBuffer.
If the connection with the EMS Buffer is not working it will timeout.

@outEMSBuffer buffer where the output data is stored.
@len length of the buffer.
@return whether the datagram has been sent in the available time or not.
*/
boolean sendRequest(char *outEMSBuffer)
{
	
	// calculate the CRC value in the sixth position of the outEMSBuffer
	outEMSBuffer[5] = crcCalculator(outEMSBuffer, 7);
	// wait until the polled address is our address (0x0B)
	char pollAddress = 0;

	// watchdog (maximum polling waiting time)
	EMSTimeout = (long)millis() + EMSMaxAnswerTime * 4;

	while((pollAddress & 0x7F) != PC_ID)
	{
		if((long)millis() > EMSTimeout) return false;

		int ptr = readBytes(inEMSBuffer);
		if(ptr == 2)
		{
			pollAddress = inEMSBuffer[0];
		}
	}

	// wait two milliseconds and send the outEMSBuffer (7 bytes + break) 
	delay(2);
	sendBuffer(outEMSBuffer, 7);
	return true;
	
}

#pragma endregion EMSOperations

#pragma region EMSCommands

/**
This function will call each EMS Command until their return status is true.

*/
void initialiceEMS()
{

	boolean getRCTimeVar = false;
	while(!getRCTimeVar)
	{
		delay(MAIN_LOOP_WAIT_TIME);
		getRCTimeVar = getRCTime();
	}

	boolean getUBAMonitorVar = false;
	while(!getUBAMonitorVar)
	{
		delay(MAIN_LOOP_WAIT_TIME);
		getUBAMonitorVar = getUBAMonitor();
	}

	boolean getUBAMonitorWarmWaterVar = false;
	while(!getUBAMonitorWarmWaterVar)
	{
		delay(MAIN_LOOP_WAIT_TIME);
		getUBAMonitorWarmWaterVar = getUBAMonitorWarmWater();
	}

	boolean getRC35MonitorHeatingCircuit1Var = false;
	while(!getRC35MonitorHeatingCircuit1Var)
	{
		delay(MAIN_LOOP_WAIT_TIME);
		getRC35MonitorHeatingCircuit1Var = getRC35MonitorHeatingCircuit(1);
	}

	boolean getRC35MonitorHeatingCircuit2Var = false;
	while(!getRC35MonitorHeatingCircuit2Var)
	{
		delay(MAIN_LOOP_WAIT_TIME);
		getRC35MonitorHeatingCircuit2Var = getRC35MonitorHeatingCircuit(2);
	}

}

/**
Send a request to EMS to read a program (switch points) in the boiler and send the answer in XML format via WiFly module. 
Due to reception problems in datagram greater than 40 bytes, the read is splited in blocks of 14 bytes
@selProgram the packet id of the user program to be read (0 - 0x38 - DHW, 1 - 0x39 - DHW Pump, 2 - 0x3F - HK1User1, 3 - 0x42 - HK1User2, 4 - 0x49 - HK2User1, 5 - 0x4C - HK2User2)
@return whether the operation has been correctly executed or not.
*/
boolean getProgramSwitchPoints(byte selProgram)
{
	boolean returnStatus = false;
	byte batchReadSize = 14;
	byte programSwitchPoints[SWITCHING_POINTS][2];

	switch (selProgram)
	{
		case 0 :
			selProgram = 0x38;
			break;
		case 1:
			selProgram = 0x39;
			break;
		case 2:
			selProgram = 0x3F;
			break;
		case 3:
			selProgram = 0x42;
			break;
		case 4:
			selProgram = 0x49;
			break;
		case 5:
			selProgram = 0x4C;
			break;
		default: goto sendXML;
			break;
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;

	// third position is the message type
	outEMSBuffer[2] = selProgram;

	// fifth position is the length of the data requested. Always ask for the maximum number of bytes.
	outEMSBuffer[4] = batchReadSize;

	for (int i = 0; i < (SWITCHING_POINTS * 2 / batchReadSize); i++)
	{
		// fourth position is the offset in the buffer. We want to read all the buffer so 0
		outEMSBuffer[3] = i * batchReadSize;

		// once the buffer is loaded, send the request.
		sendRequest(outEMSBuffer);

		// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
		long timeout = (long)millis() + EMSMaxAnswerTime;

		// wait until timeout or some new data in the EMS-Bus
		while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

		// if there is data to be read
		if (eMSSerial3.available())
		{
			// read the information sent
			int ptr = readBytes(inEMSBuffer);

			// if more than 4 bytes are read (datagram received)
			if (ptr > 4)
			{
				// check if the CRC of the information received is correct
				if (crcCheckOK(inEMSBuffer, ptr))
				{
					// check if the operation type returned corresponds with the one requested
					if (inEMSBuffer[2] == selProgram)
					{
						for (int j = 0; j < batchReadSize/2; j++)
						{
							programSwitchPoints[j + (i * batchReadSize / 2)][0] = inEMSBuffer[(j * 2) + 4];
							programSwitchPoints[j + (i * batchReadSize / 2)][1] = inEMSBuffer[(j * 2) + 4 + 1];
						}

						returnStatus = true;
					}
					else
					{
						goto sendXML;
					}
				}
				else
				{
					goto sendXML;
				}
			}
		}
	}	

sendXML:
	wifly.sendWifly(F("<GetProgram>"));
	if (returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		for (int k = 0; k < SWITCHING_POINTS; k++)
		{
#if DEBUG
			eMSSerial.println(F("--- "));
			eMSSerial.print(F("ID: ")); eMSSerial.println(k, DEC);
			eMSSerial.print(F("Byte 1: ")); eMSSerial.println(programSwitchPoints[k][0], BIN);
			eMSSerial.print(F("Byte 2: ")); eMSSerial.println(programSwitchPoints[k][1], BIN);
			eMSSerial.print(F("Day: ")); eMSSerial.println((programSwitchPoints[k][0] >> 5), DEC);
			eMSSerial.print(F("Hour: ")); eMSSerial.println((int)(programSwitchPoints[k][1] / 6), DEC);
			eMSSerial.print(F("Minute: ")); eMSSerial.println((int)((programSwitchPoints[k][1] % 6) * 10), DEC);
			eMSSerial.print(F("Action: ")); eMSSerial.println((programSwitchPoints[k][0] & 7), DEC);			
#endif	
			wifly.sendWifly(F("<SwitchPoint>"));
			wifly.sendWiflyXML(F("ID"), k);
			wifly.sendWiflyXML(F("Day"), (byte) (programSwitchPoints[k][0] >> 5));
			wifly.sendWiflyXML(F("Hour"), (byte) (programSwitchPoints[k][1] / 6));
			wifly.sendWiflyXML(F("Minute"), (byte) ((programSwitchPoints[k][1] % 6) * 10));
			wifly.sendWiflyXML(F("Action"), (byte) (programSwitchPoints[k][0] & 7));
			wifly.sendWifly(F("</SwitchPoint>"));
		}
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWifly(F("</GetProgram>"));

	return returnStatus;
}


/**
Send a request to EMS to read current time stored in the boiler and send the answer in XML format via WiFly module

@return whether the operation has been correctly executed or not.
*/
boolean getRCTime()
{
	// RCTime Variables
	byte year;
	byte month;
	byte day;
	byte hour;
	byte minute;
	byte second;
	boolean returnStatus = false;

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = RCTIME;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested. Always ask for the maximum number of bytes.
	outEMSBuffer[4] = 20;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	long timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == RCTIME)
				{
					year = inEMSBuffer[4];
					month = inEMSBuffer[5];
					day = inEMSBuffer[7];
					hour = inEMSBuffer[6];
					minute = inEMSBuffer[8];
					second = inEMSBuffer[9];

					returnStatus = true;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
	wifly.sendWifly(F("<RCTime>"));
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("Year"), year);
		wifly.sendWiflyXML(F("Month"), month);
		wifly.sendWiflyXML(F("Day"), day);
		wifly.sendWiflyXML(F("Hour"), hour);
		wifly.sendWiflyXML(F("Minute"), minute);
		wifly.sendWiflyXML(F("Second"), second);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWifly(F("</RCTime>"));

#if DEBUG
	eMSSerial.println(F("Year: ")); eMSSerial.println(year, DEC);
	eMSSerial.print(F("Month: ")); eMSSerial.println(month, DEC);
	eMSSerial.print(F("Day: ")); eMSSerial.println(day, DEC);
	eMSSerial.print(F("Hour: ")); eMSSerial.println(hour, DEC);
	eMSSerial.print(F("Minute: ")); eMSSerial.println(minute, DEC);
	eMSSerial.print(F("Second: ")); eMSSerial.println(second, DEC);
#endif

	return returnStatus;
}

/** 
Send several EMS request to get all the available data and send the answer in XML format via WiFly module

@return whether the operation has been correctly executed or not.
*/

boolean getAllMonitors()
{
	wifly.sendWifly(F("<AllMonitors>"));
	returnStatus = getUBAMonitor();
	returnStatus &= getUBAMonitorWarmWater();
	returnStatus &= getRC35MonitorHeatingCircuit(1);
	returnStatus &= getRC35MonitorHeatingCircuit(2);
	returnStatus &= getGeneralMonitor(true);
	returnStatus &= getCalduinoStats(1);
	wifly.sendWifly(F("</AllMonitors>"));

	return returnStatus;

}
/**
Send a EMS request to read UBA Monitor and send the answer in XML format via WiFly module

@return whether the operation has been correctly executed or not.
*/
boolean getUBAMonitor()
{

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_MONITOR_FAST;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 32;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_MONITOR_FAST)
				{
					selImpTemp = (uint8_t)inEMSBuffer[4];
					curImpTemp = ((float)((((uint8_t)inEMSBuffer[5] << 8) + (uint8_t)inEMSBuffer[6]))) / 10;
					retTemp = ((float)((((uint8_t)inEMSBuffer[17] << 8) + (uint8_t)inEMSBuffer[18]))) / 10;
					selBurnPow = (uint8_t)inEMSBuffer[7];
					curBurnPow = (uint8_t)inEMSBuffer[8];
					flameCurr = ((float)((((uint8_t)inEMSBuffer[19] << 8) + (uint8_t)inEMSBuffer[20]))) / 10;
					sysPress = ((float)((uint8_t)inEMSBuffer[21]) / 10);

					byte auxiliarVariable = inEMSBuffer[11];
					burnGas = bitRead(auxiliarVariable, 0);
					fanWork = bitRead(auxiliarVariable, 2);
					ignWork = bitRead(auxiliarVariable, 3);
					heatPmp = bitRead(auxiliarVariable, 5);
					wWHeat = bitRead(auxiliarVariable, 6);
					wWcirc = bitRead(auxiliarVariable, 7);
					srvCode[0] = (uint8_t)inEMSBuffer[22];
					srvCode[2] = (uint8_t)inEMSBuffer[23];
					errCode = ((int)((((uint8_t)inEMSBuffer[24] << 8) + (uint8_t)inEMSBuffer[25])));

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}


	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_MONITOR_SLOW;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 24;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_MONITOR_SLOW)
				{
					extTemp = ((float)((((uint8_t)inEMSBuffer[4] << 8) + (uint8_t)inEMSBuffer[5]))) / 10;
					boilTemp = ((float)((((uint8_t)inEMSBuffer[6] << 8) + (uint8_t)inEMSBuffer[7]))) / 10;
					pumpMod = (uint8_t)inEMSBuffer[13];
					burnStarts = (((unsigned long)(uint8_t)inEMSBuffer[14]) << 16) + (((unsigned long)(uint8_t)inEMSBuffer[15]) << 8) + ((uint8_t)inEMSBuffer[16]);
					burnWorkMin = (((unsigned long)(uint8_t)inEMSBuffer[17]) << 16) + (((unsigned long)(uint8_t)inEMSBuffer[18]) << 8) + ((uint8_t)inEMSBuffer[19]);
					heatWorkMin = (((unsigned long)(uint8_t)inEMSBuffer[23]) << 16) + (((unsigned long)(uint8_t)inEMSBuffer[24]) << 8) + ((uint8_t)inEMSBuffer[25]);
					
					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_WORKING_HOURS;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 3;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_WORKING_HOURS)
				{
					uBAWorkMin = (((unsigned long)(uint8_t)inEMSBuffer[4]) << 16) + (((unsigned long)(uint8_t)inEMSBuffer[5]) << 8) + ((uint8_t)inEMSBuffer[6]);

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
	sendXML:
	wifly.sendWifly(F("<UBAMonitor>"));
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("SelImpTemp"), selImpTemp); //0
		wifly.sendWiflyXML(F("CurImpTemp"), curImpTemp); //1
		wifly.sendWiflyXML(F("RetTemp"), retTemp);       //2
		wifly.sendWiflyXML(F("ExtTemp"), extTemp);       //3
		wifly.sendWiflyXML(F("BoilTemp"), boilTemp);     //4
		wifly.sendWiflyXML(F("SelBurnPow"), selBurnPow); //5
		wifly.sendWiflyXML(F("CurBurnPow"), curBurnPow); //6
		wifly.sendWiflyXML(F("FlameCurr"), flameCurr);   //7
		wifly.sendWiflyXML(F("SysPress"), sysPress);     //8
		wifly.sendWiflyXML(F("PumpMod"), pumpMod);       //9
		wifly.sendWiflyXML(F("BurnStarts"), burnStarts); //10
		wifly.sendWiflyXML(F("BurnWorkMin"), burnWorkMin);//11
		wifly.sendWiflyXML(F("HeatWorkMin"), heatWorkMin);//12
		wifly.sendWiflyXML(F("UBAWorkMin"), uBAWorkMin);  //13
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);//14
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}

	wifly.sendWifly(F("</UBAMonitor>"));

#if DEBUG
	eMSSerial.print(F("Selected Flow (impulsion) Temperature: ")); eMSSerial.println(selImpTemp);
	eMSSerial.print(F("Current Flow (impulsion) Temperature: ")); eMSSerial.println(curImpTemp, 1);
	eMSSerial.print(F("Return Temperature: ")); eMSSerial.println(retTemp, 1);
	eMSSerial.print(F("Selected Burner Power: ")); eMSSerial.print(selBurnPow); eMSSerial.println(F("%"));
	eMSSerial.print(F("Current Burner Power: ")); eMSSerial.print(curBurnPow); eMSSerial.println(F("%"));
	eMSSerial.print(F("Flame current: ")); eMSSerial.print(flameCurr, 1); eMSSerial.println(F(" uA"));
	eMSSerial.print(F("System pressure: ")); eMSSerial.print(sysPress, 1); eMSSerial.println(F(" bar"));
	eMSSerial.print(F("External Temperature: ")); eMSSerial.println(extTemp);
	eMSSerial.print(F("Boiler Temperature: ")); eMSSerial.println(boilTemp, 1);
	eMSSerial.print(F("Pump Modulation: ")); eMSSerial.print(pumpMod); eMSSerial.println(F("%"));
	eMSSerial.print(F("Counting Burner Starts: ")); eMSSerial.print(burnStarts); eMSSerial.println(F(" times"));
	eMSSerial.print(F("Total Working Minutes: ")); eMSSerial.print(burnWorkMin); eMSSerial.println(F(" minutes"));
	eMSSerial.print(F("Heating Working Minutes: ")); eMSSerial.print(heatWorkMin); eMSSerial.println(F(" minutes"));
	eMSSerial.print(F("Universal Automatic Burner working minutes: ")); eMSSerial.println(uBAWorkMin, DEC);
#endif

	return (returnStatus);
}

/**
Send a EMS request to read UBA Warm Water Monitor and send the answer in XML format via WiFly module

@return whether the operation has been correctly executed or not.
*/
boolean getUBAMonitorWarmWater()
{
	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_MONITOR_WW;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 24;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_MONITOR_WW)
				{
					wWCurTmp = ((float)((((uint8_t)inEMSBuffer[5] << 8) + (uint8_t)inEMSBuffer[6]))) / 10;
					wWStarts = (((unsigned long)(uint8_t)inEMSBuffer[17]) << 16) + (((unsigned long)(uint8_t)inEMSBuffer[18]) << 8) + ((uint8_t)inEMSBuffer[19]);
					wWWorkM = (((unsigned long)(uint8_t)inEMSBuffer[14]) << 16) + (((unsigned long)(uint8_t)inEMSBuffer[15]) << 8) + ((uint8_t)inEMSBuffer[16]);
					wWOneTime = bitRead((uint8_t)inEMSBuffer[9], 1);
					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}
	
	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_PARAMETER_WW;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 10;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_PARAMETER_WW)
				{
					wWSelTemp = (uint8_t)inEMSBuffer[6];

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0B is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_WORKINGMODE_WW;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 20;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_WORKINGMODE_WW)
				{
					wWWorkMod = (uint8_t)inEMSBuffer[6];
					wWPumpWorkMod = (uint8_t)inEMSBuffer[7];

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
    sendXML:
	wifly.sendWifly(F("<UBAMonitorWW>"));
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("WWCurTmp"), wWCurTmp);			//0
		wifly.sendWiflyXML(F("WWSelTemp"), wWSelTemp);			//1		
		wifly.sendWiflyXML(F("WWStarts"), wWStarts);			//2
		wifly.sendWiflyXML(F("WWWorkM"), wWWorkM);				//3
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);	//6
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWifly(F("</UBAMonitorWW>"));

#if DEBUG
	eMSSerial.print(F("Warm Water Current Temperature: ")); eMSSerial.println(wWCurTmp, 1);
	eMSSerial.print(F("Counting warm water Starts: ")); eMSSerial.print(wWStarts); eMSSerial.println(F(" times"));
	eMSSerial.print(F("Warm water Working Minutes: ")); eMSSerial.print(wWWorkM); eMSSerial.println(F(" minutes"));
	eMSSerial.print(F("Warm Water Selected Temperature: ")); eMSSerial.println(wWSelTemp);
	eMSSerial.print(F("Warm Water Working Mode: ")); eMSSerial.println(wWWorkMod);
	eMSSerial.print(F("Warm Water Pump Working Mode: ")); eMSSerial.println(wWPumpWorkMod);
#endif

	return returnStatus;
}

/**
Send a EMS request to read RC35 Monitor Heating Circuit and send the answer in XML format via WiFly module
@selHC the Heating Circuit for which we want to obtain the information (1 or 2)
@return whether the operation has been correctly executed or not.
*/
boolean getRC35MonitorHeatingCircuit(byte selHC)
{
	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type. Depends on the Heat Circuit passed as parameter
	if(selHC == 1)
	{
		outEMSBuffer[2] = RC35_MONITOR_HC1;
		strcpy_P(xmlTag, PSTR("RCMonitorHC1"));
	}
	else if(selHC == 2)
	{
		outEMSBuffer[2] = RC35_MONITOR_HC2;
		strcpy_P(xmlTag, PSTR("RCMonitorHC2"));
	}
	else goto sendXML;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 10;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == outEMSBuffer[2])
				{
					hCSelTemp[selHC - 1] = ((float)((uint8_t)inEMSBuffer[6])) / 2;
					hCDayMod[selHC - 1] = bitRead((uint8_t)inEMSBuffer[5], 1);
					hCSumMod[selHC - 1] = bitRead((uint8_t)inEMSBuffer[5], 0);

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type
	if(selHC == 1)
	{
		outEMSBuffer[2] = RC35_WORKINGMODE_HC1;
	}
	else if(selHC == 2)
	{
		outEMSBuffer[2] = RC35_WORKINGMODE_HC2;
	}
	else goto sendXML;

	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 12;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == outEMSBuffer[2])
				{
					hCSelNightTemp[selHC - 1] = ((float)((uint8_t)inEMSBuffer[5])) / 2;
					hCSelDayTemp[selHC - 1] = ((float)((uint8_t)inEMSBuffer[6])) / 2;
					hCWorkMod[selHC - 1] = (uint8_t)inEMSBuffer[11];

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}
	



	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
    sendXML:
	wifly.sendWiflyXMLTag(xmlTag,0);
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("HCSelTemp"), hCSelTemp[selHC - 1]);	//0	
		wifly.sendWiflyXML(F("HCDayMod"), hCDayMod[selHC - 1]);		//1
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);       //2
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWiflyXMLTag(xmlTag, 1);

	return returnStatus;
}

/**
Send a number of EMS requests to read all the parameters that can be changed using Calduino
@onlyPrint whether if the variables should be updated (sending EMS commands) or only printed
@return whether the operation has been correctly executed or not.
*/
boolean getGeneralMonitor(boolean onlyPrint)
{
	if(onlyPrint) goto sendXML;

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_MONITOR_FAST;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 32;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_MONITOR_FAST)
				{
					byte auxiliarVariable = inEMSBuffer[11];
					burnGas = bitRead(auxiliarVariable, 0);
					fanWork = bitRead(auxiliarVariable, 2);
					ignWork = bitRead(auxiliarVariable, 3);
					heatPmp = bitRead(auxiliarVariable, 5);
					wWHeat = bitRead(auxiliarVariable, 6);
					wWcirc = bitRead(auxiliarVariable, 7);
					srvCode[0] = (uint8_t)inEMSBuffer[22];
					srvCode[2] = (uint8_t)inEMSBuffer[23];
					errCode = ((int)((((uint8_t)inEMSBuffer[24] << 8) + (uint8_t)inEMSBuffer[25])));

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}


	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = RC35_WORKINGMODE_HC1;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 12;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == outEMSBuffer[2])
				{
					hCSelNightTemp[0] = ((float)((uint8_t)inEMSBuffer[5])) / 2;
					hCSelDayTemp[0] = ((float)((uint8_t)inEMSBuffer[6])) / 2;
					hCWorkMod[0] = (uint8_t)inEMSBuffer[11];

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = RC35_WORKINGMODE_HC2;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 12;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == outEMSBuffer[2])
				{
					hCSelNightTemp[1] = ((float)((uint8_t)inEMSBuffer[5])) / 2;
					hCSelDayTemp[1] = ((float)((uint8_t)inEMSBuffer[6])) / 2;
					hCWorkMod[1] = (uint8_t)inEMSBuffer[11];

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type. Depends on the Heat Circuit passed as parameter
	outEMSBuffer[2] = RC35_MONITOR_HC1;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 10;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == outEMSBuffer[2])
				{
					hCSelTemp[0] = ((float)((uint8_t)inEMSBuffer[6])) / 2;
					hCSumMod[0] = bitRead((uint8_t)inEMSBuffer[5], 0);
					hCDayMod[0] = bitRead((uint8_t)inEMSBuffer[5], 1);

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type. Depends on the Heat Circuit passed as parameter
	outEMSBuffer[2] = RC35_MONITOR_HC2;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 10;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == outEMSBuffer[2])
				{
					hCSelTemp[1] = ((float)((uint8_t)inEMSBuffer[6])) / 2;
					hCSumMod[1] = bitRead((uint8_t)inEMSBuffer[5], 0);
					hCDayMod[1] = bitRead((uint8_t)inEMSBuffer[5], 1);

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_MONITOR_WW;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 24;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_MONITOR_WW)
				{
					wWOneTime = bitRead((uint8_t)inEMSBuffer[9], 1);
					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_PARAMETER_WW;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 10;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_PARAMETER_WW)
				{
					wWSelTemp = (uint8_t)inEMSBuffer[6];
					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_WORKINGMODE_WW;
	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 20;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_WORKINGMODE_WW)
				{
					wWProgram = (uint8_t)inEMSBuffer[4];
					wWPumpProgram = (uint8_t)inEMSBuffer[5];
					wWWorkMod = (uint8_t)inEMSBuffer[6];
					wWPumpWorkMod = (uint8_t)inEMSBuffer[7];

					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}	

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. Masked with 0x80 as a read command.
	outEMSBuffer[1] = RC35_ID | 0x80;

	// third position is the message type
	outEMSBuffer[2] = RC35_PROGRAM1_HC1;

	// fourth position is the offset in the buffer. We want to read only position 88 of the buffer, so 88 - 4 = 84
	outEMSBuffer[3] = 84;

	// fifth position is the length of the data requested. We want to read only one byte
	outEMSBuffer[4] = 1;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if (eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if (ptr > 4)
		{
			// check if the CRC of the information received is correct
			if (crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if (inEMSBuffer[2] == RC35_PROGRAM1_HC1)
				{
					hcProgram[0] = (uint8_t)inEMSBuffer[4];
					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. Masked with 0x80 as a read command.
	outEMSBuffer[1] = RC35_ID | 0x80;

	// third position is the message type
	outEMSBuffer[2] = RC35_PROGRAM1_HC2;

	// fourth position is the offset in the buffer. We want to read only position 88 of the buffer, so 88 - 4 = 84
	outEMSBuffer[3] = 84;

	// fifth position is the length of the data requested. We want to read only one byte
	outEMSBuffer[4] = 1;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if (eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if (ptr > 4)
		{
			// check if the CRC of the information received is correct
			if (crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if (inEMSBuffer[2] == RC35_PROGRAM1_HC2)
				{
					hcProgram[1] = (uint8_t)inEMSBuffer[4];
					returnStatus = true;
				}
				else
				{
					// stop the operation in order to save time
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
sendXML:
	wifly.sendWifly(F("<GeneralMonitor>"));
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("HC1SelDayTemp"), hCSelDayTemp[0]);    //0 
		wifly.sendWiflyXML(F("HC2SelDayTemp"), hCSelDayTemp[1]);    //1
		wifly.sendWiflyXML(F("HC1SelNightTemp"), hCSelNightTemp[0]);//2
		wifly.sendWiflyXML(F("HC2SelNightTemp"), hCSelNightTemp[1]);//3
		wifly.sendWiflyXML(F("HC1SelTemp"), hCSelTemp[0]);
		wifly.sendWiflyXML(F("HC2SelTemp"), hCSelTemp[1]);
		wifly.sendWiflyXML(F("HC1WorkMod"), hCWorkMod[0]);
		wifly.sendWiflyXML(F("HC2WorkMod"), hCWorkMod[1]);
		wifly.sendWiflyXML(F("HC1DayMod"), hCDayMod[0]);
		wifly.sendWiflyXML(F("HC2DayMod"), hCDayMod[1]);
		wifly.sendWiflyXML(F("HC1SumMod"), hCSumMod[0]);
		wifly.sendWiflyXML(F("HC2SumMod"), hCSumMod[1]);
		wifly.sendWiflyXML(F("HC1Prog1"), hcProgram[0]);
		wifly.sendWiflyXML(F("HC2Prog1"), hcProgram[1]);
		wifly.sendWiflyXML(F("WWSelTemp"), wWSelTemp);
		wifly.sendWiflyXML(F("WWOneTime"), wWOneTime);
		wifly.sendWiflyXML(F("WWWorkMod"), wWWorkMod);
		wifly.sendWiflyXML(F("WWPumpWorkMod"), wWPumpWorkMod);
		wifly.sendWiflyXML(F("WWProgram"), wWProgram);
		wifly.sendWiflyXML(F("WWPumpProgram"), wWPumpProgram);
		wifly.sendWiflyXML(F("BurnGas"), burnGas);
		wifly.sendWiflyXML(F("IgnWork"), ignWork);
		wifly.sendWiflyXML(F("FanWork"), fanWork);
		wifly.sendWiflyXML(F("HeatPmp"), heatPmp);
		wifly.sendWiflyXML(F("WWHeat"), wWHeat);
		wifly.sendWiflyXML(F("WWCirc"), wWcirc);
		wifly.sendWiflyXML(F("SrvCode"), srvCode);
		wifly.sendWiflyXML(F("ErrCode"), errCode);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWifly(F("</GeneralMonitor>"));

	return returnStatus;
}


/**
Send a EMS command to set the selected RC35 Heating Circuit to the desired Program
@selHC the Heating Circuit that we want to modify (1 is HC1, 2 is HC2)
@selProgram the working program we want to configure (0x00 = User 1, 0x01 = Family, 0x02 = Morning, 0x03 = Early morning, 0x04 = Evening, 0x05 = Midmorning, 0x06 = Afternoon, 0x07 = Midday, 0x08 = Single, 0x09 = Senioren, 0x0A User2)
@return whether the operation has been correctly executed or not.
*/
boolean setRC35Program(byte selHC, byte selProgram)
{
	// RCMonitor Variables
	boolean returnStatus = false;
	char xmlTag[20];
	long timeout;

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. No Masked with 0x80!
	outEMSBuffer[1] = RC35_ID;

	// third position is the message type
	if (selHC == 1)
	{
		outEMSBuffer[2] = RC35_PROGRAM1_HC1;
		strcpy_P(xmlTag, PSTR("SetRCHC1Program"));
	}
	else if (selHC == 2)
	{
		outEMSBuffer[2] = RC35_PROGRAM1_HC2;
		strcpy_P(xmlTag, PSTR("SetRCHC2Program"));
	}
	else goto sendXML;

	// fourth position is the offset in the buffer. Working mode is at position 88, so 88-4=84
	outEMSBuffer[3] = 84;

	// fifth position is the data to send
	if (selProgram > 0x0A || selProgram < 0x00) goto sendXML;
	else outEMSBuffer[4] = selProgram;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if (eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if (inEMSBuffer[0] == 0x01)
		{
			returnStatus = true;
		}
		else
		{
			// stop the operation in order to save time
			returnStatus = false;
			goto sendXML;
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. Masked with 0x80 as a read command.
	outEMSBuffer[1] = RC35_ID | 0x80;

	// third position is the message type
	if (selHC == 1) outEMSBuffer[2] = RC35_PROGRAM1_HC1;
	else outEMSBuffer[2] = RC35_PROGRAM1_HC2;

	// fourth position is the offset in the buffer. We want to read only position 88 of the buffer, so 88 - 4 = 84
	outEMSBuffer[3] = 84;

	// fifth position is the length of the data requested. We want to read only one byte
	outEMSBuffer[4] = 1;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if (eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if (ptr > 4)
		{
			// check if the CRC of the information received is correct
			if (crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if (inEMSBuffer[2] == outEMSBuffer[2])
				{
					hcProgram[selHC - 1] = (uint8_t)inEMSBuffer[4];
					
					// check if the working mode has been effectively performed
					if (hcProgram[selHC - 1] == selProgram)
					{
						returnStatus = true;
					}
					else
					{
						returnStatus = false;
					}
				}
				else
				{
					returnStatus = false;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
sendXML:
	wifly.sendWiflyXMLTag(xmlTag, 0);
	if (returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("HCProgram"), hcProgram[selHC - 1]);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);

	}
	wifly.sendWiflyXMLTag(xmlTag, 1);

#if DEBUG
	eMSSerial.print(F("Heating Circuit X - Working Mode: ")); eMSSerial.println(hCWorkMod[selHC - 1]);
#endif
	return returnStatus;

}

/**
Send a EMS command to set the selected RC35 Heating Circuit to the desired Mode
@selHC the Heating Circuit that we want to modify (1 is HC1, 2 is HC2)
@selMode the working mode we want to configure (0-night, 1-day, 2-auto)
@return whether the operation has been correctly executed or not.
*/
boolean setRC35WorkingMode(byte selHC, byte selMode)
{
	// RCMonitor Variables
	boolean returnStatus = false;
	byte hCWorkMod[2];
	char xmlTag[20];
	long timeout;

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. No Masked with 0x80!
	outEMSBuffer[1] = RC35_ID;

	// third position is the message type
	if(selHC == 1)
	{
		outEMSBuffer[2] = RC35_WORKINGMODE_HC1;
		strcpy_P(xmlTag, PSTR("SetRCHC1WorkingMode"));
	}
	else if(selHC == 2)
	{
		outEMSBuffer[2] = RC35_WORKINGMODE_HC2;
		strcpy_P(xmlTag, PSTR("SetRCHC2WorkingMode"));
	}
	else goto sendXML;
	// fourth position is the offset in the buffer. Working mode is at position 11, so 11-4=7
	outEMSBuffer[3] = 0x07;

	// fifth position is the data to send
	if(selMode > 2 || selMode < 0) goto sendXML;
	else outEMSBuffer[4] = selMode;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if(inEMSBuffer[0] == 0x01)
		{
			returnStatus = true;
		}
		else
		{
			// stop the operation in order to save time
			returnStatus = false;
			goto sendXML;
		}
	}
	
	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. Masked with 0x80 as a read command.
	outEMSBuffer[1] = RC35_ID | 0x80;

	// third position is the message type
	if(selHC == 1) outEMSBuffer[2] = RC35_WORKINGMODE_HC1;
	else outEMSBuffer[2] = RC35_WORKINGMODE_HC2;

	// fourth position is the offset in the buffer. We want to read all the buffer so 0
	outEMSBuffer[3] = 0;

	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 12;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == outEMSBuffer[2])
				{
					hCWorkMod[selHC - 1] = (uint8_t)inEMSBuffer[11];

					// check if the working mode has been effectively performed
					if(hCWorkMod[selHC - 1] == selMode)
					{
						returnStatus = true;
					}
					else
					{
						returnStatus = false;
					}
				}
				else
				{
					returnStatus = false;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
    sendXML:
	wifly.sendWiflyXMLTag(xmlTag, 0);
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("HCWorkMod"), hCWorkMod[selHC - 1]);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);

	}
	wifly.sendWiflyXMLTag(xmlTag, 1);

#if DEBUG
	eMSSerial.print(F("Heating Circuit X - Working Mode: ")); eMSSerial.println(hCWorkMod[selHC - 1]);
#endif
	return returnStatus;

}

/**
Send a EMS command to set the selected RC35 Heating Circuit to the desired Temperature in the selected Mode
@selHC the Heating Circuit that we want to modify (1 is HC1, 2 is HC2)
@selMode the working mode we want to configure (0-night, 1-day, 2-holidays)
@selTmp the desired temperature to be configured in the selMode (multiplied by two)
@return whether the operation has been correctly executed or not.
*/
boolean setRC35SelectedTemperature(byte selHC, byte selMode, byte selTmp)
{
	// RCMonitor Variables
	float hCSelModTemp;
	char xmlTag[20];
	char xmlTag1[20];
	long timeout;
	boolean returnStatus = false;

	// load outEMSBuffer with corresponding values
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID.
	outEMSBuffer[1] = RC35_ID;

	// third position is the message type
	if(selHC == 1)
	{
		outEMSBuffer[2] = RC35_WORKINGMODE_HC1;
		strcpy_P(xmlTag, PSTR("SetRCHC1Temperature"));
	}
	else if(selHC == 2)
	{
		outEMSBuffer[2] = RC35_WORKINGMODE_HC2;
		strcpy_P(xmlTag, PSTR("SetRCHC2Temperature"));
	}
	else goto sendXML;

	// fourth position is the offset in the buffer. Night is byte 5 (Offset=5-4=1), Day is byte 6 (Offset=2), Holidays is byte 7 (Offset=3)
	if(selMode > 2 || selMode < 0) goto sendXML;
	else outEMSBuffer[3] = selMode+1;

	// fifth position is the data to send
	if((selTmp < (MAX_TEMPERATURE * 2)) && (selTmp > (MIN_TEMPERATURE * 2))) outEMSBuffer[4] = selTmp;
	else goto sendXML;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if(inEMSBuffer[0] == 0x01)
		{
			returnStatus = true;
		}
		else
		{
			// stop the operation in order to save time
			returnStatus = false;
			goto sendXML;
		}
	}
	
	// load outEMSBuffer with corresponding values
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;

	// third position is the message type
	if(selHC == 1) outEMSBuffer[2] = RC35_WORKINGMODE_HC1;
	else outEMSBuffer[2] = RC35_WORKINGMODE_HC2;

	// fourth position is the offset in the buffer.  Night is byte 5 (Offset=5-4=1), Day is byte 6 (Offset=2), Holidays is byte 7 (Offset=3)
	outEMSBuffer[3] = selMode + 1;
	switch (selMode)
	{
		case 0:
			// mode night			
			strcpy_P(xmlTag1, PSTR("HCSelNightTemp"));
			break;
		case 1:
			// mode day
			strcpy_P(xmlTag1, PSTR("HCSelDayTemp"));
			break;
		case 2:
			// mode holidays
			strcpy_P(xmlTag1, PSTR("HCSelHolyTemp"));
			break;
	}

	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 1;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == outEMSBuffer[2])
				{
					hCSelModTemp = ((float)((uint8_t)inEMSBuffer[4])) / 2;

					// check if the working mode has been effectively performed
					if(hCSelModTemp == ((float)((uint8_t)selTmp)) / 2)
					{
						returnStatus = true;
					}
					else
					{
						returnStatus = false;
					}
				}
				else
				{
					returnStatus = false;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
sendXML:
	wifly.sendWiflyXMLTag(xmlTag, 0);
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(xmlTag1, hCSelModTemp);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWiflyXMLTag(xmlTag, 1);

#if DEBUG
	eMSSerial.print(F("Heating Circuit X - Room Selected Y Temperature: ")); eMSSerial.println(hCSelModTemp, 1);
#endif

	return (returnStatus);

}

/**
Send a EMS command to set the selected Temperature for warm water
@selTmp the desired temperature
@return whether the operation has been correctly executed or not.
*/
boolean setDHWTemperature(byte selTmp)
{
	boolean returnStatus = false;
	byte wWSelTemp;
	long timeout;

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID.
	outEMSBuffer[1] = MC10_ID;

	// third position is the message type
	outEMSBuffer[2] = UBA_PARAMETER_WW;

	// fourth position is the offset in the buffer. Selected Temperature is position 6 - 4 = 2
	outEMSBuffer[3] = 2;

	// fifth position is the data to send
	if(selTmp < MAX_WW_TEMPERATURE && selTmp > MIN_WW_TEMPERATURE) outEMSBuffer[4] = selTmp; 
	else goto sendXML;	

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if(inEMSBuffer[0] == 0x01)
		{
			returnStatus = true;
		}
		else
		{
			// stop the operation in order to save time
			returnStatus = false;
			goto sendXML;
		}
	}
	
	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;

	// third position is the message type
	outEMSBuffer[2] = UBA_PARAMETER_WW;

	// fourth position is the offset in the buffer. Selected Temperature is position 6 - 4 = 2
	outEMSBuffer[3] = 2;

	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 1;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_PARAMETER_WW)
				{
					wWSelTemp = (uint8_t)inEMSBuffer[4];
					
					// check if the temperature set corresponds with the demanded one
					if(wWSelTemp == selTmp)
					{
						returnStatus = true;
					}
					else
					{
						returnStatus = false;
					}
				}
				else
				{
					returnStatus = false;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
sendXML:
	wifly.sendWifly(F("<SetWarmWatterTemperature>"));
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("wWSelTemp"), wWSelTemp);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWifly(F("</SetWarmWatterTemperature>"));

#if DEBUG
	eMSSerial.print(F("Warm Water Selected Temperature: ")); eMSSerial.println(wWSelTemp);
#endif

	return (returnStatus);
}


/**
Send a EMS command to set the warm watter program (WW and Pump)
@selProgram the working program we want to configure (0 = like the heating circuit, 255 = own program)
@return whether the operation has been correctly executed or not.
*/
boolean setDHWProgram(byte selProgram)
{
	boolean returnStatus = false;
	long timeout;

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID.
	outEMSBuffer[1] = RC35_ID;

	// third position is the message type
	outEMSBuffer[2] = UBA_WORKINGMODE_WW;

	// fourth position is the offset in the buffer. DHW Program is at position 4, so 4-4=0
	outEMSBuffer[3] = 0;

	// fifth position is the data to send
	if (selProgram != 255 && selProgram != 0) goto sendXML;
	else outEMSBuffer[4] = selProgram;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if (eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if (inEMSBuffer[0] == 0x01)
		{

			returnStatus = true;
		}
		else
		{
			returnStatus = false;
			goto sendXML;
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID.
	outEMSBuffer[1] = RC35_ID;

	// third position is the message type
	outEMSBuffer[2] = UBA_WORKINGMODE_WW;

	// fourth position is the offset in the buffer. DHW Program is at position 5, so 5-4=1
	outEMSBuffer[3] = 1;

	// fifth position is the data to send
	if (selProgram != 255 && selProgram != 0) goto sendXML;
	else outEMSBuffer[4] = selProgram;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if (eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if (inEMSBuffer[0] == 0x01)
		{
			returnStatus = true;
		}
		else
		{
			returnStatus = false;
			goto sendXML;
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;

	// third position is the message type
	outEMSBuffer[2] = UBA_WORKINGMODE_WW;

	// fourth position is the offset in the buffer. DHW Program is at position 4 and 5, so 4-4=0
	outEMSBuffer[3] = 0;

	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 2;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if (eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if (ptr > 4)
		{
			// check if the CRC of the information received is correct
			if (crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if (inEMSBuffer[2] == UBA_WORKINGMODE_WW)
				{
					wWProgram = (uint8_t)inEMSBuffer[4];
					wWPumpProgram = (uint8_t)inEMSBuffer[5];

					// check if the WWmode set corresponds with the demanded one
					if ((wWPumpProgram == selProgram) && (wWProgram == selProgram))
					{
						returnStatus = true;
					}
					else
					{
						returnStatus = false;
						goto sendXML;
					}
				}
				else
				{
					returnStatus = false;
					goto sendXML;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
sendXML:
	wifly.sendWifly(F("<SetWarmWatterWorkingMode>"));
	if (returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("WWProgram"), wWProgram);
		wifly.sendWiflyXML(F("WWPumpProgram"), wWPumpProgram);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWifly(F("</SetWarmWatterWorkingMode>"));

#if DEBUG
	eMSSerial.print(F("Parameter ")); eMSSerial.println(selProgram);
	eMSSerial.print(F("Warm Water Program: ")); eMSSerial.println(wWProgram);
	eMSSerial.print(F("Warm Water Pump Program: ")); eMSSerial.println(wWPumpProgram);
#endif

	return returnStatus;
}

/**
Send a EMS command to set the warm watter working mode (WW and Pump)
@selMode the desired mode (0-off, 1-on, 2-auto)
@return whether the operation has been correctly executed or not.
*/
boolean setDHWWorkingMode(byte selMode)
{

	boolean returnStatus = false;
	long timeout;

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID.
	outEMSBuffer[1] = RC35_ID;
	// third position is the message type
	outEMSBuffer[2] = UBA_WORKINGMODE_WW;
	// fourth position is the offset in the buffer. Warm Water Working Mode is position 6 - 4 = 2
	outEMSBuffer[3] = 2;
	// fifth position is the data to send
	if(selMode > 2 || selMode < 0) goto sendXML;
	else outEMSBuffer[4] = selMode;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if(inEMSBuffer[0] == 0x01)
		{
			returnStatus = true;
		}
		else 
		{
			returnStatus = false;
			goto sendXML;
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID;
	// third position is the message type
	outEMSBuffer[2] = UBA_WORKINGMODE_WW;
	// fourth position is the offset in the buffer. Warm Watter circulation Pump is 7 - 4 = 3
	outEMSBuffer[3] = 3;
	// fifth position is the data to be sent
	outEMSBuffer[4] = selMode;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if(inEMSBuffer[0] == 0x01)
		{
			returnStatus = true;
		}
		else
		{
			returnStatus = false;
			goto sendXML;
		}
	}
	
	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = RC35_ID | 0x80;
	// third position is the message type
	outEMSBuffer[2] = UBA_WORKINGMODE_WW;
	// fourth position is the offset in the buffer. Warm Water Working Mode is position 6 - 4 = 2 and Warm Watter circulation Pump is 7 - 4 = 3
	outEMSBuffer[3] = 2;
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 2;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_WORKINGMODE_WW)
				{
					wWWorkMod = (uint8_t)inEMSBuffer[4];
					wWPumpWorkMod = (uint8_t)inEMSBuffer[5];

					// check if the WWmode set corresponds with the demanded one
					if((wWWorkMod == selMode) && (wWPumpWorkMod == selMode))
					{
						returnStatus = true;
					}
					else
					{
						returnStatus = false;
						goto sendXML;
					}
				}
				else
				{
					returnStatus = true;
					goto sendXML;
				}
			}
		}
	}

	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
sendXML:
	wifly.sendWifly(F("<SetWarmWatterWorkingMode>"));
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("wWWorkMod"), wWWorkMod);
		wifly.sendWiflyXML(F("wWPumpWorkMod"), wWPumpWorkMod);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWifly(F("</SetWarmWatterWorkingMode>"));

#if DEBUG
	eMSSerial.print(F("Warm Water Working Mode: ")); eMSSerial.println(wWWorkMod);
	eMSSerial.print(F("Warm Water Pump Working Mode: ")); eMSSerial.println(wWPumpWorkMod);
#endif

	return returnStatus;
}

/**
Send a EMS command to set the warm watter one time function on or off
@selMode the desired mode (0-off, 1-on).
@return whether the operation has been correctly executed or not.
*/
boolean setDHWOneTime(byte selMode)
{

	boolean returnStatus = false;
	uint8_t wWOneTime;
	long timeout;
	int retries = MAX_RETRIES;

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID.
	outEMSBuffer[1] = MC10_ID;

	// third position is the message type
	outEMSBuffer[2] = UBA_FLAG_WW;

	// fifth position is the offset in the buffer. One Time Loading Warm Water is position 4 - 4 = 0
	outEMSBuffer[3] = 0;

	// fifth position is the data to send
	if(selMode == 0) outEMSBuffer[4] = WW_ONETIME_OFF;
	else if(selMode == 1) outEMSBuffer[4] = WW_ONETIME_ON;
	else goto sendXML;

retryOp:

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);
	retries--;
#if DEBUG
	eMSSerial.print(F("Retries: ")); eMSSerial.println(retries);
#endif

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if(inEMSBuffer[0] == 0x01)
		{
			returnStatus = true;
		}
		else
		{
			if(retries == 0)
			{
				returnStatus = false;
				goto sendXML; 
			}
			else
			{
				goto retryOp;
			}
		}
	}

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = MC10_ID | 0x80;

	// third position is the message type
	outEMSBuffer[2] = UBA_MONITOR_WW;

	// fourth position is the offset in the buffer. We are interested in DHW Once byte 9 - 4 = 5
	outEMSBuffer[3] = 5;

	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 1;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);
	retries--;

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if(eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if(ptr > 4)
		{
			// check if the CRC of the information received is correct
			if(crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if(inEMSBuffer[2] == UBA_MONITOR_WW)
				{
					wWOneTime = bitRead((uint8_t)inEMSBuffer[4], 1);
					
					if(wWOneTime == selMode)
					{
						returnStatus = true;
					}
					else
					{
						if(retries == 0)
						{
							returnStatus = false;
							goto sendXML;
						}
						else
						{
							goto retryOp;
						}
					}
				}
				else
				{
					if(retries > 0)
					{
						goto retryOp;
					}
					else
					{
						returnStatus = false;
						goto sendXML;
					}
				}
			}
		}
	}
	
	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
sendXML:
	wifly.sendWifly(F("<SetWarmWaterOneTime>"));
	if(returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("WWOneTime"), wWOneTime);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWifly(F("</SetWarmWaterOneTime>"));

#if DEBUG
	eMSSerial.print(F("One Time Loading: ")); eMSSerial.println(wWOneTime);
#endif

	return returnStatus;
}


/**
Send a request to EMS to edit a switch point in a program (switch points).
@selProgram the packet id of the user program to be read (0 - 0x38 - DHW, 1 - 0x39 - DHW Pump, 2 - 0x3F - HK1User1, 3 - 0x42 - HK1User2, 4 - 0x49 - HK2User1, 5 - 0x4C - HK2User2)
@switchPoint the switch point to be edited (from 0 to SWITCH_POINTS)
@byte operation to be performed: 0-OFF, 1-ON, 7-UNDEFINED
@byte day: 0-Monday, 6-Sunday
@hour hour: 0 to 23
@minute minute: 0 to 59
@return whether the operation has been correctly executed or not.
*/
boolean setProgramSwitchPoint(byte selProgram, byte switchPoint, byte operation, byte day, byte hour, byte minute)
{
	boolean returnStatus = false;
	byte byte1, byte2;

	switch (selProgram)
	{
		case 0:
			selProgram = 0x38;
			break;
		case 1:
			selProgram = 0x39;
			break;
		case 2:
			selProgram = 0x3F;
			break;
		case 3:
			selProgram = 0x42;
			break;
		case 4:
			selProgram = 0x49;
			break;
		case 5:
			selProgram = 0x4C;
			break;
		default: goto sendXML;
			break;
	}

	if (switchPoint >= SWITCHING_POINTS) goto sendXML;

	if (!((operation == 0) || (operation == 1) || (operation == 7))) goto sendXML;

	if (day > 6) goto sendXML;

	if (hour > 23) goto sendXML;

	if (minute > 59 || ((minute % 10)!=0)) goto sendXML;

	byte1 = (operation == 7) ? 0xE7 : (0x00 | (day << 5) | (operation));
	returnStatus = sendEMSWriteCommand(RC35_ID, selProgram, switchPoint * 2, byte1);

	byte2 = (operation == 7)? 0x90:((hour * 6) + (minute / 10));
	returnStatus &= sendEMSWriteCommand(RC35_ID, selProgram, (switchPoint * 2)+1, byte2);
	
	// send the XML response if the EMS operation returned correctly (tag Return contains RTC), send only tag Return (contains false) otherwise
sendXML:
	wifly.sendWifly(F("<setSwitchPointProgram>"));
	if (returnStatus)
	{
		lastEMSOperationRTC = wifly.getRTC();
		wifly.sendWiflyXML(F("SwitchPoint"), switchPoint);
		wifly.sendWiflyXML(F("Return"), lastEMSOperationRTC);
	}
	else
	{
		wifly.sendWiflyXML(F("Return"), returnStatus);
	}
	wifly.sendWifly(F("</setSwitchPointProgram>"));

#if DEBUG
	eMSSerial.print(F("ID: ")); eMSSerial.println(switchPoint, DEC);
	eMSSerial.print(F("Byte 1: ")); eMSSerial.println(byte1, BIN);
	eMSSerial.print(F("Byte 2: ")); eMSSerial.println(byte2, BIN);
	eMSSerial.print(F("Day: ")); eMSSerial.println((byte1 >> 5), DEC);
	eMSSerial.print(F("Hour: ")); eMSSerial.println((int)(byte2 / 6), DEC);
	eMSSerial.print(F("Minute: ")); eMSSerial.println((int)((byte2 % 6) * 10), DEC);
	eMSSerial.print(F("Action: ")); eMSSerial.println((byte1 & 7), DEC);
#endif

	return (returnStatus);
	
}

/**
Basic method to send an EMS write command with 1 byte and check if the operation has been correctly performed
@destinationID the destinationID
@messageType the message type
@offset the offset to apply
@data the data to be sent

@return whether the operation has been correctly executed or not.
*/
boolean sendEMSWriteCommand(byte destinationID, byte messageType, byte offset, byte data)
{
	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = PC_ID;

	// second position is destinationID.
	outEMSBuffer[1] = destinationID;

	// third position is the message type
	outEMSBuffer[2] = messageType;

	// fourth position is the offset in the buffer.
	outEMSBuffer[3] = offset;

	// fifth position is the data to send
	outEMSBuffer[4] = data;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if (eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if the answer received is 0x01, the value has been correctly sent
		if (inEMSBuffer[0] == 0x01)
		{
			returnStatus = true;
		}
		else
		{
			return false;
		}
	}
	
	// load outEMSBuffer with corresponding values.

	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = destinationID | 0x80;
	
	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 1;

	// once the buffer is loaded, send the request.
	sendRequest(outEMSBuffer);

	// check if the requested query is answered in the next EMSMaxAnswerTime milliseconds
	timeout = (long)millis() + EMSMaxAnswerTime;

	// wait until timeout or some new data in the EMS-Bus
	while ((((long)millis() - timeout) < 0) && (!eMSSerial3.available())) {}

	// if there is data to be read
	if (eMSSerial3.available())
	{
		// read the information sent
		int ptr = readBytes(inEMSBuffer);

		// if more than 4 bytes are read (datagram received)
		if (ptr > 4)
		{
			// check if the CRC of the information received is correct
			if (crcCheckOK(inEMSBuffer, ptr))
			{
				// check if the operation type returned corresponds with the one requested
				if (inEMSBuffer[2] == messageType)
				{
					// check if the data set corresponds with the set
					if ((data == (uint8_t)inEMSBuffer[4]))
					{
						returnStatus = true;
					}
					else
					{
						eMSSerial.println(F("Recieved: ")); eMSSerial.println((uint8_t)inEMSBuffer[4]);
						return false;
					}
				}
				else
				{
					return false;
				}
			}
		}
	}

	return returnStatus;
}

#pragma endregion EMSCommands

#pragma region WiFlyOperations

/**
Send the stats of Calduino in XML format via WiFly module
@mode 0=full data, 1 = basic data
@return whether the operation has been correctly executed or not.
*/
boolean getCalduinoStats(byte mode)
{

	char auxBuffer[20];
	uint32_t currentRTC = wifly.getRTC();


	// send the XML response
	wifly.sendWifly(F("<Calduino>"));
	if(mode == CALDUINO_FULL_STADISTICS)
	{
		wifly.sendWiflyXML(F("MAC"), wifly.getMAC(auxBuffer, sizeof(auxBuffer)));
		wifly.sendWiflyXML(F("IP"), wifly.getIP(auxBuffer, sizeof(auxBuffer)));
		wifly.sendWiflyXML(F("Gateway"), wifly.getGateway(auxBuffer, sizeof(auxBuffer)));
		wifly.sendWiflyXML(F("Netmask"), wifly.getNetmask(auxBuffer, sizeof(auxBuffer)));
		wifly.sendWiflyXML(F("SSID"), wifly.getSSID(auxBuffer, sizeof(auxBuffer)));
		wifly.sendWiflyXML(F("DeviceID"), wifly.getDeviceID(auxBuffer, sizeof(auxBuffer)));
		wifly.sendWiflyXML(F("FreeMemory"), wifly.getFreeMemory());
		wifly.sendWiflyXML(F("Time"), wifly.getTime(auxBuffer, sizeof(auxBuffer)));
		wifly.sendWiflyXML(F("Restarts"), wifly.getRestarts());
	}
	wifly.sendWiflyXML(F("ArdUpTime"), (uint32_t)(currentRTC - startedArduinoRTC)); //0
	wifly.sendWiflyXML(F("WiFlyUpTime"), wifly.getUptime());                        //1
	wifly.sendWiflyXML(F("SecLastEMSOp"), (uint32_t)(currentRTC - lastEMSOperationRTC));//2
	wifly.sendWiflyXML(F("OpRec"), operationsReceived - 1); //3
	wifly.sendWiflyXML(F("OpOK"), operationsOK);//4
	wifly.sendWiflyXML(F("OpNOK"), operationsNOK);//5
	wifly.sendWiflyXML(F("RTC"), currentRTC);//6
	wifly.sendWifly(F("</Calduino>"));

	return true;

}

/**
Configure WiFly  with the standard parameters and the TimeZone passed as parameter.

@reqNtpTimeZone Requested Time Zone to be configured (Winter = 0 - 23 / Summer = 1 - 22). 
@return Whether WiFly is joined to the WiFi connection configured or not.
*/
boolean configureWifly(uint8_t reqNtpTimeZone)
{

	if(reqNtpTimeZone == 1) nTPTimeZone = SUMMER_TIMEZONE;
	else if(reqNtpTimeZone == 0) nTPTimeZone = WINTER_TIMEZONE;
	else
	{
		#if DEBUG
			eMSSerial.println(F("Error configureWifly: Incorrect parameter."));
		#endif
		return false;
	}

	// send the configuration information and restart the WiFly module
	wifly.getConnection();
	wifly.close();

	wifly.leave();
	wifly.setJoin(WIFLY_WLAN_JOIN_AUTO);
	wifly.setDHCP(WIFLY_DHCP_MODE_CACHE);
	wifly.setDeviceID(myDeviceID);
	wifly.setSSID(mySSID);
	wifly.setPassphrase(myPassword);
	wifly.setIdleTime(IDLE_TIME);
	wifly.setTimeAddress(nTPServer);
	wifly.setTimePort(nTPServerPort);
	wifly.setTimezone(nTPTimeZone);
	wifly.setTimeEnable(NTP_UPDATE_TIME);
	wifly.save();
	wifly.reboot();
	
	// wait time until WiFly reboots
	delay(MAIN_LOOP_WAIT_TIME * 5);

	boolean joined = wifly.isAssociated();
	if(joined)
	{
		wifly.time();

		// wait max. 1000 seconds until RTC time is received
		while(wifly.getRTC() < MAIN_LOOP_WAIT_TIME)
		{
		#if DEBUG
			eMSSerial.println(F("ConfigureWifly: Waiting to get time."));
		#endif
			wifly.time();
			delay(MAIN_LOOP_WAIT_TIME);
		}
	#if DEBUG
		eMSSerial.println(F("ConfigureWifly: WiFly  joined and time configured."));
	#endif
	}
	else
	{
	#if DEBUG
		eMSSerial.println(F("Error ConfigureWifly: Not joined."));
	#endif
	}
	return joined;

}

/**
Reset the internal statistics of Calduino.

@return true.
*/
boolean resetCalduinoStadistics()
{
	// restart counters 
	operationsReceived = 1;
	operationsOK = 0;
	operationsNOK = 0;

	uint32_t currentRTC = wifly.getRTC();

	// initialize the RTC counters
	startedArduinoRTC = lastEMSOperationRTC = currentRTC;

	wifly.sendWifly(F("<Calduino>"));
	wifly.sendWiflyXML(F("Return"), currentRTC);
	wifly.sendWifly(F("</Calduino>"));

	return true;
}


/**
Changes the EMS_BUS timeout
@return true.
*/
boolean setEMSTimeout(uint8_t timeout)
{
	EMSMaxAnswerTime = timeout * 100;
	uint32_t currentRTC = wifly.getRTC();

	wifly.sendWifly(F("<Calduino>"));
	wifly.sendWiflyXML(F("EMSTimeout"), (int) EMSMaxAnswerTime);
	wifly.sendWiflyXML(F("Return"), currentRTC);
	wifly.sendWifly(F("</Calduino>"));

#if DEBUG
	eMSSerial.print(F("SetEMSTimeout: Value desired:")); eMSSerial.println(timeout);
	eMSSerial.print(F("SetEMSTimeout: Value set:")); eMSSerial.println(EMSMaxAnswerTime);
#endif

	return true;
}

/**
Restart the WiFly module.

@return Whether WiFly is joined to the standard WiFi connection configured or not.
*/
boolean restartWifly()
{

	// close any possible connection before restarting. 
	wifly.getConnection();
	wifly.close();
	eMSSerial2.flush();

#if DEBUG
	eMSSerial.println(F("RestartWifly: Restarting WiFly "));
#endif

	digitalWrite(RESET_PIN, LOW);
	delay(MAIN_LOOP_WAIT_TIME);
	digitalWrite(RESET_PIN, HIGH);

#if DEBUG
	// initialize WiFly module with debug mode activated
	eMSSerial.begin(WIFLY_SPEED);
	wifly.begin(&eMSSerial2, &eMSSerial);
#else
	wifly.begin(&eMSSerial2);
#endif

	// wait time until the unit is restarted
	delay(MAIN_LOOP_WAIT_TIME * 5);

	boolean joined = wifly.isAssociated();
	if(joined)
	{
		wifly.time();

		//Wait max. 1000 seconds until RTC time is received
		while(wifly.getRTC() < MAIN_LOOP_WAIT_TIME)
		{
		#if DEBUG
			eMSSerial.println(F("RestartWifly: Waiting to get time."));
		#endif
			wifly.time();
			delay(MAIN_LOOP_WAIT_TIME);
		}
	#if DEBUG
		eMSSerial.println(F("RestartWifly: WiFly  joined and time configured."));
	#endif
	}
	else
	{
	#if DEBUG
		eMSSerial.println(F("Error RestartWifly: Not joined."));
	#endif
	}
	return wifly.isAssociated();

}

/**
Setup the WiFly module.

@return Whether WiFly is joined to the standard WiFi connection configured or not.
*/
boolean setupWifly()
{
	boolean wiflyBegin;
	boolean wiflyAssociated;

#if DEBUG
	// initialize WiFly module with debug mode activated
	eMSSerial.begin(WIFLY_SPEED);
	wiflyBegin = wifly.begin(&eMSSerial2, &eMSSerial);
#else
	wiflyBegin = wifly.begin(&eMSSerial2);
#endif

	wiflyAssociated = wifly.isAssociated();

	while(!wiflyAssociated)
	{
		boolean wiflyRestarted = false;

		while(!wiflyRestarted)
		{
			wiflyRestarted = restartWifly();
		#if DEBUG
			if (!wiflyRestarted) eMSSerial.println(F("Error SetupWifly: Failed at restarting WiFly ."));
		#endif
		}

		wiflyAssociated = configureWifly(nTPTimeZone);

		if(!wiflyAssociated)
		{
		#if DEBUG
			eMSSerial.println("Error SetupWifly: Failed at configuring WiFly ");
		#endif

			// Wait a few seconds and try the operation again
			delay(MAIN_LOOP_WAIT_TIME * 5);
		}
	}

	wifly.time();

	//Wait max. 1000 seconds until RTC time is received. When NTP is not synchronized, getRTC returns WiflyUptime
	while(wifly.getRTC() < MAIN_LOOP_WAIT_TIME)
	{
	#if DEBUG
		eMSSerial.println(F("SetupWifly: Waiting to get time."));
	#endif
		wifly.time();
		delay(MAIN_LOOP_WAIT_TIME);
	}

#if DEBUG
	eMSSerial.println(F("SetupWifly: WiFly  joined and time configured."));
#endif

}

/**
Read the HTTP Request received. The operation requested will be stored in the global variable operationRequested.

@return the operation read in the HTTP Request, or NO_OPERATION if nothing is found 
*/
byte readHTTPRequest()
{

	// pointer for vector
	char index = 0;

	// initialize server
	memset(httpBuffer, 0, HTTP_BUFFER_SIZE);	

	if(wifly.available() > 0)
	{
		wifly.gets(httpBuffer, sizeof(httpBuffer));
	}

#ifdef DEBUG
	eMSSerial.print(F("httpBuffer: "));
	eMSSerial.println(httpBuffer);
#endif

	//Search the structure ?op= and get the two following digits
	byte operationRequested = getParameterFromHTTP("?op=", 2);

#ifdef DEBUG
	eMSSerial.print(F("op: "));
	eMSSerial.println(operationRequested);
#endif
	
	return operationRequested;

}

/**
Search for a parameter in the HTTP Buffer received and return the number of digits requested to be read.

@searchedString parameter searched.
@parameterLength number of digits to be read.

@return parameter requested, NO_OPERATION if not found.
*/
byte getParameterFromHTTP(char* searchedString, uint8_t parameterLength)
{
	//Pointer to NULL
	char *auxP = 0;

	//Search the String
	auxP = strstr(httpBuffer, searchedString);

	//Variable to return by default
	byte parameter = NO_OPERATION;

	if(auxP != 0)
	{
		//put the pointer at the start of the parameter to read
		auxP += strlen(searchedString);
		parameter = 0;

		// get the "parameterLength" digits and return the parameter
		for(; parameterLength > 0; parameterLength--)
		{
			parameter = parameter * 10 + *auxP - '0';
			auxP++;
		}
	}

	return parameter;

}

#pragma endregion WiflyOperations

/**
Executes the operation requested via HTTP.

@operationRequested code of the operation requested.

@return whether the operation has been correctly executed or not.
*/
boolean executeOperation(byte operationRequested)
{

	boolean returnStatus = false;
	resetBuffer(inEMSBuffer);

	switch(operationRequested)
	{
		case GET_ALL_MONITORS:
		#if DEBUG
			eMSSerial.println(F("Get All Monitors"));
		#endif
			returnStatus = getAllMonitors();
			break;

		case GET_RCTIME:
		#if DEBUG
			eMSSerial.println(F("Get RCTime"));
		#endif
			returnStatus = getRCTime();
			break;

		case GET_UBA_MONITOR:
		#if DEBUG
			eMSSerial.println(F("Get Universal Automatic Burner Monitor"));
		#endif
			returnStatus = getUBAMonitor();
			break;

		case GET_UBA_MONITOR_WW:
		#if DEBUG
			eMSSerial.println(F("Get Universal Automatic Burner Monitor Warm Water"));
		#endif
			returnStatus = getUBAMonitorWarmWater();
			break;

		case GET_RC35_MONITOR_HC1:
		#if DEBUG
			eMSSerial.println(F("Get RC35 Heating Circuit 1"));
		#endif
			returnStatus = getRC35MonitorHeatingCircuit(1);
			break;

		case GET_RC35_MONITOR_HC2:
		#if DEBUG
			eMSSerial.println(F("Get RC35 Heating Circuit 2"));
		#endif
			returnStatus = getRC35MonitorHeatingCircuit(2);
			break;

		case GET_GENERAL_MONITOR:
		#if DEBUG
			eMSSerial.println(F("Get General Monitor"));
		#endif
			returnStatus = getGeneralMonitor(false);
			break;

		case GET_PROGRAM:			 
		#if DEBUG
			eMSSerial.println(F("Get Program:")); eMSSerial.print(getParameterFromHTTP("?pr=", 1));
		#endif
			returnStatus = getProgramSwitchPoints(getParameterFromHTTP("?pr=", 1));
			break;

		case GET_CALDUINO_BASIC:
		#if DEBUG
			eMSSerial.println(F("Get Basic Calduino Statistics"));
		#endif
			returnStatus = getCalduinoStats(CALDUINO_BASIC_STADISTICS);
			break;

		case GET_CALDUINO_FULL:
		#if DEBUG
			eMSSerial.println(F("Get Full Calduino Statistics"));
		#endif
			returnStatus = getCalduinoStats(CALDUINO_FULL_STADISTICS);
			break;

		case RESET_CALDUINO_STATS:
		#if DEBUG
			eMSSerial.println(F("Reset Calduino Statistics"));
		#endif
			returnStatus = resetCalduinoStadistics();
			break;

		case SET_EMS_TIMEOUT:
		#if DEBUG
			eMSSerial.println(F("Set EMS Timeout"));
		#endif
			returnStatus = setEMSTimeout(getParameterFromHTTP("?to=", 2));
			break;

		case SET_RC35_WORKING_MODE:
		#if DEBUG
			eMSSerial.print(F("Set RC35 Circuit ")); eMSSerial.print(getParameterFromHTTP("?hc=", 1)); eMSSerial.print(F(" to working Mode (0-night, 1-day, 2-auto) ")); eMSSerial.println(getParameterFromHTTP("?wm=", 1));
		#endif
			returnStatus = setRC35WorkingMode(getParameterFromHTTP("?hc=", 1), getParameterFromHTTP("?wm=", 1));
			break;

		case SET_RC35_TEMPERATURE:
		#if DEBUG
			eMSSerial.print(F("Set RC35 Circuit ")); eMSSerial.print(getParameterFromHTTP("?hc=", 1)); eMSSerial.print(F(" for Mode (0-night, 1-day, 2-holiday) ")); eMSSerial.print(getParameterFromHTTP("?wm=", 1)); eMSSerial.print(F(" to temperature ")); eMSSerial.println(getParameterFromHTTP("?tp=", 2)/2);
		#endif
			returnStatus = setRC35SelectedTemperature(getParameterFromHTTP("?hc=", 1), getParameterFromHTTP("?wm=", 1), getParameterFromHTTP("?tp=", 2));
			break;

		case SET_WW_TEMPERATURE:
		#if DEBUG
			eMSSerial.print(F("Set warm watter temperature to ")); eMSSerial.println(getParameterFromHTTP("?tp=", 2));
		#endif
			returnStatus = setDHWTemperature(getParameterFromHTTP("?tp=", 2));
			break;

		case SET_WW_WORKING_MODE:
		#if DEBUG
			eMSSerial.print(F("Set RC35 warm water working mode (0-off, 1-on, 2-auto) ")); eMSSerial.println(getParameterFromHTTP("?wm=", 1));
		#endif
			returnStatus = setDHWWorkingMode(getParameterFromHTTP("?wm=", 1));
			break;

		case SET_WW_ONE_TIME:
		#if DEBUG
			eMSSerial.print(F("Set RC35 warm watter one time (0-off, 1-on)")); eMSSerial.println(getParameterFromHTTP("?wm=", 1));
		#endif
			returnStatus = setDHWOneTime(getParameterFromHTTP("?wm=", 1));
			break;

		case SET_RC35_PROGRAM:
		#if DEBUG
			eMSSerial.print(F("Set RC35 Circuit ")); eMSSerial.print(getParameterFromHTTP("?hc=", 1)); eMSSerial.print(F(" to Program 0x00 = User 1, 0x01 = Family, 0x02 = Morning, 0x03 = Early morning, 0x04 = Evening, 0x05 = Midmorning, 0x06 = Afternoon, 0x07 = Midday, 0x08 = Single, 0x09 = Senioren, 0x0A User2) to: ")); eMSSerial.println(getParameterFromHTTP("?pr=", 2));
		#endif
			returnStatus = setRC35Program(getParameterFromHTTP("?hc=", 1), getParameterFromHTTP("?pr=", 2));
		break;
		
		case SET_WW_PROGRAM:
		#if DEBUG
			eMSSerial.print(F("Set RC35 warm water program (0 = like the heating circuit, 255 = own program) to: ")); eMSSerial.println(getParameterFromHTTP("?pr=", 3));
		#endif
			returnStatus = setDHWProgram(getParameterFromHTTP("?pr=", 3));
		break;

		case SET_SWITCH_POINT:
		#if DEBUG
			eMSSerial.print(F("Set Switch Point for program: ")); eMSSerial.print(getParameterFromHTTP("?pr=", 1)); eMSSerial.print(F(" point number: ")); eMSSerial.print(getParameterFromHTTP("?sp=", 2)); eMSSerial.print(F(" to: ")); eMSSerial.print(getParameterFromHTTP("?on=", 1)); eMSSerial.print(F(" day: ")); eMSSerial.print(getParameterFromHTTP("?d=", 1)); eMSSerial.print(F(" hour: ")); eMSSerial.print(getParameterFromHTTP("?h=", 2)); eMSSerial.print(F(" minute: ")); eMSSerial.print(getParameterFromHTTP("?m=", 2));
		#endif
			returnStatus = setProgramSwitchPoint(getParameterFromHTTP("?pr=", 1), getParameterFromHTTP("?sp=", 2), getParameterFromHTTP("?on=", 1), getParameterFromHTTP("?d=", 1), getParameterFromHTTP("?h=", 2), getParameterFromHTTP("?m=", 2));
		break;

		case REBOOT_CALDUINO:
		#if DEBUG
			eMSSerial.print(F("Rebooting Calduino"));
		#endif
			returnStatus = restartWifly();
			break;

		case RECONFIGURE_CALDUINO:
		#if DEBUG
			eMSSerial.print(F("Configuring WiFly  (0 winter - 1 summer):")); eMSSerial.println(getParameterFromHTTP("?tz=", 1));
		#endif
			returnStatus = configureWifly(getParameterFromHTTP("?tz=", 1));
			break;

		case NO_OPERATION:
		#if DEBUG
			eMSSerial.println(F("Incorrect operation received. Closing connection"));
		#endif
			returnStatus = false;
			break;
	}

	if(returnStatus)
	{
		operationsOK++;

	}
	else
	{
		operationsNOK++;
	}

	if(operationRequested != NO_OPERATION)
	{
		// rearm timeout
		wiflyTimeout = (uint32_t)millis() + WIFLY_TIMEOUT;

	}

	return returnStatus;

}

/**
Check if there has not been communication with the WiFly module since a predetermined timeout.
If so, check if the WiFly is already associated. If not associated, try to restart and configure it.
Will not end until a connection is reached.

@operationRequested code of the operation requested.

@return whether the operation has been correctly executed or not.
*/
void checkWiFlyTimeout()
{
	// check if wiflyTimeout has expired. If so, test the connection and communication with wily
	if((uint32_t)millis() > wiflyTimeout)
	{
	#if DEBUG
		eMSSerial.println(F("CheckWiFlyTimeout: Checking WiFly association."));
	#endif

		if(!wifly.isAssociated())
		{
		#if DEBUG
			eMSSerial.println(F("CheckWiFlyTimeout: WiFly not associated! Restarting..."));
		#endif
			if(!configureWifly(SUMMER_TIME))
			{
				setupWifly();
			}
		}
		else
		{
			// rearm timeout (watchdog)
			wiflyTimeout = (uint32_t)millis() + WIFLY_TIMEOUT;
		#if DEBUG
			eMSSerial.println(F("CheckWiFlyTimeout: WiFly associated."));
		#endif
		}
	}
}

void setup()
{

	// reset pin is HIGH. Active when low
	pinMode(RESET_PIN, OUTPUT);
	digitalWrite(RESET_PIN, HIGH);

	// start the serial connection with the EMS BUS at EMS_BUS_SPEED bauds (9700)
	eMSSerial3.begin(EMS_BUS_SPEED);

	// start the serial connection with the WiFly module at WIFLY_SPEED bauds (9600)
	eMSSerial2.begin(WIFLY_SPEED);

	// initialize WiFly module
	setupWifly();

	// initialize all variables
	//initialiceEMS();

	// initialize the RTC counters
	startedArduinoRTC = lastEMSOperationRTC = wifly.getRTC();

	// set the WiflyTimeout to restart it if communication fails.
	wiflyTimeout = (uint32_t)millis() + WIFLY_TIMEOUT;

#if DEBUG
	eMSSerial.println(F("Calduino Correctly Started."));
#endif
}

void loop()
{

	byte operationRequested = NO_OPERATION;
	uint8_t available = wifly.available();
	boolean connected = wifly.isConnected();

	//Check if there is a HTTP Request ready to be read
	if(available && connected)
	{
		//Read the HTTP Request and extract the requested operation
		operationRequested = readHTTPRequest();
		operationsReceived++;
		boolean operationReturn = executeOperation(operationRequested);
	
	#if DEBUG
		eMSSerial.print(F("Loop: Operation returned:"));
		eMSSerial.println(operationReturn);
	#endif
	}

	wifly.forceClose();	
	checkWiFlyTimeout();
	delay(MAIN_LOOP_WAIT_TIME);

}
