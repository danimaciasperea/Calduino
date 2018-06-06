/*
* Copyright (c) 2018 Daniel Macías Perea (dani.macias.perea@gmail.com)
*   
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
* 
* Legal Notices
* 'Bosch Group', 'Buderus', 'Nefit' and 'Worcester' are brands of Bosch Thermotechnology.
* All other trademarks are the property of their respective owners.
*/

/* Release history
*
* Version  Date         Description
* 0.1      25-Apr-2018  First release.
* 0.2	   22-May-2018  Operation to set Thermal Disinfection splitted in two. 
*						Retry factor adapted to minimize errors.
* 0.3      05-Jun-2018  Corrected UBAMonitorDHW Datagram
*/

/**
* @mainpage EMS Bus - Arduino library.
*
* This library provides functions to communicate through the EMS Bus with Buderus/Nefit/Worcester
* (or any other EMS Bus compatible) boilers. It includes commands for both getting status information
* (UBA Monitor, DHW Monitor, etc.) and setting new configurations (Set Day/Night Temperature,
* Set Working Mode, etc.).
*
* @author	dani.macias.perea@gmail.com
*/

/**
* @file Calduino.h
*
* @brief The Calduino class definition including EMSSerial, CalduinoDebug, CalduinoSerial, Calduino Datagrams and Calduino Datas.
*/


#ifndef Calduino_h
#define Calduino_h

#define SERIAL_BUFFER_SIZE 48
#define MAX_EMS_READ 32

#if MAX_EMS_READ > SERIAL_BUFFER_SIZE
# MAX_EMS_READ = SERIAL_BUFFER_SIZE
#endif


#define ERROR_VALUE 0xFF
#define HEATING_CIRCUITS 2

#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))

typedef const PROGMEM char prog_char;

/* EMSSerial declaration */
#pragma region EMSSERIAL

/**
 * Hardware Serial class adapted to EMS Buffer characteristics. There is only a reception
 * buffer. WriteEOF will disable reception and change UART parity to send without errors an 11
 * bits 0 chain. Flush operation will disable and enable reception to erase the buffer.
 */

class EMSSerial : public Stream {
protected:
	volatile uint8_t *_ubrrh;
	volatile uint8_t *_ubrrl;
	volatile uint8_t *_ucsra;
	volatile uint8_t *_ucsrb;
	volatile uint8_t *_ucsrc;
	volatile uint8_t *_udr;
	uint8_t _rxen;
	uint8_t _txen;
	uint8_t _rxcie;
	uint8_t _error;
	bool _written;

public:
	volatile uint8_t _rx_buffer_head;
	volatile uint8_t _rx_buffer_tail;

	unsigned char _rx_buffer[SERIAL_BUFFER_SIZE];
	bool _error_flag[SERIAL_BUFFER_SIZE];

	EMSSerial(
		volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
		volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
		volatile uint8_t *ucsrc, volatile uint8_t *udr,
		uint8_t rxen, uint8_t txen, uint8_t rxcie);
	EMSSerial();
	bool begin(unsigned long baud);
	void end();
	void writeEOF();
	virtual int available(void);
	bool frameError() { bool ret = _error; 	_error = false;  return ret; }
	virtual int peek(void);
	virtual int read(void);
	virtual void flush(void);
	virtual size_t write(uint8_t);
	inline size_t write(unsigned long n) { return write((uint8_t)n); }
	inline size_t write(long n) { return write((uint8_t)n); }
	inline size_t write(unsigned int n) { return write((uint8_t)n); }
	inline size_t write(int n) { return write((uint8_t)n); }
	//using Print::write; // pull in write(str) and write(buf, size) from Print
	operator bool();
};

#if defined(UBRRH) || defined(UBRR0H)
extern EMSSerial EMSSerial0;
#endif
#if defined(UBRR1H)
extern EMSSerial EMSSerial1;
#endif
#if defined(UBRR2H)
extern EMSSerial EMSSerial2;
#endif
#if defined(UBRR3H) 
extern EMSSerial EMSSerial3;
#endif

#pragma endregion EMSSerial

/* CalduinoData declaration */
#pragma region CalduinoData

/**
 * Enumeration of Calduino Units. It should match the calduinoUnits array, which contains a
 * flash string for each unit.
 */

 enum CalduinoUnit {
	None,
	Celsius,
	YesNo,
	MAmper,
	Bar,
	Minute,
	Times,
	Percentage,
	Seconds
};


/**
 * Enumeration that represent calduino encode types. Each enconde type will parse the bytes contained
 * in the EMS Datagram in a different way.
 */

 enum CalduinoEncodeType {
	Byte,
	Bit,
	Float,
	ULong,
	SwithPoint
};


/**
 * Print Format enumeration.
 * - Standard prints values and units.  
 * - XML surrounds values with name tags.  
 * - NoUnit prints only values.
 */

 enum PrintFormat {
	Standard,
	NoUnit,
	XML
};


/**
 * Switch point struct definition.
 * - Id is the identification of the Switch Point.
 * - Action is the operation performed (0 - off/night, 1 - on/day, 7 - undefined).
 * - Point in time (day from 0 - monday to 6 - sunday,
 * hours from 0 to 23 and minute from 0 to 50, in 10 minutes increments).
 */

typedef struct SwitchPoint {
	byte id;
	byte action;
	byte day;
	byte hour;
	byte minute;
};


/**
 * Calduino data struct definition.
 * - Name is the ID of the data.  
 * - Encode Type to use by parsing from the EMS Buffer.  
 * - Unit of the data.  
 * - Offset is the position that the data occupies in the EMS Buffer.  
 * - Bit Offset is only used for bit types.  
 * - Float bytes is only used for float types. It is the number of bytes represented by the
 * value in the EMS Buffer.  
 * - Float factor is only used for float types. Floats are coded multiplied by a factor (2 or
 * 10) to avoid decimals in the EMS Buffer. To decode a float it should by diveded by this
 * factor.
 */

struct CalduinoData {
	prog_char* dataName;
	CalduinoEncodeType encodeType;
	CalduinoUnit unit;
	byte offset;
	byte bitOffset;
	byte floatBytes;
	byte floatFactor;

	byte decodeByteValue(byte* inEMSBuffer);
	bool decodeBitValue(byte* inEMSBuffer);
	unsigned long decodeULongValue(byte* inEMSBuffer);
	SwitchPoint decodeSwitchPoint(byte *inEMSBuffer);
	float decodeFloatValue(byte* inEMSBuffer);
	void decodeValue(byte* inEMSBuffer, char* value);
	void printfValue(char* str, char* value, PrintFormat printFormat);
};

#pragma endregion CalduinoData

/* EMSDatagram declaration */
#pragma region EMSDatagram

/* Enumeration of EMS Datagram. It matches the eMSDatagramIDs array. */
typedef enum EMSDatagramID {
	RC_Datetime,
	UBA_Working_Time,
	UBA_Monitor_Fast,
	UBA_Monitor_Slow,
	UBA_Parameter_DHW,
	UBA_Monitor_DHW,
	Flags_DHW,
	Working_Mode_DHW,
	Program_DHW,
	Program_Pump_DHW,
	Working_Mode_HC_1,
	Monitor_HC_1,
	Program_1_HC_1,
	Program_2_HC_1,
	Working_Mode_HC_2,
	Monitor_HC_2,
	Program_1_HC_2,
	Program_2_HC_2,
	Working_Mode_HC_3,
	Monitor_HC_3,
	Program_1_HC_3,
	Program_2_HC_3,
	Working_Mode_HC_4,
	Monitor_HC_4,
	Program_1_HC_4,
	Program_2_HC_4,
	Monitor_MM_10
};

/** Enumeration of Message Identifiers in the EMS Bus. */
enum MessageID {
	RC_Datetime_ID = 0x06,
	UBA_Working_Time_ID = 0x14,
	UBA_Monitor_Fast_ID = 0x18,
	UBA_Monitor_Slow_ID = 0x19,
	UBA_Parameter_DHW_ID = 0X33,
	UBA_Monitor_DHW_ID = 0x34,
	Flags_DHW_ID = 0x35,
	Working_Mode_DHW_ID = 0X37,
	Program_DHW_ID = 0X38,
	Program_Pump_DHW_ID = 0X39,
	Working_Mode_HC_1_ID = 0x3D,
	Monitor_HC_1_ID = 0x3E,
	Program_1_HC_1_ID = 0x3F,
	Program_2_HC_1_ID = 0x42,
	Working_Mode_HC_2_ID = 0x47,
	Monitor_HC_2_ID = 0x48,
	Program_1_HC_2_ID = 0x49,
	Program_2_HC_2_ID = 0x4C,
	Working_Mode_HC_3_ID = 0x51,
	Monitor_HC_3_ID = 0x52,
	Program_1_HC_3_ID = 0x53,
	Program_2_HC_3_ID = 0x56,
	Working_Mode_HC_4_ID = 0x5B,
	Monitor_HC_4_ID = 0x5C,
	Program_1_HC_4_ID = 0x5D,
	Program_2_HC_4_ID = 0x60,
	Monitor_MM_10_ID = 0xAB
};

/** Enumeration of Device Identifiers in the EMS Bus. */
enum DeviceID {
	UBA = 0x08,
	BC_10 = 0x09,
	PC = 0x0B,
	RC_35 = 0x10,
	WM_10 = 0x11,
	RC_20 = 0x17,
	MM_10 = 0x21
};


/**
* Enumeration that contains the position that each Calduino Data occupies inside the data array
* of the EMS Datagram. This enumeration is shared by all the Datagrams, so there will be
* repeated values.
*/

enum DatagramDataIndex {
	yearIdx,
	monthIdx,
	dayIdx,
	hourIdx,
	minuteIdx,
	secondIdx,
	uBAWorkingMinIdx = 0,
	selImpTempIdx = 0,
	curImpTempIdx,
	selBurnPowIdx,
	curBurnPowIdx,
	burnGasIdx,
	fanWorkIdx,
	ignWorkIdx,
	heatPmpIdx,
	threeWayValveDHWIdx,
	circDHWIdx,
	retTempIdx,
	flameCurrIdx,
	sysPressIdx,
	srvCode1Idx,
	srvCode2Idx,
	errCodeIdx,
	extTempIdx = 0,
	boilTempIdx,
	pumpModIdx,
	burnStartsIdx,
	burnWorkMinIdx,
	burnWorkMinHIdx,
	selTempDHWIdx = 0,
	tempTDDHWIdx,
	curTempDHWIdx = 0,
	dayModeDHWIdx,
	oneTimeDHW1Idx,
	desDHWIdx,
	prepareDHWIdx,
	burnStartsDHWIdx,
	burnWorkMinDHWIdx,
	oneTimeDHW2Idx = 0,
	progDHWIdx = 0,
	progPumpDHWIdx,
	workModeDHWIdx,
	workModePumpDHWIdx,
	workModeTDDHWIdx,
	dayTDDHWIdx,
	hourTDDHWIdx,
	selNightTempHCIdx = 0,
	selDayTempHCIdx,
	selHoliTempHCIdx,
	roomTempInfHCIdx,
	roomTempOffHCIdx,
	workModeHCIdx,
	sWThresTempHCIdx,
	nightSetbackHCIdx,
	nightOutTempHCIdx,
	holiModHCIdx = 0,
	summerModHCIdx,
	dayModHCIdx,
	pauseModHCIdx,
	selRoomTempHCIdx,
	programNameIdx = 42,
	pauseTimeIdx,
	partyTimeIdx,
	startHolidayDayIdx,
	startHolidayMonthIdx,
	startHolidayYearIdx,
	endHolidayDayIdx,
	endHolidayMonthIdx,
	endHolidayYearIdx,
	startHomeHolidayDayIdx,
	startHomeHolidayMonthIdx,
	startHomeHolidayYearIdx,
	endHomeHolidayDayIdx,
	endHomeHolidayMonthIdx,
	endHomeHolidayYearIdx,
	selImpTempMM10Idx = 0,
	curImpTempMM10Idx,
	statusMM10Idx
};


/**
 * Enumeration containing all the available Calduino Data of type Byte. It matches byteRequests
 * array.
 */

 enum ByteRequest {
	year_b,
	month_b,
	day_b,
	hour_b,
	minute_b,
	second_b,
	selImpTemp_b,
	selBurnPow_b,
	curBurnPow_b,
	srvCode1_b,
	srvCode2_b,
	pumpMod_b,
	selTempDHW_b,
	tempTDDHW_b,
	oneTimeDHW2_b,
	progDHW_b,
	progPumpDHW_b,
	workModeDHW_b,
	workModePumpDHW_b,
	dayTDDHW_b,
	hourTDDHW_b,
	workModeHC1_b,
	sWThresTempHC1_b,
	nightSetbackHC1_b,
	workModeHC2_b,
	sWThresTempHC2_b,
	nightSetbackHC2_b,
	workModeHC3_b,
	sWThresTempHC3_b,
	nightSetbackHC3_b,
	workModeHC4_b,
	sWThresTempHC4_b,
	nightSetbackHC4_b,
	programNameHC1_b,
	pauseTimeHC1_b,
	partyTimeHC1_b,
	programNameHC2_b,
	pauseTimeHC2_b,
	partyTimeHC2_b,
	programNameHC3_b,
	pauseTimeHC3_b,
	partyTimeHC3_b,
	programNameHC4_b,
	pauseTimeHC4_b,
	partyTimeHC4_b,
};


/**
 * Enumeration containing all the available Calduino Data of type Float. It matches FloatRequests
 * array.
 */

 enum FloatRequest {
	curImpTemp_f,
	retTemp_f,
	flameCurr_f,
	sysPress_f,
	errCode_f,
	extTemp_f,
	boilTemp_f,
	curTempDHW_f,
	selNightTempHC1_f,
	selDayTempHC1_f,
	selHoliTempHC1_f,
	roomTempInfHC1_f,
	roomTempOffHC1_f,
	nightOutTempHC1_f,
	selNightTempHC2_f,
	selDayTempHC2_f,
	selHoliTempHC2_f,
	roomTempInfHC2_f,
	roomTempOffHC2_f,
	nightOutTempHC2_f,
	selNightTempHC3_f,
	selDayTempHC3_f,
	selHoliTempHC3_f,
	roomTempInfHC3_f,
	roomTempOffHC3_f,
	nightOutTempHC3_f,
	selNightTempHC4_f,
	selDayTempHC4_f,
	selHoliTempHC4_f,
	roomTempInfHC4_f,
	roomTempOffHC4_f,
	nightOutTempHC4_f,
	selRoomTempHC1_f,
	selRoomTempHC2_f,
	selRoomTempHC3_f,
	selRoomTempHC4_f,
	curImpTempMM10_f,
};


/**
 * Enumeration containing all the available Calduino Data of type ULong. It matches
 * ULongRequests array.
 */

 enum ULongRequest {
	uBAWorkingMin_ul,
	burnStarts_ul,
	burnWorkMin_ul,
	burnWorkMinH_ul,
	burnStartsDHW_ul,
	burnWorkMinDHW_ul,
};


/**
 * Enumeration containing all the available Calduino Data of type Bit. It matches BitRequests
 * array.
 */

 enum BitRequest {
		burnGas_t,
		fanWork_t,
		ignWork_t,
		heatPmp_t,
		threeWayValveDHW_t,
		circDHW_t,
		dayModeDHW_t,
		oneTimeDHW_t,
		desDHW_t,
		prepareDHW_t,
		holiModHC1_t,
		summerModHC1_t,
		dayModHC1_t,
		pauseModHC1_t,
		holiModHC2_t,
		summerModHC2_t,
		dayModHC2_t,
		pauseModHC2_t,
		holiModHC3_t,
		summerModHC3_t,
		dayModHC3_t,
		pauseModHC3_t,
		holiModHC4_t,
		summerModHC4_t,
		dayModHC4_t,
		pauseModHC4_t
};

/**
 * EMS Datagram struct definition. Each datagram contains:
 * - Name in Flash string (for printing/debugging).  
 * - MessageID.  
 * - DeviceID.  
 * - MessageLength is the length in bytes of the EMS Message.
 * - DataSize contain the number of Calduino Data that this Datagram contains. It will match the
 * array size of data.
 * - Data is an array in PROGMEM containing the Data included in this Datagram.
 */

struct EMSDatagram {
	prog_char* messageName;
	MessageID messageID;
	DeviceID destinationID;
	byte messageLength;
	byte dataSize;
	const PROGMEM CalduinoData* data;

	void printMessageName(char* str, boolean header, PrintFormat printFormat);
	void printErrorTag(char* str, PrintFormat printFormat);
};


typedef const PROGMEM CalduinoData Prog_CalduinoDataType;
typedef const PROGMEM EMSDatagram Prog_EMSDatagram;
#pragma endregion EMSDatagram

/* CalduinoDebug declaration */
#pragma region CalduinoDebug

class CalduinoDebug : public Stream {
private:
	Stream *debugSerial;

public:
	CalduinoDebug();
	void begin(Stream *_debugSerial);

	virtual size_t write(uint8_t byte);
	virtual int read() { return debugSerial->read(); }
	virtual int available() { return debugSerial->available(); }
	virtual void flush() { return debugSerial->flush(); }
	virtual int peek() { return debugSerial->peek(); }

	using Print::write;
};

#pragma endregion CalduinoDebug

/* CalduinoSerial declaration */
#pragma region CalduinoSerial

class CalduinoSerial {
private:
	EMSSerial * calduinoSerial;

public:
	CalduinoSerial();
	void begin(EMSSerial *_calduinoSerial);

	virtual size_t write(uint8_t byte) { return calduinoSerial->write(byte); }
	virtual int read() { return calduinoSerial->read(); }
	virtual int available() { return calduinoSerial->available(); }
	virtual void flush() { return calduinoSerial->flush(); }
	virtual int peek() { return calduinoSerial->peek(); }
	virtual void writeEOF() { return calduinoSerial->writeEOF(); }
	virtual bool frameError() { return calduinoSerial->frameError(); }
};

#pragma endregion CalduinoSerial

/* Calduino declaration */
#pragma region Calduino

class Calduino {
private:
	uint8_t crcCalculator(byte *eMSBuffer, int len);
	boolean crcCheckOK(byte * inEMSBuffer, int len);
	int readBytes(byte * inEMSBuffer, byte len, uint32_t eMSTimeout);
	void sendBuffer(byte * outEMSBuffer, int len);
	boolean sendRequest(byte *outEMSBuffer);
	boolean getEMSBuffer(byte *inEMSBuffer, EMSDatagram eMSDatagram, byte length = 0, byte offset = 0);
	boolean getEMSCommand(byte *inEMSBuffer, byte destinationID, byte messageID, byte length, byte offset = 0);
	boolean setEMSCommand(byte destinationID, byte messageID, byte offset, byte data);
	boolean updateEMSDatagram(EMSDatagramID eMSDatagramID, DatagramDataIndex datagramDataIndex, byte data, byte extraOffset = 0);

	unsigned long EMSMaxWaitTime;
	CalduinoDebug debugSerial;
	CalduinoSerial calduinoSerial;

public:
	Calduino();

	boolean begin(EMSSerial *_calduinoSerial, Stream *debugSerial = NULL);

	// Get EMS Commands
	boolean printEMSDatagram(EMSDatagramID eMSDatagramID, DatagramDataIndex datagramDataIndex = ERROR_VALUE);
	byte getCalduinoByteValue(ByteRequest typeIdx);
	float getCalduinoFloatValue(FloatRequest typeIdx);
	unsigned long getCalduinoUlongValue(ULongRequest typeIdx);
	boolean getCalduinoBitValue(BitRequest typeIdx);
	SwitchPoint getCalduinoSwitchPoint(EMSDatagramID selProgram, byte switchPointID);

	// Set EMS Commands
	boolean setWorkModeHC(byte selHC, byte selMode);
	boolean setTemperatureHC(byte selHC, byte selMode, byte selTmp);
	boolean setProgramHC(byte selHC, byte selProgram);
	boolean setSWThresholdTempHC(byte selHC, byte selTmp);
	boolean setNightSetbackModeHC(byte selHC, byte selMode);
	boolean setNightThresholdOutTempHC(byte selHC, int8_t selTmp);
	boolean setRoomTempOffsetHC(byte selHC, int8_t selTmp);
	boolean setPauseModeHC(byte selHC, byte duration);
	boolean setPartyModeHC(byte selHC, byte duration);
	boolean setHolidayModeHC(byte selHC, byte startHolidayDay, byte startHoldidayMonth, byte startHolidayYear, byte endHolidayDay, byte endHoldidayMonth, byte endHolidayYear);
	boolean setHomeHolidayModeHC(byte selHC, byte startHomeHolidayDay, byte startHomeHoldidayMonth, byte startHomeHolidayYear, byte endHomeHolidayDay, byte endHomeHoldidayMonth, byte endHomeHolidayYear);
	boolean setWorkModeDHW(byte selMode);
	boolean setWorkModePumpDHW(byte selMode);
	boolean setTemperatureDHW(byte selTmp);
	boolean setTemperatureTDDHW(byte selTmp);
	boolean setProgramDHW(byte selProgram);
	boolean setProgramPumpDHW(byte selProgram);
	boolean setOneTimeDHW(boolean selMode);
	boolean setWorkModeTDDHW(byte selMode);
	boolean setDayTDDHW(byte dayTherDisDHW);
	boolean setHourTDDHW(byte hourTherDisDHW);
	boolean setProgramSwitchPoint(EMSDatagramID selProgram, byte switchPointID, byte operationSwitchPoint, byte daySwitchPoint, byte hourSwitchPoint, byte minuteSwitchPoint);

	PrintFormat printFormat;
};

#pragma endregion Calduino

#endif
