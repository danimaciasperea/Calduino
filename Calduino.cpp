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
* @file Calduino.cpp
* 		
* @brief The Calduino class declaration including EMSSerial, CalduinoDebug, CalduinoSerial, Calduino Datagrams and Calduino Data Types.
*
*/

#include "wiring_private.h"
#include "Calduino.h"

/* EMS max/min values and constants */
#define MAX_HC_CIRCUIT 2
#define MAX_WORKING_MODE 2
#define MAX_SETBACK_MODE 3
#define MAX_PROGRAM 10
#define MAX_DHW_TEMPERATURE 80
#define MIN_DHW_TEMPERATURE 40
#define MAX_TEMPERATURE 29
#define MIN_TEMPERATURE 6
#define MAX_ROOM_TEMPERATURE_OFFSET 5
#define MIN_ROOM_TEMPERATURE_OFFSET -5
#define MAX_SUMMER_WINTER_THRESHOLD 30
#define MIN_SUMMER_WINTER_THRESHOLD 10
#define MAX_OUT_NIGHT_THRESHOLD 10
#define MIN_OUT_NIGHT_THRESHOLD -20
#define DHW_ONETIME_ON  39
#define DHW_ONETIME_OFF 7
#define SWITCHING_POINTS 42
#define MAX_DAY 31
#define MAX_DAY_WEEK 7
#define MAX_MONTH 12
#define MAX_HOUR_DAY 24
#define MAX_MINUTE_HOUR 60

/* EMS Bus Serial Parameters */
#define INITIAL_OFFSET 4
#define OUT_EMS_BUFFER_SIZE 7
#define TEXT_BUFFER_SIZE 40
#define EMS_DATAGRAM_OVERHEAD 6
#define EMS_MAX_WAIT_TIME 1000
#define RETRY_FACTOR 4

/* Message size*/
#define RC_DATETIME_VALUES_COUNT 6
#define RC_DATETIME_MESSAGE_SIZE 8
#define UBA_WORKING_TIME_VALUES_COUNT 1
#define UBA_WORKING_TIME_MESSAGE_SIZE 3
#define UBA_MONITOR_FAST_VALUES_COUNT 16
#define UBA_MONITOR_FAST_MESSAGE_SIZE 27
#define UBA_MONITOR_SLOW_VALUES_COUNT 6
#define UBA_MONITOR_SLOW_MESSAGE_SIZE 25
#define UBA_PARAMETER_DHW_VALUES_COUNT 2
#define UBA_PARAMETER_DHW_MESSAGE_SIZE 11
#define FLAGS_DHW_VALUES_COUNT 1
#define FLAGS_DHW_MESSAGE_SIZE 1
#define UBA_MONITOR_DHW_VALUES_COUNT 7
#define UBA_MONITOR_DHW_MESSAGE_SIZE 16
#define WORKING_MODE_DHW_VALUES_COUNT 7
#define WORKING_MODE_DHW_MESSAGE_SIZE 10
#define WORKING_MODE_HC_VALUES_COUNT 9
#define WORKING_MODE_HC_MESSAGE_SIZE 42
#define MONITOR_HC_VALUES_COUNT 5
#define MONITOR_HC_MESSAGE_SIZE 16
#define MONITOR_MM_10_VALUES_COUNT 3
#define MONITOR_MM_10_MESSAGE_SIZE 8
#define SWITCHING_PROGRAM_1_VALUES_COUNT 57
#define SWITCHING_PROGRAM_1_MESSAGE_SIZE 99
#define SWITCHING_PROGRAM_2_VALUES_COUNT 42
#define SWITCHING_PROGRAM_2_MESSAGE_SIZE 84

/** EMSSerial definition */
#pragma region EMSSERIAL

// on ATmega8, the uart and its bits are not numbered, so there is no "TXC0"
// definition.
#if !defined(TXC0)
#if defined(TXC)
#define TXC0 TXC
#elif defined(TXC1)
// Some devices have uart1 but no uart0
#define TXC0 TXC1
#else
#error TXC0 not definable in Calduino.h
#endif
#endif


/**
 * Stores a character in the EMS Serial Buffer.
 * This operation is called when a serial event is triggered
 *
 * @author	Administrator
 * @date	5/2/2018
 *
 * @param 		  	c 	Byte received.
 * @param 		  	fe	Wether the bit error is activated or not.
 * @param [in,out]	s 	The EMS Serial stream.
 */

inline void store_char(unsigned char c, bool fe, EMSSerial *s)
{
	int i = (unsigned int)(s->_rx_buffer_head + 1) % SERIAL_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != s->_rx_buffer_tail) {
		s->_rx_buffer[s->_rx_buffer_head] = c;
		s->_error_flag[s->_rx_buffer_head] = fe;
		s->_rx_buffer_head = i;
	}
}

#if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(USART_RX_vect) && !defined(USART0_RX_vect) && \
	!defined(USART_RXC_vect)
#error "Don't know what the Data Received vector is called for the first UART"
#else
/** Ems serial event 0 */
void EMSSerialEvent0() __attribute__((weak));
/** Ems serial event 0 */
void EMSSerialEvent0() {}
#define EMSSerialEvent0_implemented
#if defined(USART_RX_vect)
ISR(USART_RX_vect)
#elif defined(USART0_RX_vect)
ISR(USART0_RX_vect)
#elif defined(USART_RXC_vect)
ISR(USART_RXC_vect) // ATmega8
#endif
{
#if defined(UDR0)

	bool fe = bitRead(UCSR0A, FE0);
	unsigned char c = UDR0;
	store_char(c, fe, &EMSSerial0);
#elif defined(UDR)
	bool fe = bitRead(UCSRA, FE);
	unsigned char c = UDR;
	store_char(c, fe, &EMSSerial0);

#else
#error UDR not defined
#endif
}
#endif
#endif

#if defined(USART1_RX_vect)
void EMSSerialEvent1() __attribute__((weak));
void EMSSerialEvent1() {}
#define EMSSerialEvent1_implemented
ISR(USART1_RX_vect)
{

	bool fe = bitRead(UCSR1A, FE1);
	unsigned char c = UDR1;

	store_char(c, fe, &EMSSerial1);
}
#endif

#if defined(USART2_RX_vect) && defined(UDR2)
void EMSSerialEvent2() __attribute__((weak));
void EMSSerialEvent2() {}
#define EMSSerialEvent2_implemented
ISR(USART2_RX_vect)
{

	bool fe = bitRead(UCSR2A, FE2);
	unsigned char c = UDR2;

	store_char(c, fe, &EMSSerial2);
}
#endif

#if defined(USART3_RX_vect) && defined(UDR3)
void EMSSerialEvent3() __attribute__((weak));
void EMSSerialEvent3() {}
#define EMSSerialEvent3_implemented
ISR(USART3_RX_vect)
{

	bool fe = bitRead(UCSR3A, FE3);
	unsigned char c = UDR3;

	store_char(c, fe, &EMSSerial3);
}
#endif


/**
 * EMS Serial event run
 *
 * @author	Administrator
 * @date	5/2/2018
 */

void EMSSerialEventRun(void)
{
#ifdef EMSSerialEvent0_implemented
	if (EMSSerial0.available()) EMSSerialEvent0();
#endif
#ifdef EMSSerialEvent1_implemented
	if (EMSSerial1.available()) EMSSerialEvent1();
#endif
#ifdef EMSSerialEvent2_implemented
	if (EMSSerial2.available()) EMSSerialEvent2();
#endif
#ifdef EMSSerialEvent3_implemented
	if (EMSSerial3.available()) EMSSerialEvent3();
#endif
}


/**
 * EMS Serial Constructor
 *
 * @param [in]	ubrrh	USART Baud Rate Register.
 * @param [in]	ubrrl	UBRRL contains the eight least significant bits of the USART baud rate.
 * @param [in]	ucsra	USART Control and Status Register A.
 * @param [in]	ucsrb	USART Control and Status Register B.
 * @param [in]	ucsrc	USART Control and Status Register C.
 * @param [in]	udr  	The transmit buffer.
 * @param 	  	rxen 	Receiver Enable.
 * @param 	  	txen 	Transmitter Enable.
 * @param 	  	rxcie	RX Complete Interrupt Enable.
 */

EMSSerial::EMSSerial(
	volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
	volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
	volatile uint8_t *ucsrc, volatile uint8_t *udr,
	uint8_t rxen, uint8_t txen, uint8_t rxcie)
{
	_rx_buffer_head = _rx_buffer_tail = 0;
	_error = false;
	_ubrrh = ubrrh;
	_ubrrl = ubrrl;
	_ucsra = ucsra;
	_ucsrb = ucsrb;
	_ucsrc = ucsrc;
	_udr = udr;
	_rxen = rxen;
	_txen = txen;
	_rxcie = rxcie;

}


/** Default constructor */

EMSSerial::EMSSerial() {}


/**
 * Start the communication with the EMS Bus – UART Interface Circuit
 *
 * @param	baud	- baud rate of the UART port.
 *
 * @return	True if it succeeds, false if it fails.
 */

bool EMSSerial::begin(unsigned long baud)
{
	uint16_t baud_setting;

	*_ucsra = 0;
	
	_written = false;


	baud_setting = (F_CPU / 8 / baud - 1) / 2;

	// assign the baud_setting, ubbr (USART Baud Rate Register) and UBRRL (eight least significant
	// bits of the USART baud rate). 
	*_ubrrh = baud_setting >> 8;
	*_ubrrl = baud_setting;

	// set the bits in ucsrb port enabling reception, transmission and complete interrupt.
	sbi(*_ucsrb, _rxen); 
	sbi(*_ucsrb, _txen);
	sbi(*_ucsrb, _rxcie);
}


/** Ends the EMS Serial communication */

void EMSSerial::end()
{
	// clear the bits in ucsrb port disabling reception, transmission and complete interrupt.
	cbi(*_ucsrb, _rxen); 
	cbi(*_ucsrb, _txen);
	cbi(*_ucsrb, _rxcie);
	
	// clear any received data
	_rx_buffer_head = _rx_buffer_tail;
}

/**
 * Gets the bytes pending to be read
 *
 * @return	Available bytes in the EMS Serial Buffer.
 */

int EMSSerial::available(void)
{
	return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail) % SERIAL_BUFFER_SIZE;
}


/**
 * Returns the top-of-stack object without removing it, or -1 otherwise.
 *
 * @return	The current top-of-stack object. If there is no object, return -1.
 */

int EMSSerial::peek(void)
{
	if (_rx_buffer_head == _rx_buffer_tail) {
		return -1;
	}
	else {
		return _rx_buffer[_rx_buffer_tail];
	}
}


/**
 * Returns the top-of-stack object removing it.
 *
 * @return	The current top-of-stack object, if any. Return -1 otherwise.
 */

int EMSSerial::read(void)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (_rx_buffer_head == _rx_buffer_tail) {
		return -1;
	}
	else {
		unsigned char c = _rx_buffer[_rx_buffer_tail];
		_error = _error_flag[_rx_buffer_tail];
		_rx_buffer_tail = (unsigned int)(_rx_buffer_tail + 1) % SERIAL_BUFFER_SIZE;
		return c;
	}
}


/** Flushes the  EMS Serial by clearing the buffer contents */

void EMSSerial::flush()
{
	// clear read buffer
	_rx_buffer_head = _rx_buffer_tail;
	_error = false;

	// disable and enable reception in order to flush the receive buffer
	cbi(*_ucsrb, _rxen);
	sbi(*_ucsrb, _rxen);
}


/**
 * Writes the given character in the EMS Serial
 *
 * @param	c	The char to write.
 *
 * @return	The number of bytes written.
 */

size_t EMSSerial::write(uint8_t c)
{
	_written = true;

	// UDRE Flag indicates if the transmit buffer (UDR) is ready to receive new data. If UDRE is
	// one, the buffer is empty, and therefore ready to be written. Wait therefore until buffer
	// empty signal. 
	while (!(bitRead(*_ucsra, UDRE0))) {}

	// write value in the trasmit buffer
	*_udr = c;

	return 1;
}


/**
 * Write a EMS end-of-frame character to the port First halt temporarily reception, change
 * parity to even and then send a break-character. Then restore the settings and re-enable
 * reception
 */

void EMSSerial::writeEOF() {
	uint8_t t;
	cbi(*_ucsrb, _rxen);   						//disable reception
	while (!(bitRead(*_ucsra, UDRE0))) {}		// wait for data register empty
	t = *_ucsrc;								// save settings
	sbi(*_ucsrc, UPM01);						//set parity even
	sbi(*_ucsra, TXC0);							//reset TX-complete (seems to be needed to get parity change)
	*_udr = 0;									//directly write a break
	while (!(bitRead(*_ucsra, TXC0))) {}		//wait for transmit complete
	*_ucsrc = t;								//restore settings
	sbi(*_ucsra, TXC0);							//reset TX-complete (seems to be needed to get parity change)
	sbi(*_ucsrb, _rxen);   						//re-enable reception
}


/**
 * Cast that converts the given to a bool
 *
 * @return	The result of the operation.
 */

EMSSerial::operator bool() {
	return true;
}

#if defined(UBRRH) && defined(UBRRL)
EMSSerial EMSSerial0(&UBRRH, &UBRRL, &UCSRA, &UCSRB, &UCSRC, &UDR, RXEN, TXEN, RXCIE);
#elif defined(UBRR0H) && defined(UBRR0L)
EMSSerial EMSSerial0(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0, RXEN0, TXEN0, RXCIE0);
#endif
#if defined(UBRR1H)
EMSSerial EMSSerial1(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UCSR1C, &UDR1, RXEN1, TXEN1, RXCIE1);
#endif
#if defined(UBRR2H)
EMSSerial EMSSerial2(&UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UCSR2C, &UDR2, RXEN2, TXEN2, RXCIE2);
#endif
#if defined(UBRR3H)
EMSSerial EMSSerial3(&UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UCSR3C, &UDR3, RXEN3, TXEN3, RXCIE3);
#endif

#pragma endregion EMSSerial

/** CalduinoData definition */
#pragma region CalduinoData

/** Printf formatting options */
prog_char decimal[] = { "%d" };
prog_char unsignedLong[] = { "%lu" };
prog_char returnTag[] = { "Return" };

/** Units for formatting purposes. */
prog_char noneUnit[] = { "" };
prog_char celsiusUnit[] = { "\xC2\xB0\C" };
prog_char yesNoUnit[] = { "Yes/No" };
prog_char mAmperUnit[] = { "mAmper" };
prog_char barUnit[] = { "bar" };
prog_char minuteUnit[] = { "minutes" };
prog_char timesUnit[] = { "times" };
prog_char percentageUnit[] = { "%" };
prog_char dayUnit[] = { "day" };
prog_char switchPointUnit[] = { "" };
prog_char secondUnit[] = { "seconds" };

/** CalduinoUnits Array. */
const prog_char *calduinoUnits[] = {
	noneUnit,
	celsiusUnit,
	yesNoUnit,
	mAmperUnit,
	barUnit,
	minuteUnit,
	timesUnit,
	percentageUnit,
	switchPointUnit,
	secondUnit
};


/**
 * Decode a Calduino Data of type byte.
 *
 * @param [in]	inEMSBuffer	- The EMS bytes received.
 *
 * @return	The value of the Calduino Data in the EMS Buffer received.
 */

byte CalduinoData::decodeByteValue(byte* inEMSBuffer)
{
	return (uint8_t)inEMSBuffer[offset];
}



/**
* Decode a Calduino Data of type bit.
*
* @param [in]	inEMSBuffer	- The EMS bytes received.
*
* @return	The value of the Calduino Data in the EMS Buffer received.
*/

bool CalduinoData::decodeBitValue(byte* inEMSBuffer)
{
	return (bool)bitRead((uint8_t)inEMSBuffer[offset], bitOffset);
}

/**
* Decode a Calduino Data of type ULong.
*
* @param [in]	inEMSBuffer	- The EMS bytes received.
*
* @return	The value of the Calduino Data in the EMS Buffer received.
*/

unsigned long CalduinoData::decodeULongValue(byte* inEMSBuffer)
{
	return (((unsigned long)(uint8_t)inEMSBuffer[offset]) << 16) + (((unsigned long)(uint8_t)inEMSBuffer[offset + 1]) << 8) + ((uint8_t)inEMSBuffer[offset + 2]);
}


/**
 * Decode a Calduino Data of type float.
 *
 * @param [in]	inEMSBuffer	- The EMS bytes received.
 *
 * @return	The value of the Calduino Data in the EMS Buffer received.
 */

float CalduinoData::decodeFloatValue(byte *inEMSBuffer)
{
	return ((floatBytes == 2) ? (((float)((((uint8_t)inEMSBuffer[offset] << 8) + (uint8_t)inEMSBuffer[offset + 1]))) / floatFactor) : ((float)(((int8_t)inEMSBuffer[offset])) / floatFactor));
}

/**
* Decode a Calduino Data of type SwitchPoint.
*
* @param [in]	inEMSBuffer	- The EMS bytes received.
*
* @return	The SwitchPoint value contained in the EMS Buffer received.
*/

SwitchPoint CalduinoData::decodeSwitchPoint(byte *inEMSBuffer)
{
	SwitchPoint value = { (offset / 2) - 2, (uint8_t)(inEMSBuffer[offset] & 7), (uint8_t)(inEMSBuffer[offset] >> 5), (uint8_t)(inEMSBuffer[offset + 1] / 6), (uint8_t)((inEMSBuffer[offset + 1] % 6) * 10) };
	return value;
}


/**
 * Decodes the Calduino Data with the bytes contained in the EMS Buffer and convert returns an
 * string containing this decoded value.
 *
 * @param [in] 	inEMSBuffer	- EMS bytes received.
 * @param [out]	value	   	- pointer to an array of char elements where the decoded value is
 * 							is stored.
 */

void CalduinoData::decodeValue(byte* inEMSBuffer, char* value)
{
	// First decode the value of the CalduinoData contained in inEMSBuffer and convert it to an string in tempBuf
	switch (encodeType)
	{
		case CalduinoEncodeType::Byte:
		{
			sprintf_P(value, decimal, decodeByteValue(inEMSBuffer));
			break;
		}
		case CalduinoEncodeType::Bit:
		{
			sprintf_P(value, decimal, decodeBitValue(inEMSBuffer));
			break;
		}
		case CalduinoEncodeType::ULong:
		{
			sprintf_P(value, unsignedLong, decodeULongValue(inEMSBuffer));
			break;
		}
		case CalduinoEncodeType::Float:
		{
			dtostrf(decodeFloatValue(inEMSBuffer), 1, 1, value);
			break;
		}
		case CalduinoEncodeType::SwithPoint:
		{
			SwitchPoint valueSP = decodeSwitchPoint(inEMSBuffer);
			sprintf_P(value, PSTR("%d %d %d %d %d"), valueSP.id, valueSP.action, valueSP.day, valueSP.hour, valueSP.minute);
		}
	}
}


/**
 * Composes a formatted string with the CalduinoDataType name and its value. Depending on the
 * print format it will be encoded as an XML and will or not include units.
 *
 * @param [out]	str		   	- pointer to an array of char elements where the resulting string
 * 								is stored.
 * @param [in] 	value	   	- value of the current calduino data to be printed.
 * @param 	   	printFormat	- active printing format.
 */

void CalduinoData::printfValue(char* str, char* value, PrintFormat printFormat)
{
	// Second compose the output string according to the printFormat
	switch (printFormat)
	{
		case PrintFormat::NoUnit:
		{
			snprintf_P(str, TEXT_BUFFER_SIZE, PSTR("%S: %s"), dataName, value);
			break;
		}
		case PrintFormat::XML:
		{
			snprintf_P(str, TEXT_BUFFER_SIZE, PSTR("<%S>%s</%S>"), dataName, value, dataName);
			break;
		}
		case PrintFormat::Standard:
		{
			snprintf_P(str, TEXT_BUFFER_SIZE, PSTR("%S: %s %S"), dataName, value, calduinoUnits[unit]);
			break;
		}
	}
}

#pragma endregion CalduinoData

/** EMSDatagram definition */
#pragma region EMSDatagram

/** RC Datetime Datagram */
prog_char rCDatetimeName[] = { "RCDatetime" };
prog_char year[] = { "Year" };
prog_char month[] = { "Month" };
prog_char day[] = { "Day" };
prog_char hour[] = { "Hour" };
prog_char minute[] = { "Minute" };
prog_char second[] = { "Second" };

const PROGMEM CalduinoData rCDatetimeValues[] =
{
	{ year, CalduinoEncodeType::Byte, CalduinoUnit::None, 4 },
	{ month, CalduinoEncodeType::Byte, CalduinoUnit::None, 5 },
	{ day, CalduinoEncodeType::Byte, CalduinoUnit::None, 7 },
	{ hour, CalduinoEncodeType::Byte, CalduinoUnit::None, 6 },
	{ minute, CalduinoEncodeType::Byte, CalduinoUnit::None, 8 },
	{ second, CalduinoEncodeType::Byte, CalduinoUnit::None, 9 }
};

const PROGMEM EMSDatagram rCDatetime = { rCDatetimeName, MessageID::RC_Datetime_ID, DeviceID::RC_35, RC_DATETIME_MESSAGE_SIZE, RC_DATETIME_VALUES_COUNT, rCDatetimeValues };

/** UBA Working Time Datagram */
prog_char uBAWorkingTimeName[] = { "UBAWorkingTime" };
prog_char uBAWorkingMin[] = { "UBAWorkMin" };

const PROGMEM CalduinoData uBAWorkingTimeValues[] = {
	{uBAWorkingMin, CalduinoEncodeType::ULong, CalduinoUnit::Minute, 4}
};

const PROGMEM EMSDatagram uBAWorkingTime = { uBAWorkingTimeName, MessageID::UBA_Working_Time_ID, DeviceID::UBA, UBA_WORKING_TIME_MESSAGE_SIZE, UBA_WORKING_TIME_VALUES_COUNT, uBAWorkingTimeValues };

/** UBA Monitor Fast Datagram */
prog_char uBAMonitorFastName[] = { "UBAMonitorFast" };
prog_char selImpTemp[] = { "SelImpTemp" };
prog_char curImpTemp[] = { "CurImpTemp" };
prog_char retTemp[] = { "RetTemp" };
prog_char selBurnPow[] = { "SelBurnPow" };
prog_char curBurnPow[] = { "CurBurnPow" };
prog_char flameCurr[] = { "FlameCurr" };
prog_char sysPress[] = { "SysPress" };
prog_char burnGas[] = { "BurnGas" };
prog_char fanWork[] = { "FanWork" };
prog_char ignWork[] = { "IgnWork" };
prog_char heatPmp[] = { "HeatPmp" };
prog_char threeWayValveDHW[] = { "Way3ValveDHW" };
prog_char circDHW[] = { "CircDHW" };
prog_char srvCode1[] = { "SrvCode1" };
prog_char srvCode2[] = { "SrvCode2" };
prog_char errCode[] = { "ErrCode" };

const PROGMEM CalduinoData uBAMonitorFastValues[] = {
	{selImpTemp, CalduinoEncodeType::Byte, CalduinoUnit::Celsius, 4},
	{curImpTemp, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 5, 0, 2, 10},
	{selBurnPow, CalduinoEncodeType::Byte, CalduinoUnit::Percentage, 7},
	{curBurnPow, CalduinoEncodeType::Byte, CalduinoUnit::Percentage, 8},
	{burnGas, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 11, 0},
	{fanWork, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 11, 2},
	{ignWork, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 11, 3},
	{heatPmp, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 11, 5},
	{threeWayValveDHW, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 11, 6},
	{circDHW, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 11, 7},
	{retTemp, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 17, 0, 2, 10},
	{flameCurr, CalduinoEncodeType::Float, CalduinoUnit::MAmper, 19, 0, 2, 10},
	{sysPress, CalduinoEncodeType::Float, CalduinoUnit::Bar, 21, 0, 1, 10},
	{srvCode1, CalduinoEncodeType::Byte, CalduinoUnit::None, 22},
	{srvCode2, CalduinoEncodeType::Byte, CalduinoUnit::None, 23},
	{errCode, CalduinoEncodeType::Float, CalduinoUnit::None, 24, 0, 2, 1}
};

const PROGMEM EMSDatagram uBAMonitorFast = { uBAMonitorFastName, MessageID::UBA_Monitor_Fast_ID, DeviceID::UBA, UBA_MONITOR_FAST_MESSAGE_SIZE, UBA_MONITOR_FAST_VALUES_COUNT, uBAMonitorFastValues };

/** UBA Monitor Slow Datagram */
prog_char uBAMonitorSlowName[] = { "UBAMonitorSlow" };
prog_char extTemp[] = { "ExtTemp" };
prog_char boilTemp[] = { "BoilTemp" };
prog_char pumpMod[] = { "PumpMod" };
prog_char burnStarts[] = { "BurnStarts" };
prog_char burnWorkMin[] = { "BurnWorkMin" };
prog_char burnWorkMinH[] = { "BurnWorkMinH" };

const PROGMEM CalduinoData uBAMonitorSlowValues[] = {
	{extTemp, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 4, 0, 2, 10},
	{boilTemp, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 6, 0, 2, 10},
	{pumpMod, CalduinoEncodeType::Byte, CalduinoUnit::Percentage, 13},
	{burnStarts, CalduinoEncodeType::ULong, CalduinoUnit::Times, 14},
	{burnWorkMin, CalduinoEncodeType::ULong, CalduinoUnit::Minute, 17},
	{burnWorkMinH, CalduinoEncodeType::ULong, CalduinoUnit::Minute, 23}
};

const PROGMEM EMSDatagram uBAMonitorSlow = { uBAMonitorSlowName, MessageID::UBA_Monitor_Slow_ID, DeviceID::UBA, UBA_MONITOR_SLOW_MESSAGE_SIZE, UBA_MONITOR_SLOW_VALUES_COUNT, uBAMonitorSlowValues };

/** UBA Parameter DHW Datagram */
prog_char uBAParameterDHWName[] = { "UBAParameterDHW" };
prog_char selTempDHW[] = { "SelTempDHW" };
prog_char tempTDDHW[] = { "SelTempTDDHW" };

const PROGMEM CalduinoData uBAParameterDHWValues[] = {
	{selTempDHW, CalduinoEncodeType::Byte, CalduinoUnit::Celsius, 6},
	{tempTDDHW, CalduinoEncodeType::Byte, CalduinoUnit::Celsius, 12}
};

const PROGMEM EMSDatagram uBAParameterDHW = { uBAParameterDHWName, MessageID::UBA_Parameter_DHW_ID, DeviceID::UBA, UBA_PARAMETER_DHW_MESSAGE_SIZE, UBA_PARAMETER_DHW_VALUES_COUNT, uBAParameterDHWValues };

/** UBA Monitor DHW Datagram */
prog_char uBAMonitorDHWName[] = { "UBAMonitorDHW" };
prog_char curTempDHW[] = { "CurTempDHW" };
prog_char dayModeDHW[] = { "DayModeDHW" };
prog_char oneTimeDHW[] = { "OneTimeDHW" };
prog_char desDHW[] = { "DesDHW" };
prog_char prepareDHW[] = { "PrepareDHW" };
prog_char burnWorkMinDHW[] = { "BurnWorkMinDHW" };
prog_char burnStartsDHW[] = { "BurnStartsDHW" };

const PROGMEM CalduinoData uBAMonitorDHWValues[] = {
	{curTempDHW, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 5, 0, 2, 10},
	{dayModeDHW, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 9, 0},
	{oneTimeDHW, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 9, 1},
	{desDHW, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 9, 2},
	{prepareDHW, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 9, 3},
	{burnWorkMinDHW, CalduinoEncodeType::ULong, CalduinoUnit::Minute, 14},
	{burnStartsDHW, CalduinoEncodeType::ULong, CalduinoUnit::Times, 17}
};

const PROGMEM EMSDatagram uBAMonitorDHW = { uBAMonitorDHWName, MessageID::UBA_Monitor_DHW_ID, DeviceID::UBA, UBA_MONITOR_DHW_MESSAGE_SIZE, UBA_MONITOR_DHW_VALUES_COUNT, uBAMonitorDHWValues };

/** UBA Flags DHW Datagram */
prog_char flagsDHWName[] = { "FlagsDHW" };

const PROGMEM CalduinoData uBAflagsDHWValues[] = {
	{ oneTimeDHW, CalduinoEncodeType::Byte, CalduinoUnit::Celsius, 4 }
};

const PROGMEM EMSDatagram uBAFlagsDHW = { flagsDHWName, MessageID::Flags_DHW_ID, DeviceID::UBA, FLAGS_DHW_MESSAGE_SIZE, FLAGS_DHW_VALUES_COUNT, uBAflagsDHWValues };

/** Working Mode DHW Datagram */
prog_char workingModeDHWName[] = { "WorkingModeDHW" };
prog_char progDHW[] = { "ProgDHW" };
prog_char progPumpDHW[] = { "ProgPumpDHW" };
prog_char workModeDHW[] = { "WorkModeDHW" };
prog_char workModePumpDHW[] = { "WorkModePumpDHW" };
prog_char workModeTDDHW[] = { "WorkModeTDDHW" };
prog_char dayTDDHW[] = { "DayTDDHW" };
prog_char hourTDDHW[] = { "HourTDDHW" };

const PROGMEM CalduinoData workingModeDHWValues[] = {
	{progDHW, CalduinoEncodeType::Byte, CalduinoUnit::None, 4},
	{progPumpDHW, CalduinoEncodeType::Byte, CalduinoUnit::None, 5},
	{workModeDHW, CalduinoEncodeType::Byte, CalduinoUnit::None, 6},
	{workModePumpDHW, CalduinoEncodeType::Byte, CalduinoUnit::None, 7},
	{workModeTDDHW, CalduinoEncodeType::Byte, CalduinoUnit::YesNo, 8},
	{dayTDDHW, CalduinoEncodeType::Byte, CalduinoUnit::None, 9},
	{hourTDDHW, CalduinoEncodeType::Byte, CalduinoUnit::None, 10}
};

const PROGMEM EMSDatagram workingModeDHW = { workingModeDHWName, MessageID::Working_Mode_DHW_ID, DeviceID::RC_35, WORKING_MODE_DHW_MESSAGE_SIZE, WORKING_MODE_DHW_VALUES_COUNT, workingModeDHWValues };

/** Working Mode HC Datagram */
prog_char workingModeHC1Name[] = { "WorkingModeHC1" };
prog_char workingModeHC2Name[] = { "WorkingModeHC2" };
prog_char workingModeHC3Name[] = { "WorkingModeHC3" };
prog_char workingModeHC4Name[] = { "WorkingModeHC4" };
prog_char selNightTempHC[] = { "SelNightTempHC" };
prog_char selDayTempHC[] = { "SelDayTempHC" };
prog_char selHoliTempHC[] = { "SelHoliTempHC" };
prog_char roomTempInfHC[] = { "RoomTempInfHC" };
prog_char roomTempOffHC[] = { "RoomTempOffHC" };
prog_char workModeHC[] = { "WorkModeHC" };
prog_char sWThresTempHC[] = { "SWThresTempHC" };
prog_char nightSetbackHC[] = { "NightSetbackHC" };
prog_char nightOutTempHC[] = { "NightOutTempHC" };

const PROGMEM CalduinoData workingModeHCValues[] = {
	{selNightTempHC, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 5, 0, 1, 2},
	{selDayTempHC, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 6, 0, 1, 2},
	{selHoliTempHC, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 7, 0, 1, 2},
	{roomTempInfHC, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 8, 0, 1, 2},
	{roomTempOffHC, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 10, 0, 1, 2},
	{workModeHC, CalduinoEncodeType::Byte, CalduinoUnit::None, 11},
	{sWThresTempHC, CalduinoEncodeType::Byte, CalduinoUnit::Celsius, 26},
	{nightSetbackHC, CalduinoEncodeType::Byte, CalduinoUnit::None, 29},
	{nightOutTempHC, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 43, 0, 1, 1}
};

const PROGMEM EMSDatagram workingModeHC1 = { workingModeHC1Name, MessageID::Working_Mode_HC_1_ID, DeviceID::RC_35, WORKING_MODE_HC_MESSAGE_SIZE, WORKING_MODE_HC_VALUES_COUNT, workingModeHCValues };
const PROGMEM EMSDatagram workingModeHC2 = { workingModeHC2Name, MessageID::Working_Mode_HC_2_ID, DeviceID::RC_35, WORKING_MODE_HC_MESSAGE_SIZE, WORKING_MODE_HC_VALUES_COUNT, workingModeHCValues };
const PROGMEM EMSDatagram workingModeHC3 = { workingModeHC3Name, MessageID::Working_Mode_HC_3_ID, DeviceID::RC_35, WORKING_MODE_HC_MESSAGE_SIZE, WORKING_MODE_HC_VALUES_COUNT, workingModeHCValues };
const PROGMEM EMSDatagram workingModeHC4 = { workingModeHC4Name, MessageID::Working_Mode_HC_4_ID, DeviceID::RC_35, WORKING_MODE_HC_MESSAGE_SIZE, WORKING_MODE_HC_VALUES_COUNT, workingModeHCValues };

/** Monitor HC Datagram */
prog_char monitorHC1Name[] = { "MonitorHC1" };
prog_char monitorHC2Name[] = { "MonitorHC2" };
prog_char monitorHC3Name[] = { "MonitorHC3" };
prog_char monitorHC4Name[] = { "MonitorHC4" };
prog_char holiModHC[] = { "HoliModHC" };
prog_char summerModHC[] = { "SummerModHC" };
prog_char dayModHC[] = { "DayModHC" };
prog_char pauseModHC[] = { "PauseModHC" };
prog_char selRoomTempHC[] = { "SelRoomTempHC" };

const PROGMEM CalduinoData monitorHCValues[] = {
	{holiModHC, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 4, 5},
	{summerModHC, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 5, 0},
	{dayModHC, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 5, 1},
	{pauseModHC, CalduinoEncodeType::Bit, CalduinoUnit::YesNo, 5, 7},
	{selRoomTempHC, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 6, 0, 1, 2}
};

const PROGMEM EMSDatagram monitorHC1 = { monitorHC1Name, MessageID::Monitor_HC_1_ID, DeviceID::RC_35, MONITOR_HC_MESSAGE_SIZE, MONITOR_HC_VALUES_COUNT, monitorHCValues };
const PROGMEM EMSDatagram monitorHC2 = { monitorHC2Name, MessageID::Monitor_HC_2_ID, DeviceID::RC_35, MONITOR_HC_MESSAGE_SIZE, MONITOR_HC_VALUES_COUNT, monitorHCValues };
const PROGMEM EMSDatagram monitorHC3 = { monitorHC3Name, MessageID::Monitor_HC_3_ID, DeviceID::RC_35, MONITOR_HC_MESSAGE_SIZE, MONITOR_HC_VALUES_COUNT, monitorHCValues };
const PROGMEM EMSDatagram monitorHC4 = { monitorHC4Name, MessageID::Monitor_HC_4_ID, DeviceID::RC_35, MONITOR_HC_MESSAGE_SIZE, MONITOR_HC_VALUES_COUNT, monitorHCValues };

/** Switching Program Datagram */
prog_char programDHWName[] = { "ProgramDHW" };
prog_char programPumpDHWName[] = { "ProgramPumpDHW" };
prog_char program1HC1Name[] = { "Program1HC1" };
prog_char program1HC2Name[] = { "Program1HC2" };
prog_char program1HC3Name[] = { "Program1HC3" };
prog_char program1HC4Name[] = { "Program1HC4" };
prog_char program2HC1Name[] = { "Program2HC1" };
prog_char program2HC2Name[] = { "Program2HC2" };
prog_char program2HC3Name[] = { "Program2HC3" };
prog_char program2HC4Name[] = { "Program2HC4" };
prog_char switchPoint[] = { "SwitchPoint" };
prog_char programName[] = { "ProgramName" };
prog_char pauseTime[] = { "PauseTime" };
prog_char partyTime[] = { "PartyTime" };
prog_char startHolidayDay[] = { "StartHoliDay" };
prog_char startHolidayMonth[] = { "StartHoliMonth" };
prog_char startHolidayYear[] = { "StartHoliYear" };
prog_char endHolidayDay[] = { "EndHoliDay" };
prog_char endHolidayMonth[] = { "EndHoliMonth" };
prog_char endHolidayYear[] = { "EndHoliYear" };
prog_char startHomeHolidayDay[] = { "StartHHolDay" };
prog_char startHomeHolidayMonth[] = { "StartHHoliMonth" };
prog_char startHomeHolidayYear[] = { "StartHHoliYear" };
prog_char endHomeHolidayDay[] = { "EndHHoliDay" };
prog_char endHomeHolidayMonth[] = { "EndHHoliMonth" };
prog_char endHomeHolidayYear[] = { "EndHHoliYear" };

const PROGMEM CalduinoData switchingProgramValues[] = {
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 4},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 6},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 8},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 10}, 
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 12},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 14},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 16},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 18},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 20},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 22},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 24},	//10
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 26},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 28},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 30},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 32},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 34},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 36},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 38},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 40},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 42},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 44},	//20
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 46},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 48},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 50},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 52},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 54},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 56},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 58},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 60},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 62},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 64},	//30
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 66},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 68},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 70},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 72},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 74},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 76},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 78},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 80},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 82},
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 84},	//40
	{switchPoint, CalduinoEncodeType::SwithPoint, CalduinoUnit::None, 86},
	{programName, CalduinoEncodeType::Byte, CalduinoUnit::None, 88},
	{pauseTime, CalduinoEncodeType::Byte, CalduinoUnit::None, 89},			
	{partyTime, CalduinoEncodeType::Byte, CalduinoUnit::None, 90},
	{startHolidayDay, CalduinoEncodeType::Byte, CalduinoUnit::None, 91},
	{startHolidayMonth, CalduinoEncodeType::Byte, CalduinoUnit::None, 92},
	{startHolidayYear, CalduinoEncodeType::Byte, CalduinoUnit::None, 93},
	{endHolidayDay, CalduinoEncodeType::Byte, CalduinoUnit::None, 94},
	{endHolidayMonth, CalduinoEncodeType::Byte, CalduinoUnit::None, 95},
	{endHolidayYear, CalduinoEncodeType::Byte, CalduinoUnit::None, 96},		//50
	{startHomeHolidayDay, CalduinoEncodeType::Byte, CalduinoUnit::None, 97},
	{startHomeHolidayMonth, CalduinoEncodeType::Byte, CalduinoUnit::None, 98},
	{startHomeHolidayYear, CalduinoEncodeType::Byte, CalduinoUnit::None, 99},
	{endHomeHolidayDay, CalduinoEncodeType::Byte, CalduinoUnit::None, 100},
	{endHomeHolidayMonth, CalduinoEncodeType::Byte, CalduinoUnit::None, 101},
	{endHomeHolidayYear, CalduinoEncodeType::Byte, CalduinoUnit::None, 102}
};

const PROGMEM EMSDatagram program1HC1 = { program1HC1Name, MessageID::Program_1_HC_1_ID, DeviceID::RC_35, SWITCHING_PROGRAM_1_MESSAGE_SIZE, SWITCHING_PROGRAM_1_VALUES_COUNT, switchingProgramValues };
const PROGMEM EMSDatagram program1HC2 = { program1HC2Name, MessageID::Program_1_HC_2_ID, DeviceID::RC_35, SWITCHING_PROGRAM_1_MESSAGE_SIZE, SWITCHING_PROGRAM_1_VALUES_COUNT, switchingProgramValues };
const PROGMEM EMSDatagram program1HC3 = { program1HC3Name, MessageID::Program_1_HC_3_ID, DeviceID::RC_35, SWITCHING_PROGRAM_1_MESSAGE_SIZE, SWITCHING_PROGRAM_1_VALUES_COUNT, switchingProgramValues };
const PROGMEM EMSDatagram program1HC4 = { program1HC4Name, MessageID::Program_1_HC_4_ID, DeviceID::RC_35, SWITCHING_PROGRAM_1_MESSAGE_SIZE, SWITCHING_PROGRAM_1_VALUES_COUNT, switchingProgramValues };
const PROGMEM EMSDatagram program2HC1 = { program2HC1Name, MessageID::Program_2_HC_1_ID, DeviceID::RC_35, SWITCHING_PROGRAM_2_MESSAGE_SIZE, SWITCHING_PROGRAM_2_VALUES_COUNT, switchingProgramValues };
const PROGMEM EMSDatagram program2HC2 = { program2HC2Name, MessageID::Program_2_HC_2_ID, DeviceID::RC_35, SWITCHING_PROGRAM_2_MESSAGE_SIZE, SWITCHING_PROGRAM_2_VALUES_COUNT, switchingProgramValues };
const PROGMEM EMSDatagram program2HC3 = { program2HC3Name, MessageID::Program_2_HC_3_ID, DeviceID::RC_35, SWITCHING_PROGRAM_2_MESSAGE_SIZE, SWITCHING_PROGRAM_2_VALUES_COUNT, switchingProgramValues };
const PROGMEM EMSDatagram program2HC4 = { program2HC4Name, MessageID::Program_2_HC_4_ID, DeviceID::RC_35, SWITCHING_PROGRAM_2_MESSAGE_SIZE, SWITCHING_PROGRAM_2_VALUES_COUNT, switchingProgramValues };
const PROGMEM EMSDatagram programDHW = { programDHWName, MessageID::Program_DHW_ID, DeviceID::RC_35, SWITCHING_PROGRAM_1_MESSAGE_SIZE, SWITCHING_PROGRAM_1_VALUES_COUNT, switchingProgramValues };
const PROGMEM EMSDatagram programPumpDHW = { programPumpDHWName, MessageID::Program_Pump_DHW_ID, DeviceID::RC_35, SWITCHING_PROGRAM_1_MESSAGE_SIZE, SWITCHING_PROGRAM_1_VALUES_COUNT, switchingProgramValues };

/** Monitor MM10 Datagram */
prog_char monitorMM10Name[] = { "MonitorMM10" };
prog_char selImpTempMM10[] = { "SelImpTempMM10" };
prog_char curImpTempMM10[] = { "CurImpTempMM10" };
prog_char statusMM10[] = { "ModMM10" };

const PROGMEM CalduinoData monitorMM10Values[] = {
	{selImpTempMM10, CalduinoEncodeType::Byte, CalduinoUnit::Celsius, 4},
	{curImpTempMM10, CalduinoEncodeType::Float, CalduinoUnit::Celsius, 5, 0, 2, 10},
	{statusMM10, CalduinoEncodeType::Byte, CalduinoUnit::Percentage, 7}
};

const PROGMEM EMSDatagram monitorMM10  = { monitorMM10Name, MessageID::Monitor_MM_10_ID, DeviceID::MM_10, MONITOR_MM_10_MESSAGE_SIZE, MONITOR_MM_10_VALUES_COUNT, monitorMM10Values };

/** EMS Datagram Array , pointer to the defined EMS Datagrams */
EMSDatagram* eMSDatagramIDs[]  =
{
	&rCDatetime,
	&uBAWorkingTime,
	&uBAMonitorFast,
	&uBAMonitorSlow,
	&uBAParameterDHW,
	&uBAMonitorDHW,
	&uBAFlagsDHW,
	&workingModeDHW,
	&programDHW,
	&programPumpDHW,
	&workingModeHC1,
	&monitorHC1,
	&program1HC1,
	&program2HC1,
	&workingModeHC2,
	&monitorHC2,
	&program1HC2,
	&program2HC2,
	&workingModeHC3,
	&monitorHC3,
	&program1HC3,
	&program2HC3,
	&workingModeHC4,
	&monitorHC4,
	&program1HC4,
	&program2HC4,
	&monitorMM10
};


 /**
  * Structure that contains a Calduino Data and the EMS Datagram where it is contained. It is
  * used in order to print single values contained in a EMS Buffer.
  */

 struct CalduinoDataRequest
{
	CalduinoData* dataType;
	EMSDatagram* eMSDatagram;
};

/** Array with all the Calduino Data of type bytes. Is referenced by ByteRequest enumeration. */
const PROGMEM CalduinoDataRequest byteRequests[] =
{
	{ &rCDatetimeValues[yearIdx],				&rCDatetime },		//0
	{ &rCDatetimeValues[monthIdx],				&rCDatetime },
	{ &rCDatetimeValues[dayIdx],				&rCDatetime },
	{ &rCDatetimeValues[hourIdx],				&rCDatetime },
	{ &rCDatetimeValues[minuteIdx],				&rCDatetime },
	{ &rCDatetimeValues[secondIdx],				&rCDatetime },		//5
	{ &uBAMonitorFastValues[selImpTempIdx],		&uBAMonitorFast },
	{ &uBAMonitorFastValues[selBurnPowIdx],		&uBAMonitorFast },
	{ &uBAMonitorFastValues[curBurnPowIdx],		&uBAMonitorFast },
	{ &uBAMonitorFastValues[srvCode1Idx],		&uBAMonitorFast },
	{ &uBAMonitorFastValues[srvCode2Idx],		&uBAMonitorFast },	//10
	{ &uBAMonitorSlowValues[pumpModIdx],		&uBAMonitorSlow },
	{ &uBAParameterDHWValues[selTempDHWIdx],	&uBAParameterDHW },	
	{ &uBAParameterDHWValues[tempTDDHWIdx],		&uBAParameterDHW },
	{ &uBAflagsDHWValues[oneTimeDHW2Idx],		&uBAFlagsDHW },
	{ &workingModeDHWValues[progDHWIdx],		&workingModeDHW },	//15
	{ &workingModeDHWValues[progPumpDHWIdx],	&workingModeDHW },
	{ &workingModeDHWValues[workModeDHWIdx],	&workingModeDHW },
	{ &workingModeDHWValues[workModePumpDHWIdx],&workingModeDHW },
	{ &workingModeDHWValues[dayTDDHWIdx],		&workingModeDHW },
	{ &workingModeDHWValues[hourTDDHWIdx],		&workingModeDHW },	//20
	{ &workingModeHCValues[workModeHCIdx],		&workingModeHC1 },
	{ &workingModeHCValues[sWThresTempHCIdx],	&workingModeHC1 },
	{ &workingModeHCValues[nightSetbackHCIdx],	&workingModeHC1 },
	{ &workingModeHCValues[workModeHCIdx],		&workingModeHC2 },
	{ &workingModeHCValues[sWThresTempHCIdx],	&workingModeHC2 },	//25
	{ &workingModeHCValues[nightSetbackHCIdx],	&workingModeHC2 },
	{ &workingModeHCValues[workModeHCIdx],		&workingModeHC3 },
	{ &workingModeHCValues[sWThresTempHCIdx],	&workingModeHC3 },
	{ &workingModeHCValues[nightSetbackHCIdx],	&workingModeHC3 },
	{ &workingModeHCValues[workModeHCIdx],		&workingModeHC4 },	//30
	{ &workingModeHCValues[sWThresTempHCIdx],	&workingModeHC4 },
	{ &workingModeHCValues[nightSetbackHCIdx],	&workingModeHC4 },
	{ &switchingProgramValues[programNameIdx],	&program1HC1 },
	{ &switchingProgramValues[pauseTimeIdx],	&program1HC1 },
	{ &switchingProgramValues[partyTimeIdx],	&program1HC1 },		//35
	{ &switchingProgramValues[programNameIdx],	&program1HC2 },
	{ &switchingProgramValues[pauseTimeIdx],	&program1HC2 },
	{ &switchingProgramValues[partyTimeIdx],	&program1HC2 },
	{ &switchingProgramValues[programNameIdx],	&program1HC3 },
	{ &switchingProgramValues[pauseTimeIdx],	&program1HC3 },		//40
	{ &switchingProgramValues[partyTimeIdx],	&program1HC3 },
	{ &switchingProgramValues[programNameIdx],	&program1HC4 },
	{ &switchingProgramValues[pauseTimeIdx],	&program1HC4 },
	{ &switchingProgramValues[partyTimeIdx],	&program1HC4 },		//44
};

/** Array with all the Calduino Data of type float. Is referenced by FloatRequest enumeration. */
const PROGMEM CalduinoDataRequest floatRequests[] =
{
	{ &uBAMonitorFastValues[curImpTempIdx],	&uBAMonitorFast },		//0
	{ &uBAMonitorFastValues[retTempIdx],	&uBAMonitorFast },
	{ &uBAMonitorFastValues[flameCurrIdx],	&uBAMonitorFast },
	{ &uBAMonitorFastValues[sysPressIdx],	&uBAMonitorFast },
	{ &uBAMonitorFastValues[errCodeIdx],	&uBAMonitorFast },
	{ &uBAMonitorSlowValues[extTempIdx],	&uBAMonitorSlow },		//5
	{ &uBAMonitorSlowValues[boilTempIdx],	&uBAMonitorSlow },
	{ &uBAMonitorDHWValues[curTempDHWIdx],	&uBAMonitorDHW },
	{ &workingModeHCValues[selNightTempHCIdx],	&workingModeHC1 },
	{ &workingModeHCValues[selDayTempHCIdx],	&workingModeHC1 },
	{ &workingModeHCValues[selHoliTempHCIdx],	&workingModeHC1 },	//10
	{ &workingModeHCValues[roomTempInfHCIdx],	&workingModeHC1 },
	{ &workingModeHCValues[roomTempOffHCIdx],	&workingModeHC1 },
	{ &workingModeHCValues[nightOutTempHCIdx],	&workingModeHC1 },
	{ &workingModeHCValues[selNightTempHCIdx],	&workingModeHC2 },
	{ &workingModeHCValues[selDayTempHCIdx],	&workingModeHC2 },	//15
	{ &workingModeHCValues[selHoliTempHCIdx],	&workingModeHC2 },
	{ &workingModeHCValues[roomTempInfHCIdx],	&workingModeHC2 },
	{ &workingModeHCValues[roomTempOffHCIdx],	&workingModeHC2 },
	{ &workingModeHCValues[nightOutTempHCIdx],	&workingModeHC2 },
	{ &workingModeHCValues[selNightTempHCIdx],	&workingModeHC3 },	//20
	{ &workingModeHCValues[selDayTempHCIdx],	&workingModeHC3 },
	{ &workingModeHCValues[selHoliTempHCIdx],	&workingModeHC3 },
	{ &workingModeHCValues[roomTempInfHCIdx],	&workingModeHC3 },
	{ &workingModeHCValues[roomTempOffHCIdx],	&workingModeHC3 },
	{ &workingModeHCValues[nightOutTempHCIdx],	&workingModeHC3 },	//25
	{ &workingModeHCValues[selNightTempHCIdx],	&workingModeHC4 },
	{ &workingModeHCValues[selDayTempHCIdx],	&workingModeHC4 },
	{ &workingModeHCValues[selHoliTempHCIdx],	&workingModeHC4 },
	{ &workingModeHCValues[roomTempInfHCIdx],	&workingModeHC4 },
	{ &workingModeHCValues[roomTempOffHCIdx],	&workingModeHC4 },	//30
	{ &workingModeHCValues[nightOutTempHCIdx],	&workingModeHC4 },
	{ &workingModeHCValues[selRoomTempHCIdx],	&workingModeHC1 },
	{ &workingModeHCValues[selRoomTempHCIdx],	&workingModeHC2 },
	{ &workingModeHCValues[selRoomTempHCIdx],	&workingModeHC3 },
	{ &workingModeHCValues[selRoomTempHCIdx],	&workingModeHC4 },	//35
	{ &monitorMM10Values[curImpTempMM10Idx],	&monitorMM10 },		//36
};

/** Array with all the Calduino Data of type bit. Is referenced by BitRequest enumeration. */
const PROGMEM CalduinoDataRequest bitRequests[] =
{
	{ &uBAMonitorFastValues[burnGasIdx],			&uBAMonitorFast },	//0
	{ &uBAMonitorFastValues[fanWorkIdx],			&uBAMonitorFast },
	{ &uBAMonitorFastValues[ignWorkIdx],			&uBAMonitorFast },
	{ &uBAMonitorFastValues[heatPmpIdx],			&uBAMonitorFast },
	{ &uBAMonitorFastValues[threeWayValveDHWIdx],	&uBAMonitorFast },
	{ &uBAMonitorFastValues[circDHWIdx],			&uBAMonitorFast },	//5
	{ &uBAMonitorDHWValues[dayModeDHWIdx],			&uBAMonitorDHW },
	{ &uBAMonitorDHWValues[oneTimeDHW1Idx],			&uBAMonitorDHW },
	{ &uBAMonitorDHWValues[desDHWIdx],				&uBAMonitorDHW },
	{ &uBAMonitorDHWValues[prepareDHWIdx],			&uBAMonitorDHW },
	{ &uBAMonitorDHWValues[holiModHCIdx],			&monitorHC1 },		//10
	{ &uBAMonitorDHWValues[summerModHCIdx],			&monitorHC1 },
	{ &uBAMonitorDHWValues[dayModHCIdx],			&monitorHC1 },
	{ &uBAMonitorDHWValues[pauseModHCIdx],			&monitorHC1 },
	{ &uBAMonitorDHWValues[holiModHCIdx],			&monitorHC2 },
	{ &uBAMonitorDHWValues[summerModHCIdx],			&monitorHC2 },		//15
	{ &uBAMonitorDHWValues[dayModHCIdx],			&monitorHC2 },
	{ &uBAMonitorDHWValues[pauseModHCIdx],			&monitorHC2 },
	{ &uBAMonitorDHWValues[holiModHCIdx],			&monitorHC3 },
	{ &uBAMonitorDHWValues[summerModHCIdx],			&monitorHC3 },
	{ &uBAMonitorDHWValues[dayModHCIdx],			&monitorHC3 },		//20
	{ &uBAMonitorDHWValues[pauseModHCIdx],			&monitorHC3 },
	{ &uBAMonitorDHWValues[holiModHCIdx],			&monitorHC4 },
	{ &uBAMonitorDHWValues[summerModHCIdx],			&monitorHC4 },
	{ &uBAMonitorDHWValues[dayModHCIdx],			&monitorHC4 },
	{ &uBAMonitorDHWValues[pauseModHCIdx],			&monitorHC4 },		//25
};

/** Array with all the Calduino Data of type ulong. Is referenced by uLongRequest enumeration. */
const PROGMEM CalduinoDataRequest uLongRequests[] =
{
	{ &uBAWorkingTimeValues[uBAWorkingMinIdx], &uBAWorkingTime },
	{ &uBAMonitorSlowValues[burnStartsIdx], &uBAMonitorSlow },
	{ &uBAMonitorSlowValues[burnWorkMinIdx], &uBAMonitorSlow },
	{ &uBAMonitorSlowValues[burnWorkMinHIdx], &uBAMonitorSlow },
	{ &uBAMonitorDHWValues[burnStartsDHWIdx], &uBAMonitorDHW },
	{ &uBAMonitorDHWValues[burnWorkMinDHWIdx], &uBAMonitorDHW },
};


/**
 * Composes a formatted string with the EMSDatagram name sending the output to a char array
 * pointed by str.
 *
 * @param [out]	str		   	- pointer to an array of char elements where the resulting string
 * 								is stored.
 * @param 	   	header	   	- whether the current operation is a header or tail message.
 * @param 	   	printFormat	- selected printing format.
 */

void EMSDatagram::printMessageName(char* str, boolean header, PrintFormat printFormat)
{
	// compose the output string according to the printFormat
	switch (printFormat)
	{
		case PrintFormat::NoUnit: 
		case PrintFormat::Standard:
		{
			snprintf_P(str, TEXT_BUFFER_SIZE, PSTR("--- %S ---"), messageName);
			break;
		}
		case PrintFormat::XML:
		{
			snprintf_P(str, TEXT_BUFFER_SIZE, (header ? PSTR("<%S>") : PSTR("</%S>")), messageName);
			break;
		}
	}
}

/**
 * Composes a formatted string with the EMS Datagram error status sending the output to a char
 * array pointed by str.
 *
 * @param [out]	str		   	- pointer to an array of char elements where the resulting
 * 								string is stored.
 * @param 	   	printFormat	- selected printing format.
 */

void EMSDatagram::printErrorTag(char* str, PrintFormat printFormat)
{
	// compose the output string according to the printFormat
	switch (printFormat)
		{
		case PrintFormat::NoUnit:
		{
			snprintf_P(str, TEXT_BUFFER_SIZE, PSTR("%S: %d"), returnTag, 0);
			break;
		}
		case PrintFormat::XML:
		{
			snprintf_P(str, TEXT_BUFFER_SIZE, PSTR("<%S>%d</%S>"), returnTag, 0, returnTag);
			break;
		}
		case PrintFormat::Standard:
		{
			snprintf_P(str, TEXT_BUFFER_SIZE, PSTR("%S: %d"), returnTag, 0);
			break;
		}
	}
}


#pragma endregion EMSDatagram

/* CalduinoDebug definition */
#pragma region CalduinoDebug

/** Default constructor */
CalduinoDebug::CalduinoDebug()
{
	debugSerial = NULL;
}


/**
 * Begins the given debug serial
 *
 * @param [in,out]	_debugSerial	The stream to be used as debug serial.
 */

void CalduinoDebug::begin(Stream *_debugSerial)
{
	debugSerial = _debugSerial;
}


/**
 * Writes the given data in the debug serial
 *
 * @param	data	The data to write.
 *
 * @return	The number of bytes written.
 */

size_t CalduinoDebug::write(uint8_t data)
{
	if (debugSerial != NULL) {
		return debugSerial->write(data);
	}

	return 0;
}

#pragma endregion CalduinoDebug

/* CalduinoSerial definition */
#pragma region CalduinoSerial

/** Default constructor */
CalduinoSerial::CalduinoSerial()
{
	calduinoSerial = NULL;
}


/**
 * Begins the given calduino serial
 *
 * @param [in,out]	_calduinoSerial	The stream to be used as calduino serial.
 */

void CalduinoSerial::begin(EMSSerial *_calduinoSerial)
{
	calduinoSerial = _calduinoSerial;
}
#pragma endregion CalduinoSerial

/* Calduino definition */
#pragma region Calduino

/** Calduino constructor method. Initialize variables. */
Calduino::Calduino()
{
	EMSMaxWaitTime = EMS_MAX_WAIT_TIME;
	printFormat = PrintFormat::Standard;
}


/**
 * Start the Calduino device assigning the serial streams used.
 *
 * @param [in,out]	_calduinoSerial	- UART port used to communicate with the EMS Serial.
 * @param [in,out]	_debugSerial   	- Optional stream to report errors and status.
 *
 * @return	True if there is communication with the EMS Buffer, false otherwise.
 */

boolean Calduino::begin(EMSSerial *_calduinoSerial, Stream *_debugSerial)
{
	calduinoSerial.begin(_calduinoSerial);
	debugSerial.begin(_debugSerial);

	// Define DEBUG FUNCTIONS
	if (_debugSerial != NULL)
	{
#ifndef DEBUG
#define DEBUG
#define DPRINT(item) debugSerial.print(item)
#define DPRINTLN(item) debugSerial.println(item)
#define DPRINTVALUE(item1, item2) debugSerial.print(item1);debugSerial.print(": ");debugSerial.println(item2)
#endif
	}

	return getCalduinoByteValue(ByteRequest::second_b);
}


/**
 * Calculate the CRC code of the buffer passed as parameter.
 *
 * @param [in]	eMSBuffer	Buffer for which the CRC is calculated.
 * @param 	  	len		 	Length of the buffer.
 *
 * @return	CRC code.
 */

uint8_t Calduino::crcCalculator(byte * eMSBuffer, int len)
{
	uint8_t i, crc = 0x0;
	uint8_t d;
	for (i = 0; i < len - 2; i++)
	{
		d = 0;
		if (crc & 0x80)
		{
			crc ^= 12;
			d = 1;
		}
		crc = (crc << 1) & 0xfe;
		crc |= d;
		crc ^= eMSBuffer[i];
	}
	return crc;
}


/**
 * Check if the CRC code calculated for the buffer corresponds with the CRC value received in
 * the buffer (second to last position).
 *
 * @param [in]	inEMSBuffer	Buffer for which the CRC is calculated.
 * @param 	  	len		   	The length of the buffer.
 *
 * @return	If the CRC received and the calculated are equal.
 */

boolean Calduino::crcCheckOK(byte * inEMSBuffer, int len)
{
	int crc = crcCalculator(inEMSBuffer, len);
	boolean crcOK = (crc == (uint8_t)inEMSBuffer[len - 2]);
	return crcOK;
}


/**
 * Read one bus frame and return number of read bytes. Includes a timeout in order not to block
 * the program if there is no communication with the EMS Bus
 *
 * @param [in,out]	inEMSBuffer	Buffer where the income data will be stored.
 * @param 		  	len		   	The maximum length of the read datagram expected.
 * @param 		  	eMSTimeout 	Operation timeout in milliseconds.
 *
 * @return	Number of read bytes.
 */

int Calduino::readBytes(byte * inEMSBuffer, byte len, unsigned long eMSTimeout)
{
	int ptr = 0;

	// while there is available data and no timeout, skip the 0's in the buffer
	while (calduinoSerial.available() && (millis() < eMSTimeout))
	{
		// if the first byte to be read is 0x00 (break)
		if ((uint8_t)calduinoSerial.peek() == 0)
		{
			// skip breaks
			calduinoSerial.read();
		}
		else
		{
			// break the loop
			break;
		}
	}

	// read data until frame-error, max bytes are read or timeout
	while ((!calduinoSerial.frameError()) && (ptr < len) && (millis() < eMSTimeout))
	{
		if (calduinoSerial.available())
		{
			// store the information in inEMSBuffer and update the number of bytes read
			inEMSBuffer[ptr] = calduinoSerial.read();
			ptr++;
		}
	}

	// flush the possible pending information left to be read (garbage)
	calduinoSerial.flush();

	// return the number of bytes read
	return ptr;
}


/**
 * Send a data frame to the EMS BUS.
 *
 * @param [in]	outEMSBuffer	Buffer where the output data is stored.
 * @param 	  	len				The length of the buffer.
 */

void Calduino::sendBuffer(byte * outEMSBuffer, int len)
{
	for (byte j = 0; j < len - 1; j++)
	{
		calduinoSerial.write(outEMSBuffer[j]);
		delay(3);
	}

	// write a EMS end - of - frame character to the serial buffer
	calduinoSerial.writeEOF();
	delay(2);
	calduinoSerial.flush();
}


/**
 * Wait until Calduino is polled and send the content of the buffer. If the connection with the
 * EMS Serial is not working properly, the function will timeout and return false.
 *
 * @param [in,out]	outEMSBuffer	Buffer where the output data is stored.
 *
 * @return	Whether the datagram has been sent in the available time or not.
 */

boolean Calduino::sendRequest(byte *outEMSBuffer)
{
	// calculate the CRC value in the sIdxth position of the buffer
	outEMSBuffer[5] = crcCalculator(outEMSBuffer, OUT_EMS_BUFFER_SIZE);

	// last polled address (wait until this is 0x0B)
	byte pollAddress = 0;

	// flush the EMS Serial Stream by clearing (if any) current buffer contents
	calduinoSerial.flush();

	// watchdog (maximum polling waiting time)
	unsigned long eMSTimeout = millis() + EMSMaxWaitTime * RETRY_FACTOR;

	byte auxBuffer[MAX_EMS_READ];

	// wait until being polled by the Bus Master
	// Loop unitl PollAddress is the device ID that Calduino simulates (PC)
	while ((pollAddress & 0x7F) != DeviceID::PC)
	{
		// end the operation without sending the request if the timeout expires
		if (millis() > eMSTimeout) return false;
		
		// Read next datagram without limits of size (do not force 2 bytes read, in case UBA sends a monitor)
		// Assign the first  read byte to pollAddress only if two bytes are read (bus master polls:
		// pollAddress + Break) 
		if (readBytes(auxBuffer, MAX_EMS_READ, eMSTimeout) == 2)
		{
			pollAddress = auxBuffer[0];
		}
	}

	// wait two milliseconds and send the 7 bytes buffer (6 bytes + break) 
	delay(2);
	sendBuffer(outEMSBuffer, OUT_EMS_BUFFER_SIZE);

	return true;

}


/**
 * Generic method to send an EMS get command and save the response in a buffer passed as
 * parameter. If the message length is greater than the MAX_EMS_READ, the command will be
 * splitted in shorter requests.
 *
 * @param [out]	inEMSBuffer  	Pointer to the buffer where the EMS Datagram received will be
 * 								saved. It will always have the size of the whole EMS Message, no
 * 								matter if only a Calduino Data is requested.
 * @param 	   	destinationID	The destinationID of the EMS device.
 * @param 	   	messageID	 	The messageID requested.
 * @param 	   	length		 	Expected length of the receiving EMS Datagram.
 * @param 	   	offset		 	(Optional) Only used in order to recieve a unique Calduino Data
 * 								and not whole EMS Message. Length parameter should be adapted to
 * 								the size of the Calduino Data.
 *
 * @return	True if it succeeds, false otherwise.
 */

boolean Calduino::getEMSCommand(byte *inEMSBuffer, byte destinationID, byte messageID, byte length, byte offset = 0)
{
	byte outEMSBuffer[OUT_EMS_BUFFER_SIZE];
	unsigned long timeout;

	// load outEMSBuffer with corresponding values.
	// first position is the transmitterID. Ox0B is the ComputerID (Calduino address)
	outEMSBuffer[0] = DeviceID::PC;

	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = destinationID | 0x80;

	// third position is the messageID
	outEMSBuffer[2] = messageID;
	
	// assign this fake value in offset position just to force a first loop
	outEMSBuffer[3] = -1;

	// while there are still bytes pending to be read and the last EMS Command was correctly executed (offset has been increased)
	while (length > 0 && (offset > (char)outEMSBuffer[3]))
	{
		// fourth position is the offset in the buffer. If the message is split in different commands,
		// the offset contains the byte required from the EMS Datagram
		outEMSBuffer[3] = offset;

		// fifth position is the length of the data requested. If it is greater than the maximum utile bytes read (MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD), read then only this quantity
		outEMSBuffer[4] = (length > (MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD) ? (MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD) : length);

		// once the buffer is loaded, send the request.
		if (sendRequest(outEMSBuffer))
		{
			// check if the requested query is answered in the next EMSMaxWaitTime milliseconds
			timeout = millis() + EMSMaxWaitTime;

			// wait until timeout or there is some new data in the EMS-Bus
			while ((millis() < timeout) && (!calduinoSerial.available())) {}

			// if there is data to be read
			if (calduinoSerial.available())
			{
				// auxiliar buffer with a length long enough to capture current EMS Datagram
				byte auxBuffer[outEMSBuffer[4] + EMS_DATAGRAM_OVERHEAD];

				// read in auxiliar buffer the information received in EMS Serial
				int ptr = readBytes(auxBuffer, outEMSBuffer[4] + EMS_DATAGRAM_OVERHEAD, timeout);

				// if more than 4 bytes are read (datagram received)
				// check if the CRC of the information received is correct and the operation type returned corresponds with the one requested
				if ((ptr > 4) && crcCheckOK(auxBuffer, ptr) && (auxBuffer[2] == messageID))
				{
					// copy the bytes read from auxiliarBuffer to inEMSBuffer taking into account the internal offset (reconstruct the EMS Datagram)
					for (int i = 0; i < outEMSBuffer[4]; i++)
					{
						inEMSBuffer[i + 4 + auxBuffer[3]] = auxBuffer[i + 4];
					}

					// update the length and offset values to prepare the read of the next block
					length -= (length >(MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD) ? (MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD) : length);
					offset += (MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD);
				}
			}
		}
	}

	return (length == 0);
}


/**
 * Generic method to send an EMS set command with 1 data byte and check if the configuration
 * change request has been correctly performed.
 *
 * @param	destinationID	The destinationID of the EMS device.
 * @param	messageID	 	The messageID where the configuration is.
 * @param	offset		 	The offset of the data inside the message.
 * @param	data		 	The data/configuration to be set.
 *
 * @return	True if it succeeds, false otherwise.
 */

boolean Calduino::setEMSCommand(byte destinationID, byte messageID, byte offset, byte data)
{
	boolean operationStatus = false;

	byte outEMSBuffer[OUT_EMS_BUFFER_SIZE];
	byte inEMSBuffer[OUT_EMS_BUFFER_SIZE];

	unsigned long timeout;

	// First load outEMSBuffer with corresponding values for a SET Command
	// first position is the transmitterID. Ox0b is the ComputerID (our address)
	outEMSBuffer[0] = DeviceID::PC;

	// second position is destinationID.
	outEMSBuffer[1] = destinationID;

	// third position is the messageID
	outEMSBuffer[2] = messageID;

	// fourth position is the offset in the buffer.
	outEMSBuffer[3] = offset;

	// fifth position is the data to send
	outEMSBuffer[4] = data;

	// once the buffer is loaded, send the request.
	if (sendRequest(outEMSBuffer))
	{
		// check if the requested query is answered in the next EMSMaxWaitTime milliseconds
		timeout = millis() + EMSMaxWaitTime;

		// wait until timeout or some new data in the EMS-Bus
		while ((millis() < timeout) && (!calduinoSerial.available())) {}

		// if there is data to be read
		if (calduinoSerial.available())
		{
			// search confirmation datagram
			int ptr = readBytes(inEMSBuffer, 1, timeout);

			// if the answer received is 0x01, the value has been correctly sent, return with false otherwise
			if (inEMSBuffer[0] != 0x01)
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
	// Second Load outEMSBuffer with corresponding values for a GET Command and check if value received matches
	// second position is destinationID. Masked with 0x80 as a read command
	outEMSBuffer[1] = destinationID | 0x80;

	// fifth position is the length of the data requested.
	outEMSBuffer[4] = 1;

	// once the buffer is loaded, send the request.
	if (sendRequest(outEMSBuffer))
	{
		// check if the requested query is answered in the next EMSMaxWaitTime milliseconds
		timeout = millis() + EMSMaxWaitTime;

		// wait until timeout or some new data in the EMS-Bus
		while ((millis() < timeout) && (!calduinoSerial.available())) {}

		// if there is data to be read
		if (calduinoSerial.available())
		{
			// read the information sent
			int ptr = readBytes(inEMSBuffer, OUT_EMS_BUFFER_SIZE, timeout);

			// if more than 4 bytes are read (datagram received)
			// the CRC of the information received is correct
			// and the operation type returned corresponds with the one requested
			if ((ptr > 4) && crcCheckOK(inEMSBuffer, ptr) && (inEMSBuffer[2] == messageID))
			{
				// check if the data received corresponds with the change requested
				operationStatus = ((data == (uint8_t)inEMSBuffer[4]));
			}
		}
	}

	return operationStatus;
}


/**
 * Generic method to get an EMS Datagram. By default will obtain the whole EMSDatagram message
 * Length bytes. To obtain only a Calduino Data, set accordingly length and offset parameters.
 *
 * @param [out]	inEMSBuffer	Pointer to the buffer where the EMS Datagram received will be saved.
 * 							It will always have the size of the whole EMS Message, no matter if
 * 							only a Calduino Data is requested.
 * @param 	   	eMSDatagram	The EMSDatagram to obtain.
 * @param 	   	length	   	(Optional) The length of the Calduino Data.
 * @param 	   	offset	   	(Optional) The offset of the Calduino Data in the EMSBuffer.
 *
 * @return	True if it succeeds, false otherwise.
 */

boolean Calduino::getEMSBuffer(byte* inEMSBuffer, EMSDatagram eMSDatagram, byte length = 0, byte offset = 0)
{
	boolean operationStatus;

	// configure a timeout
	unsigned long timeout = (long)millis() + EMSMaxWaitTime * RETRY_FACTOR * 2;

	// get the EMS Datagram Bytes, repeat operation if failed until timeout
	do
	{
		operationStatus = getEMSCommand(inEMSBuffer, eMSDatagram.destinationID, eMSDatagram.messageID, (length == 0 ? eMSDatagram.messageLength : length), (offset == 0 ? offset : offset - INITIAL_OFFSET));
	} while ((millis() < timeout) && (!operationStatus));

	return operationStatus;
}

/**
 * Get a Calduino Data of type Byte.
 *
 * @param	typeIdx	Identifier of the Calduino Data Byte requested.
 *
 * @return	The value of the Calduino Data requested, ERROR_VALUE otherwise.
 */

byte Calduino::getCalduinoByteValue(ByteRequest typeIdx)
{
	byte result = ERROR_VALUE;

	// get from program memory the CalduinoDataRequest
	CalduinoDataRequest calduinoDataType;
	memcpy_P(&calduinoDataType, &byteRequests[typeIdx], sizeof(CalduinoDataRequest));

	// get from program memory the EMSDatagram
	EMSDatagram eMSDatagram;
	memcpy_P(&eMSDatagram, calduinoDataType.eMSDatagram, sizeof(EMSDatagram));

	// get from program memory the CalduinoData 
	CalduinoData calduinoData;
	memcpy_P(&calduinoData, calduinoDataType.dataType, sizeof(CalduinoData));

	// buffer where the EMS Datagram will be saved (size is message size plus EMS_DATAGRAM_OVERHEAD bytes to store the headers, CRC and break)
	byte inEMSBuffer[eMSDatagram.messageLength + EMS_DATAGRAM_OVERHEAD];

	// get an EMS Buffer with the parameters requested. Length is 1 (byte) and offset is the position of the data type in the EMSBuffer
	boolean operationStatus = getEMSBuffer(inEMSBuffer, eMSDatagram, 1, calduinoData.offset);

	if (operationStatus)
	{
		result = calduinoData.decodeByteValue(inEMSBuffer);
	}

	return result;
}


/**
 * Get a Calduino Data of type Float.
 *
 * @param	typeIdx	Identifier of the Calduino Data Float requested.
 *
 * @return	The value of the Calduino Data requested, NAN otherwise.
 */

float Calduino::getCalduinoFloatValue(FloatRequest typeIdx)
{
	float result = NAN;

	// get from program memory the CalduinoDataRequest
	CalduinoDataRequest calduinoDataType;
	memcpy_P(&calduinoDataType, &floatRequests[typeIdx], sizeof(CalduinoDataRequest));

	// get from program memory the EMSDatagram
	EMSDatagram eMSDatagram;
	memcpy_P(&eMSDatagram, calduinoDataType.eMSDatagram, sizeof(EMSDatagram));

	// get from program memory the CalduinoData 
	CalduinoData calduinoData;
	memcpy_P(&calduinoData, calduinoDataType.dataType, sizeof(CalduinoData));

	// buffer where the EMS Datagram will be saved (size is message size plus EMS_DATAGRAM_OVERHEAD bytes to store the headers, CRC and break)
	byte inEMSBuffer[eMSDatagram.messageLength + EMS_DATAGRAM_OVERHEAD];

	// get an EMS Buffer with the parameters requested. Length is 1 or 2 (bytes) and offset is the position of the data type in the EMSBuffer
	boolean operationStatus = getEMSBuffer(inEMSBuffer, eMSDatagram, calduinoData.floatBytes, calduinoData.offset);

	if (operationStatus)
	{
		result = calduinoData.decodeFloatValue(inEMSBuffer);
	}

	return result;
}


/**
 * Get a Calduino Data of type ULong.
 *
 * @param	typeIdx	Identifier of the Calduino Data ULong requested.
 *
 * @return	The value of the Calduino Data requested, NAN otherwise.
 */

unsigned long Calduino::getCalduinoUlongValue(ULongRequest typeIdx)
{
	unsigned long result = NAN;

	// get from program memory the CalduinoDataRequest
	CalduinoDataRequest calduinoDataType;
	memcpy_P(&calduinoDataType, &uLongRequests[typeIdx], sizeof(CalduinoDataRequest));

	// get from program memory the EMSDatagram
	EMSDatagram eMSDatagram;
	memcpy_P(&eMSDatagram, calduinoDataType.eMSDatagram, sizeof(EMSDatagram));

	// get from program memory the CalduinoData 
	CalduinoData calduinoData;
	memcpy_P(&calduinoData, calduinoDataType.dataType, sizeof(CalduinoData));

	// buffer where the EMS Datagram will be saved (size is message size plus EMS_DATAGRAM_OVERHEAD bytes to store the headers, CRC and break)
	byte inEMSBuffer[eMSDatagram.messageLength + EMS_DATAGRAM_OVERHEAD];

	// get an EMS Buffer with the parameters requested. Length is 3 (bytes) and offset is the position of the data type in the EMSBuffer
	boolean operationStatus = getEMSBuffer(inEMSBuffer, eMSDatagram, 3, calduinoData.offset);

	if (operationStatus)
	{
		result = calduinoData.decodeULongValue(inEMSBuffer);
	}

	return result;
}


/**
 * Get a Calduino Data of type Bit.
 *
 * @param	typeIdx	Identifier of the Calduino Data Bit requested.
 *
 * @return	The value of the Calduino Data requested, false otherwise.
 */

boolean Calduino::getCalduinoBitValue(BitRequest typeIdx)
{
	boolean result = NAN;

	// get from program memory the CalduinoDataRequest
	CalduinoDataRequest calduinoDataType;
	memcpy_P(&calduinoDataType, &bitRequests[typeIdx], sizeof(CalduinoDataRequest));

	// get from program memory the EMSDatagram
	EMSDatagram eMSDatagram;
	memcpy_P(&eMSDatagram, calduinoDataType.eMSDatagram, sizeof(EMSDatagram));

	// get from program memory the CalduinoData 
	CalduinoData calduinoData;
	memcpy_P(&calduinoData, calduinoDataType.dataType, sizeof(CalduinoData));

	// buffer where the EMS Datagram will be saved (size is message size plus EMS_DATAGRAM_OVERHEAD bytes to store the headers, CRC and break)
	byte inEMSBuffer[eMSDatagram.messageLength + EMS_DATAGRAM_OVERHEAD];

	// get an EMS Buffer with the parameters requested. Length is 1 (byte) and offset is the position of the data type in the EMSBuffer
	boolean operationStatus = getEMSBuffer(inEMSBuffer, eMSDatagram, 1, calduinoData.offset);

	if (operationStatus)
	{
		result = calduinoData.decodeBitValue(inEMSBuffer);
	}

	return result;
}


/**
 * Get a Calduino Data of type Switch Point.
 *
 * @param	selProgram   	Identifier of the Calduino Data Bit requested.
 * @param	switchPointID	Identifier for the switch point.
 *
 * @return	The value of the Calduino Data requested, false otherwise.
 */

SwitchPoint Calduino::getCalduinoSwitchPoint(EMSDatagramID selProgram, byte switchPointID)
{
	SwitchPoint result;
	
	if (((selProgram == EMSDatagramID::Program_1_HC_1) || (selProgram == EMSDatagramID::Program_2_HC_1) ||
		(selProgram == EMSDatagramID::Program_1_HC_2) || (selProgram == EMSDatagramID::Program_2_HC_2) ||
		(selProgram == EMSDatagramID::Program_1_HC_3) || (selProgram == EMSDatagramID::Program_2_HC_3) ||
		(selProgram == EMSDatagramID::Program_1_HC_4) || (selProgram == EMSDatagramID::Program_2_HC_4) ||
		(selProgram == EMSDatagramID::Program_DHW) || (selProgram == EMSDatagramID::Program_Pump_DHW)) &&
		(switchPointID < SWITCHING_POINTS))
	{
		// get from program memory the EMSDatagram
		EMSDatagram eMSDatagram;
		memcpy_P(&eMSDatagram, eMSDatagramIDs[selProgram], sizeof(EMSDatagram));

		// get from program memory the CalduinoData 
		CalduinoData calduinoData;
		memcpy_P(&calduinoData, &eMSDatagram.data[switchPointID], sizeof(CalduinoData));

		// buffer where the EMS Datagram will be saved (size is message size plus EMS_DATAGRAM_OVERHEAD bytes to store the headers, CRC and break)
		byte inEMSBuffer[eMSDatagram.messageLength + EMS_DATAGRAM_OVERHEAD];

		// get an EMS Buffer with the parameters requested. Length is 2 (bytes) and offset is the position of the data type in the EMSBuffer
		boolean operationStatus = getEMSBuffer(inEMSBuffer, eMSDatagram, 2, calduinoData.offset);

		if (operationStatus)
		{
			result = calduinoData.decodeSwitchPoint(inEMSBuffer);
		}
	}

	return result;
}

/**
 * Get the EMS Datagram passed as parameter by sending a get EMS command and parsing the bytes
 * obtained. If datagramDataIndex is ERROR_VALUE, get the whole datagram. Get only the Data
 * Index otherwise. If debug is activated the results will be sent to the Debug Serial Stream
 * following the print format activated.
 *
 * @param	eMSDatagramID	 	- The EMS Datagram ID to be obtained.
 * @param	datagramDataIndex	- (Optional) If not ERROR_VALUE, the position that the data to be
 * 								recovered occupies in the calduinoDataValues array.
 *
 * @return	True if it succeeds, false otherwise.
 */

boolean Calduino::printEMSDatagram(EMSDatagramID eMSDatagramID, DatagramDataIndex datagramDataIndex = ERROR_VALUE)
{
	// buffer where the char buffer will be saved
	char textBuffer[TEXT_BUFFER_SIZE];

	// get from program memory the EMS Datagram passed as parameter
	EMSDatagram eMSDatagram;
	memcpy_P(&eMSDatagram, eMSDatagramIDs[eMSDatagramID], sizeof(EMSDatagram));

	// print the EMS Datagram Header Tag
	eMSDatagram.printMessageName(textBuffer, true, printFormat);
	DPRINTLN(textBuffer);

	// if only one data is requested, obtain the calduinoData
	CalduinoData calduinoData;
	if (datagramDataIndex != ERROR_VALUE)
	{
		memcpy_P(&calduinoData, &eMSDatagram.data[datagramDataIndex], sizeof(CalduinoData));
	}
	
	// buffer where the EMS Datagram will be saved (size is message size plus 5 bytes to store the headers, CRC and break)
	byte inEMSBuffer[eMSDatagram.messageLength + EMS_DATAGRAM_OVERHEAD];

	// launch the get EMS Buffer operation. Depending on the datagramDataIndex value it will
	// require the whole datagram (length = 0) or just 3 bytes (maximum size of a Data Type). 
	boolean operationStatus = getEMSBuffer(inEMSBuffer, eMSDatagram, (datagramDataIndex == ERROR_VALUE ? 0 : 3), (datagramDataIndex == ERROR_VALUE ? 0 : calduinoData.offset));

	if (operationStatus)
	{
		// Print all the values of the datagram or just the requested, depending on datagramDataIndex
		// value. 
		if (datagramDataIndex == ERROR_VALUE)
		{
			// decode and print each value contained in the EMS Datagram
			for (int i = 0; i < eMSDatagram.dataSize; i++)
			{
				memcpy_P(&calduinoData, &eMSDatagram.data[i], sizeof(CalduinoData));

				char value[15];
				calduinoData.decodeValue(inEMSBuffer, value);
				calduinoData.printfValue(textBuffer, value, printFormat);
				DPRINTLN(textBuffer);
			}
		}
		else
		{
			// decode and print only the value requested
			char value[15];
			calduinoData.decodeValue(inEMSBuffer, value);
			calduinoData.printfValue(textBuffer, value, printFormat);
			DPRINTLN(textBuffer);

		}

	}
	else
	{
		// print the EMS Datagram Error Tag
		eMSDatagram.printErrorTag(textBuffer, printFormat);
		DPRINTLN(textBuffer);
	}

	// print the EMS Datagram Tail Tag
	eMSDatagram.printMessageName(textBuffer, false, printFormat);
	DPRINTLN(textBuffer);

	return operationStatus;
}


/**
 * Send a Set Command updating the value of the data situaded in CalduinoDataValuesIndex of
 * eMSDatagramID. If debug is activated the result of the operation will be sent to the Debug
 * Serial Stream following the selected print format.
 *
 * @param	eMSDatagramID	 	- The EMSDatagram to be updated.
 * @param	calduinoDataIndex	- The position that the value to be updated occupies in the
 * 									calduinoDataValues array.
 * @param	data			 	- The data/value to be assigned.
 * @param	extraOffset		 	(Optional) - An extra offset to apply to the EMS Command offset,
 * 								in case this data type uses two or more bytes (Switch Point).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::updateEMSDatagram(EMSDatagramID eMSDatagramID, DatagramDataIndex datagramDataIndex, byte data, byte extraOffset = 0)
{
	boolean operationStatus = false;

	// buffer where the char buffer str will be saved
	char textBuffer[TEXT_BUFFER_SIZE];

	// get from program memory the EMS Datagram passed as parameter
	EMSDatagram eMSDatagram;
	memcpy_P(&eMSDatagram, eMSDatagramIDs[eMSDatagramID], sizeof(EMSDatagram));

	// print the EMS Datagram Header Tag
	eMSDatagram.printMessageName(textBuffer, true, printFormat);
	DPRINTLN(textBuffer);

	// get from program memory the Calduino Data  of the configuration to be changed 
	CalduinoData calduinoData;
	memcpy_P(&calduinoData, &eMSDatagram.data[datagramDataIndex], sizeof(CalduinoData));

	// get the EMS Datagram Bytes, repeat operation if failed until timeout
	unsigned long timeout = millis() + EMSMaxWaitTime * RETRY_FACTOR;

	// get the EMS Datagram Bytes, repeat operation if failed until timeout
	do
	{
		operationStatus = setEMSCommand(eMSDatagram.destinationID, eMSDatagram.messageID, calduinoData.offset - INITIAL_OFFSET + extraOffset, data);
	} while ((millis() < timeout) && (!operationStatus));

	// if success and debug activated, print the set value
	if (operationStatus)
	{
		char value[10];
		// consider floats as signed values
		sprintf_P(value, decimal, (calduinoData.encodeType == CalduinoEncodeType::Byte ? (uint8_t)data : (int8_t)data));
		calduinoData.printfValue(textBuffer, value, printFormat);
		DPRINTLN(textBuffer);
	}
	else
	{
		// print the EMS Datagram Error Tag
		eMSDatagram.printErrorTag(textBuffer, printFormat);
		DPRINTLN(textBuffer);
	}

	// print the EMS Datagram Tail Tag
	eMSDatagram.printMessageName(textBuffer, false, printFormat);
	DPRINTLN(textBuffer);

	return operationStatus;
}


/**
 * Send an EMS command to set the selected RC35 heating circuit to the desired Mode
 *
 * @param	selHC  	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @param	selMode	The selected working mode to be configured (0-night, 1-day, 2-auto/program).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setWorkModeHC(byte selHC, byte selMode)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	// evaluate if the selected mode exists
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT) && (selMode <= MAX_WORKING_MODE))
	{
		// Working Mode is in position 5 of eMSDatagram.Values array in Working_Mode_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_HC_1 + (selHC - 1) * 4, 5, selMode);
	}

	return operationStatus;
}


/**
 * Send an EMS command to set in the selected RC35 heating circuit and Mode, the desired
 * temperature.
 *
 * @param	selHC  	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @param	selMode	The working mode to be configured (0-night, 1-day, 2-holidays).
 * @param	selTmp 	The desired temperature (multiplied by two, originally in increments of 0,5).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setTemperatureHC(byte selHC, byte selMode, byte selTmp)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	// evaluate if the selected mode exists
	// evaluate if the selected temperature is inside limits
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT) && (selMode <= MAX_WORKING_MODE) && (selTmp <= MAX_TEMPERATURE * 2) && (selTmp >= MIN_TEMPERATURE * 2))
	{
		// Data Type SelNight/Day/Holi Temp is in position 0 to 2 (selMode) of EMS Datagram Values array in EMS Datagram Working_Mode_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_HC_1 + (selHC - 1) * 4, selMode, selTmp);
	}

	return operationStatus;
}


/**
 * Send an EMS command to set the selected RC35 heating circuit to the desired program.
 * @selHC the heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @selProgram the working program to set (0x00 = User 1, 0x01 = Family, 0x02 = Morning, 0x03 =
 * Early morning, 0x04 = Evening, 0x05 = Midmorning, 0x06 = Afternoon, 0x07 = Midday, 0x08 =
 * Single, 0x09 = Senioren, 0x0A User2).
 *
 * @param	selHC	  	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @param	selProgram	The the working program to set (0x00 = User 1, 0x01 = Family, 0x02 =
 * 						Morning, 0x03 = Early morning, 0x04 = Evening, 0x05 = Midmorning, 0x06 =
 * 						Afternoon, 0x07 = Midday, 0x08 = Single, 0x09 = Senioren, 0x0A User2).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setProgramHC(byte selHC, byte selProgram)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	// evaluate if the selected program exists
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT) && (selProgram <= MAX_PROGRAM))
	{
		// Data Type programName is in position 42 of eMSDatagram.Values array in EMS Datagram Program_1_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::programNameIdx, selProgram);
	}

	return operationStatus;
}


/**
 * Send an EMS command to configure the temperature for summer / winter threshold switchpoint in
 * the selected heating circuit.
 *
 * @param	selHC 	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @param	selTmp	The desired Summer / Winter Threshold temperature.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setSWThresholdTempHC(byte selHC, byte selTmp)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	// evaluate if the selected program exists
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT) && (selTmp <= MAX_SUMMER_WINTER_THRESHOLD) && (selTmp >= MIN_SUMMER_WINTER_THRESHOLD))
	{
		// Data Type sWThresTempHC is in position 6 of eMSDatagram.Values array in EMS Datagram Working_Mode_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_HC_1 + (selHC - 1) * 4, DatagramDataIndex::sWThresTempHCIdx, selTmp);
	}

	return operationStatus;
}


/**
 * Send an EMS command to configure the night setback mode in the selected heating circuit.
 *
 * @param	selHC  	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @param	selMode	The desired setback mode (0 - Shutdown, 1 - Reduced Operation, 2 - Room
 * 					Setback, 3 - Outdoor Setback).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setNightSetbackModeHC(byte selHC, byte selMode)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	// evaluate if the selected program exists
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT) && (selMode <= MAX_SETBACK_MODE))
	{
		// Data Type nightSetbackMode is in position 7 of eMSDatagram.Values array in EMS Datagram Working_Mode_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_HC_1 + (selHC - 1) * 4, DatagramDataIndex::nightSetbackHCIdx, selMode);
	}

	return operationStatus;
}


/**
 * Send an EMS command to configure the night setback outside temperature threshold in the
 * selected heating circuit. This temperature is only considered if night setback mode is set to
 * 3 (Outdoor Setback).
 *
 * @param	selHC 	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @param	selTmp	The desired outside temperature to switch between shutdown and reduced mode.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setNightThresholdOutTempHC(byte selHC, int8_t selTmp)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	// evaluate if the temperature is inside limits
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT) && (selTmp <= MAX_OUT_NIGHT_THRESHOLD) && (selTmp >= MIN_OUT_NIGHT_THRESHOLD))
	{
		// Data Type sWThresTempHC is in position 8 of eMSDatagram.Values array in EMS Datagram Working_Mode_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_HC_1 + (selHC - 1) * 4, DatagramDataIndex::nightOutTempHCIdx, selTmp);
	}

	return operationStatus;
}


/**
 * Send an EMS command to configure the room temperature offset in the selected heating circuit.
 * This value shifts left and right the heating characteristic curve.
 *
 * @param	selHC 	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @param	selTmp	The desired offset temperature.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setRoomTempOffsetHC(byte selHC, int8_t selTmp)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	// evaluate if the temperature is inside limits
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT) && (selTmp <= MAX_ROOM_TEMPERATURE_OFFSET * 2) && (selTmp >= MIN_ROOM_TEMPERATURE_OFFSET * 2))
	{
		// Data Type roomTempOffHC is in position 4 of eMSDatagram.Values array in EMS Datagram Working_Mode_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_HC_1 + (selHC - 1) * 4, DatagramDataIndex::roomTempOffHCIdx, selTmp);
	}

	return operationStatus;
}


/**
 * Send an EMS command to configure pause mode in duration hours in the selected heating circuit.
 *
 * @param	selHC   	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @param	duration	The desired duration of the pause mode.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setPauseModeHC(byte selHC, byte duration)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT))
	{
		// Data Type pauseTime is in position 43 of eMSDatagram.Values array in EMS Datagram Program_1_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::pauseTimeIdx, duration);
	}

	return operationStatus;
}


/**
 * Send an EMS command to configure party mode in duration hours in the selected heating circuit.
 * @selHC the heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @duration the desired duration of the party mode.
 *
 * @param	selHC   	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4 is HC4).
 * @param	duration	The desired duration of the party mode.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setPartyModeHC(byte selHC, byte duration)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT))
	{
		// Data Type party is in position 44 of eMSDatagram.Values array in EMS Datagram Program_1_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::partyTimeIdx, duration);
	}

	return operationStatus;
}


/**
 * Send an EMS command to configure Holidays mode between start and end daySwitchPoint.
 *
 * @param	selHC			  	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3 and 4
 * 								is HC4).
 * @param	startHolidayDay   	The start holiday day.
 * @param	startHoldidayMonth	The start holdiday month.
 * @param	startHolidayYear  	The start holiday year.
 * @param	endHolidayDay	  	The end holiday dat.
 * @param	endHoldidayMonth  	The end holdiday month.
 * @param	endHolidayYear	  	The end holiday year.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setHolidayModeHC(byte selHC, byte startHolidayDay, byte startHoldidayMonth, byte startHolidayYear, byte endHolidayDay, byte endHoldidayMonth, byte endHolidayYear)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	// evaluate if the selected daySwitchPoint and month exists
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT) && (startHolidayDay <= MAX_DAY) && (endHolidayDay <= MAX_DAY) && (startHoldidayMonth <= MAX_MONTH) && (endHoldidayMonth <= MAX_MONTH))
	{
		// Data Type startHolidayDay is in position 45, startHoldidayMonth 46, startHolidayYear 47 of eMSDatagram.Values array in EMS Datagram Program_1_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::startHolidayDayIdx, startHolidayDay);
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::startHolidayMonthIdx, startHoldidayMonth);
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::startHolidayYearIdx, startHolidayYear);

		// Data Type endHolidayDay is in position 48, endHoldidayMonth 49, endHolidayYear 50 of eMSDatagram.Values array in EMS Datagram Program_1_HC_selHC
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::endHolidayDayIdx, endHolidayDay);
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::endHolidayMonthIdx, endHoldidayMonth);
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::endHolidayYearIdx, endHolidayYear);
	}

	return operationStatus;
}


/**
 * Send an EMS command to configure Home Holiday mode (saturday configuration) between start and
 * end daySwitchPoint.
 *
 * @param	selHC				  	The heating circuit to modify (1 is HC1, 2 is HC2, 3 is HC3
 * 									and 4 is HC4).
 * @param	startHomeHolidayDay   	The start home holiday day.
 * @param	startHomeHoldidayMonth	The start home holdiday month.
 * @param	startHomeHolidayYear  	The start home holiday year.
 * @param	endHomeHolidayDay	  	The end home holiday.
 * @param	endHomeHoldidayMonth  	The end home holdiday month.
 * @param	endHomeHolidayYear	  	The end home holiday year.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setHomeHolidayModeHC(byte selHC, byte startHomeHolidayDay, byte startHomeHoldidayMonth, byte startHomeHolidayYear, byte endHomeHolidayDay, byte endHomeHoldidayMonth, byte endHomeHolidayYear)
{
	boolean operationStatus = false;

	// evaluate if the selected heating circuit exists
	// evaluate if the selected daySwitchPoint and month exists
	if ((selHC > 0) && (selHC <= MAX_HC_CIRCUIT) && (startHomeHolidayDay <= MAX_DAY) && (endHomeHolidayDay <= MAX_DAY) && (startHomeHoldidayMonth <= MAX_MONTH) && (endHomeHoldidayMonth <= MAX_MONTH))
	{
		// Data Type startHomeHolidayDay is in position 51, startHomeHoldidayMonth 52, startHomeHolidayYear 53 of eMSDatagram.Values array in EMS Datagram Program_1_HC_selHC
		operationStatus = updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::startHomeHolidayDayIdx, startHomeHolidayDay);
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::startHomeHolidayMonthIdx, startHomeHoldidayMonth);
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::startHomeHolidayYearIdx, startHomeHolidayYear);

		// Data Type endHomeHolidayDay is in position 54, endHomeHoldidayMonth 55, endHomeHolidayYear 56 of eMSDatagram.Values array in EMS Datagram Program_1_HC_selHC
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::endHomeHolidayDayIdx, endHomeHolidayDay);
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::endHomeHolidayMonthIdx, endHomeHoldidayMonth);
		operationStatus &= updateEMSDatagram(EMSDatagramID::Program_1_HC_1 + (selHC - 1) * 4, DatagramDataIndex::endHomeHolidayYearIdx, endHomeHolidayYear);
	}

	return operationStatus;
}


/**
 * Send an EMS command to set the DHW working mode.
 * @selMode the working mode to be configured (0-off, 1-on, 2-auto)
 *
 * @param	selMode	The working mode to be configured (0-off, 1-on, 2-auto).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setWorkModeDHW(byte selMode)
{
	boolean operationStatus = false;

	// evaluate if the selected mode exists
	if (selMode <= MAX_WORKING_MODE)
	{
		// Working Mode is in position 2 of eMSDatagram.Values array in Working_Mode_DHW
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_DHW, 2, selMode);
	}

	return operationStatus;
}


/**
 * Send an EMS command to set the DHW Pump working mode.
 *
 * @param	selMode	The working mode to be configured (0-off, 1-on, 2-auto)
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setWorkModePumpDHW(byte selMode)
{
	boolean operationStatus = false;

	// evaluate if the selected mode exists
	if (selMode <= MAX_WORKING_MODE)
	{
		// Working Mode is in position 3 of eMSDatagram.Values array in Working_Mode_DHW
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_DHW, DatagramDataIndex::workModePumpDHWIdx, selMode);
	}

	return operationStatus;
}


/**
 * Send an EMS command to set the desired temperature in the Domestic How Water System.
 *
 * @param	selTmp	The desired temperature to be configured.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setTemperatureDHW(byte selTmp)
{
	boolean operationStatus = false;

	// evaluate if the selected temperature is inside limits
	if ((selTmp <= MAX_DHW_TEMPERATURE) && (selTmp >= MIN_DHW_TEMPERATURE))
	{
		// Data Type selTempDHW is in position 0 of EMS Datagram Values array in EMS Datagram UBA_Parameter_DHW
		operationStatus = updateEMSDatagram(EMSDatagramID::UBA_Parameter_DHW, DatagramDataIndex::selTempDHWIdx, selTmp);
	}

	return operationStatus;
}


/**
 * Send an EMS command to set the desired disinfection temperature in the Domestic How Water
 * System.
 *
 * @param	selTmp	The desired temperature to be configured.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setTemperatureTDDHW(byte selTmp)
{
	boolean operationStatus = false;

	// evaluate if the selected temperature is inside limits
	if ((selTmp <= MAX_DHW_TEMPERATURE) && (selTmp >= MIN_DHW_TEMPERATURE))
	{
		// Data Type disinfTempDHW is in position 1 of EMS Datagram Values array in EMS Datagram UBA_Parameter_DHW
		operationStatus = updateEMSDatagram(EMSDatagramID::UBA_Parameter_DHW, DatagramDataIndex::tempTDDHWIdx, selTmp);
	}

	return operationStatus;
}


/**
 * Send an EMS command to change the DHW program.
 *
 * @param	selProgram	The working program to be set (0 = like the heating circuit, 255 = own
 * 						program).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setProgramDHW(byte selProgram)
{
	boolean operationStatus = false;

	// evaluate if the selected mode exists (0 or 255)
	if ((selProgram == 0) || (selProgram == ERROR_VALUE))
	{
		// progDHW is in position 0 of eMSDatagram.Values array in Working_Mode_DHW
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_DHW, DatagramDataIndex::progDHWIdx, selProgram);
	}

	return operationStatus;
}


/**
 * Send an EMS command to change the Pump DHW program.
 *
 * @param	selProgram	The working program to be set (0 = like the heating circuit, 255 = own
 * 						program).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setProgramPumpDHW(byte selProgram)
{
	boolean operationStatus = false;

	// evaluate if the selected mode exists (0 or 255)
	if ((selProgram == 0) || (selProgram == ERROR_VALUE))
	{
		// progPumpDHW is in position 1 of eMSDatagram.Values array in Working_Mode_DHW
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_DHW, DatagramDataIndex::progPumpDHWIdx, selProgram);
	}

	return operationStatus;
}


/**
 * Send an EMS command to change the thermal disinfection DHW working mode.
 *
 * @param	selMode	255 to enable thermal disinfection DHW function, 0 to disable it.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setWorkModeTDDHW(byte selMode)
{
	boolean operationStatus = false;
	
	// evaluate if the selected mode exists (0 or 255)
	if ((selMode == 0) || (selMode == ERROR_VALUE))
	{
		// workModeTDDHW is in position 4 of eMSDatagram.Values array in Working_Mode_DHW
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_DHW, DatagramDataIndex::workModeTDDHWIdx, selMode);
	}
	return operationStatus;
}


/**
 * Send an EMS command to change the thermal disinfection DHW running day. It will run once per week
 * at the selected daySwitchPoint and hour, or once per day if dayTDDHW is 7.
 *
 * @param	dayTDDHW 	Day of the week to run the thermal disinfection (7 is everyday).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setDayTDDHW(byte dayTDDHW)
{
	boolean operationStatus = false;

	// evaluate if the selected dayTDDHW is correct
	if (dayTDDHW <= MAX_DAY_WEEK)
	{
		// dayTDDHW is in position 5 of eMSDatagram.Values array in Working_Mode_DHW
		operationStatus = updateEMSDatagram(EMSDatagramID::Working_Mode_DHW, DatagramDataIndex::dayTDDHWIdx, dayTDDHW);
	}

	return operationStatus;
}

/**
* Send an EMS command to change the thermal disinfection DHW program. It will run once per week
* at the selected daySwitchPoint and hour, or once per day if dayTDDHW is 7.
*
* @param	hourTDDHW	Hour of the week to run the thermal disinfection.
*
* @return	True if it succeeds, false if it fails.
*/

boolean Calduino::setHourTDDHW(byte hourTDDHW)
{
	boolean operationStatus = false;

	// evaluate if the selected hourTDDHW is2 correct
	if (hourTDDHW < MAX_HOUR_DAY)
	{
		// desHourDHW is in position 6 of eMSDatagram.Values array in Working_Mode_DHW
		operationStatus &= updateEMSDatagram(EMSDatagramID::Working_Mode_DHW, DatagramDataIndex::hourTDDHWIdx, hourTDDHW);
	}

	return operationStatus;
}


/**
 * Send an EMS command to change an specific switch point in the program passed as parameter.
 *
 * @param	selProgram				The selected program to be updated.
 * @param	switchPointID			Identifier for the switch point.
 * @param	operationSwitchPoint	The operation to be performed in the switch point (0 -
 * 									off/night, 1 - on/day, 7 - undefined).
 * @param	daySwitchPoint			Day of the week (0 - monday, ..., 6 - sunday).
 * @param	hourSwitchPoint			Hour of the day (0 to 23).
 * @param	minuteSwitchPoint   	Minute (The minimum time between switching points is 10 min).
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setProgramSwitchPoint(EMSDatagramID selProgram, byte switchPointID, byte operationSwitchPoint, byte daySwitchPoint, byte hourSwitchPoint, byte minuteSwitchPoint)
{
	boolean operationStatus = false;

	// evaluate if the selected selProgram is correct
	// evaluate if the switch point is inside limits
	// evaluate if the operationSwitchPoint is correct
	// evaluate if the daySwitchPoint time is correct
	if (((selProgram == EMSDatagramID::Program_DHW) || (selProgram == EMSDatagramID::Program_Pump_DHW) ||
		(selProgram == EMSDatagramID::Program_1_HC_1) || (selProgram == EMSDatagramID::Program_2_HC_1) ||
		(selProgram == EMSDatagramID::Program_1_HC_2) || (selProgram == EMSDatagramID::Program_2_HC_2) ||
		(selProgram == EMSDatagramID::Program_1_HC_3) || (selProgram == EMSDatagramID::Program_2_HC_3) ||
		(selProgram == EMSDatagramID::Program_1_HC_4) || (selProgram == EMSDatagramID::Program_2_HC_4)) &&
		(switchPointID < SWITCHING_POINTS) &&
		((operationSwitchPoint == 0) || (operationSwitchPoint == 1) || (operationSwitchPoint == 7)) &&
		(daySwitchPoint < MAX_DAY_WEEK) &&
		(hourSwitchPoint < MAX_HOUR_DAY) &&
		((minuteSwitchPoint < MAX_MINUTE_HOUR) && (minuteSwitchPoint % 10 == 0)))
	{
		byte byte1 = (operationSwitchPoint == 7) ? 0xE7 : (0x00 | (daySwitchPoint << 5) | (operationSwitchPoint));
		byte byte2 = (operationSwitchPoint == 7) ? 0x90 : ((hourSwitchPoint * 6) + (minuteSwitchPoint / 10));
		
		operationStatus = updateEMSDatagram(selProgram, switchPointID, byte1);
		operationStatus = updateEMSDatagram(selProgram, switchPointID, byte2 , 1);
	}

	return operationStatus;
}


/**
 * Send an EMS command to set the warm watter one time function on or off.
 *
 * @param	selMode	True to enable one time function, false to disable it.
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean Calduino::setOneTimeDHW(boolean selMode)
{
	boolean operationStatus;

	operationStatus = updateEMSDatagram(EMSDatagramID::Flags_DHW, DatagramDataIndex::oneTimeDHW2Idx, selMode ? DHW_ONETIME_ON: DHW_ONETIME_OFF);
	operationStatus |= getCalduinoBitValue(BitRequest::oneTimeDHW_t);
	return operationStatus;
}

#pragma endregion Calduino


