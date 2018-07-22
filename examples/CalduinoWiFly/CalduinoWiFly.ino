/**
*	Name:		CalduinoWiFly.ino
*	Created:	23/04/2018
*	Author:		dani.macias.perea@gmail.com
*	Last edit:  07/05/2018
*	
*	Hardware:	
*	- Arduino Mega 2560  
*	- EMS Bus – UART Interface Circuit  
*	- WiFly RN 171 XV*	
*
*	UART ports used:
*	- UART_0 (EMSSerial0) -> Debug
*	- UART_2 (EMSSerial2) -> WiFly
*	- UART_3 (EMSSerial3) -> Calduino
*
*	Get Commands (matches EMSDatagramID enumeration)
*	-	RC_Datetime					calduino/?op=00
*	-	UBA_Working_Time			calduino/?op=01
*	-	UBA_Monitor_Fast			calduino/?op=02
*	-	UBA_Monitor_Slow			calduino/?op=03
*	-	UBA_Parameter_DHW			calduino/?op=04
*	-	UBA_Monitor_DHW				calduino/?op=05
*	-	Flags_DHW					calduino/?op=06
*	-	Working_Mode_DHW			calduino/?op=07
*	-	Program_DHW					calduino/?op=08
*	-	Program_Pump_DHW			calduino/?op=09
*	-	Working_Mode_HC_1			calduino/?op=10
*	-	Monitor_HC_1				calduino/?op=11
*	-	Program_1_HC_1				calduino/?op=12
*	-	Program_2_HC_1				calduino/?op=13
*	-	Working_Mode_HC_2			calduino/?op=14
*	-	Monitor_HC_2				calduino/?op=15
*	-	Program_1_HC_2				calduino/?op=16
*	-	Program_2_HC_2				calduino/?op=17
*	-	Working_Mode_HC_3			calduino/?op=18
*	-	Monitor_HC_3				calduino/?op=19
*	-	Program_1_HC_3				calduino/?op=20
*	-	Program_2_HC_3				calduino/?op=21
*	-	Working_Mode_HC_4			calduino/?op=22
*	-	Monitor_HC_4				calduino/?op=23
*	-	Program_1_HC_4				calduino/?op=24
*	-	Program_2_HC_4				calduino/?op=25
*	-	Monitor_MM_10				calduino/?op=26
*	-	All_Monitors				calduino/?op=29
*	
*	Set Commands
*	-	Set Working Mode HC			calduino/?op=30&hc=X&wm=Y		hc - Heating Circuit 1/2/3/4	wm - Working Mode 0 = night/ 1 = day/ 2 = auto  
*	-	Set Temperature HC			calduino/?op=31&hc=X&wm=Y&tp=ZZ	hc - Heating Circuit 1/2/3/4	wm - Working Mode 0 = night/ 1 = day/ 2 = auto	tp - Desired Temperature x 2  from 6°C*2 to 29°C*2 with 0,5 °C increments
*	-	Set Program HC				calduino/?op=32&hc=X&pr=YY		hc - Heating Circuit 1/2/3/4	pr - Program 00 = User 1/ 0x01 = Family / 0x02 = Morning / 0x03 = Early morning / 0x04 = Evening / 0x05 = Midmorning / 0x06 = Afternoon / 0x07 = Midday / 0x08 = Single / 0x09 = Senioren / 0x0A User2  
*	-	Set S/W Threshold HC		calduino/?op=33&hc=X&tp=ZZ		hc - Heating Circuit 1/2/3/4	tp - Desired Temperature from 6°C to 29°C with 0,5 °C increments 
*	-	Set Night Setback Mode HC	calduino/?op=34&hc=X&wm=Y		hc - Heating Circuit 1/2/3/4	wm - Working Mode 0 = Shutdown / 1 = Reduced Operation / 2 = Room Setback / 3 = Outdoor Setback  
*	-	Set Out Night Thr Temp HC	calduino/?op=35&hc=X&tp=(Z)ZZ	hc - Heating Circuit 1/2/3/4	tp - Desired Temperature from -20°C to 10°C with 1 °C increments  
*	-	Set Room Temp Offset HC		calduino/?op=36&hc=X&tp=(Z)ZZ	hc - Heating Circuit 1/2/3/4	tp - Desired Offset x 2 from -5°C*2 to 5°C*2 with 0,5 °C increments   
*	-	Set Pause Mode				calduino/?op=37&hc=X&h=YY		hc - Heating Circuit 1/2/3/4	h - Desired Hours  
*	-	Set Party Mode				calduino/?op=38&&hc=X&h=YY		hc - Heating Circuit 1/2/3/4	h - Desired Hours  
*	-	Set Holiday Mode HC			calduino/?op=39&hc=X&sd=DD&ed=DD&sm=MM&em=MM&sy=YY&ey=YY	hc - Heating Circuit 1/2/3/4	sd/ed - Starting/Ending day	sm/em - Starting/Ending month sy/ey - Starting/Ending Year  
*	-	Set Holiday Home Mode HC	calduino/?op=40&hc=X&sd=DD&ed=DD&sm=MM&em=MM&sy=YY&ey=YY	hc - Heating Circuit 1/2/3/4	sd/ed - Starting/Ending day	sm/em - Starting/Ending month sy/ey - Starting/Ending Year  
*	-	Set Work Mode DHW			calduino/?op=41&wm=Y			wm - Working Mode 0 = off/ 1 = on/ 2 = auto  
*	-	Set Work Mode Pump DHW		calduino/?op=42&wm=Y			wm - Working Mode 0 = off/ 1 = on/ 2 = auto  
*	-	Set Temperature DHW			calduino/?op=43&tp=ZZ			tp - Desired Temperature from 30 to 80 with 1 °C increments  
*	-	Set Temperature TD DHW		calduino/?op=44&tp=ZZ			tp - Desired Temperature from 30 to 80 with 1 °C increments  
*	-	Set Program DHW				calduino/?op=45&pr=YYY			pr - Program 000 = Like HC1 / 255 = Own Program  
*	-	Set Program Pump DHW		calduino/?op=46&pr=YYY			pr - Program 000 = Like HC1 / 255 = Own Program  
*	-	Set One Time DHW			calduino/?op=47&wm=0			wm - Working Mode 0 = off/ 1 = on  
*	-	Set Working Mode TD DHW		calduino/?op=48&wm=YYY			wm - Working Mode 000 = off/ 255 = on  
*	-	Set Day TD DHW				calduino/?op=49&d=D					d - Thermal disinfection day (0 - Monday, 1 - Tuesday, ... , 7 - Everyday)  
*	-	Set Hou TD DHW				calduino/?op=50&h=HH			h - Thermal disinfection starting hour
*	-	Set Program Switch Point	calduino/?op=51&pr=YY&sp=XX&e=Z&d=D&h=HH&m=MM				pr - Program matches EMSDatagramID enumeration	sp - Switch Point Number 0 to 41	op = 0 off/ 1 = on/ 7 = undef	d = day 0=monday,...,6=sunday  
*
*	Calduino Commands
*	-	Get Calduino stats basic	calduino/?op=60  
*	-	Get Calduino stats advan	calduino/?op=61  
*	-	Restart WiFly module		calduino/?op=70
* 
*/

#include <WiFlyHQ.h>
#include <Calduino.h>

/* Debugging functions */
#undef DEBUG

#ifdef DEBUG
#define DPRINT(item) EMSSerial0.print(item)
#define DPRINTLN(item) EMSSerial0.println(item)
#define DPRINTVALUE(item1, item2) EMSSerial0.print(item1);EMSSerial0.print(": ");EMSSerial0.println(item2)
#else
#define DPRINT(item)
#define DPRINTLN(item)
#define DPRINTVALUE(item1, item2)
#endif

#define SEND_WIFLY_XML_S(item1, item2, item3) snprintf_P(item3, sizeof(item3), PSTR("<%S>%s</%S>"), item1, item2, item1); wifly.println(item3);
#define SEND_WIFLY_XML_D(item1, item2, item3) snprintf_P(item3, sizeof(item3), PSTR("<%S>%d</%S>"), item1, item2, item1); wifly.println(item3);
#define SEND_WIFLY_XML_UL(item1, item2, item3) snprintf_P(item3, sizeof(item3), PSTR("<%S>%lu</%S>"), item1, item2, item1); wifly.println(item3);


#define RESET_PIN						40			///< Pin to reset the WiFly module
#define WIFLY_UART_RATE					9600		///< Wifly UART rate
#define DEBUG_UART_RATE					9600		///< Debug UART rate
#define EMS_BUS_UART_RATE				9700		///< EMS Bus - UART Interface rate
#define MAIN_LOOP_WAIT_TIME				1000		///< Wait time between loops
#define NO_OPERATION					0xFF			
#define HTTP_BUFFER_SIZE				80
#define CALDUINO_FULL_STATISTICS		1

#define GET_ALL_MONITORS				29

#define SET_WORK_MODE_HC				30
#define SET_TEMPERATURE_HC				31
#define SET_PROGRAM_HC					32
#define SET_SW_THRESHOLD_TEMP_HC		33
#define SET_NIGHT_SETBACK_MODE_HC		34
#define SET_NIGHT_THRESHOLD_OUT_TEMP_HC	35
#define SET_ROOM_TEMP_OFFSET_HC			36
#define SET_PAUSE_MODE_HC				37
#define SET_PARTY_MODE_HC				38
#define SET_HOLIDAY_MODE_HC				39
#define SET_HOME_HOLIDAY_MODE_HC		40
#define SET_WORK_MODE_DHW				41
#define SET_WORK_MODE_PUMP_DHW			42
#define SET_TEMPERATURE_DHW				43
#define SET_TEMPERATURE_TD_DHW			44
#define SET_PROGRAM_DHW					45
#define SET_PROGRAM_PUMP_DHW			46
#define SET_ONETIME_DHW					47
#define SET_WORK_MODE_TD_DHW			48
#define SET_DAY_TD_DHW					49
#define SET_HOUR_TD_DHW					50
#define SET_PROGRAM_SWITCH_POINT		51

#define GET_CALDUINO_BASIC				60
#define GET_CALDUINO_FULL				61

#define RESTART_WIFLY					70

Calduino calduino;
WiFly wifly;

prog_char wiFlyDeviceId[] = "Calduino";
prog_char op[] = "?op=";
prog_char hc[] = "&hc=";
prog_char wm[] = "&wm=";
prog_char tp[] = "&tp=";
prog_char pr[] = "&pr=";
prog_char e[] = "&e=";
prog_char d[] = "&d=";
prog_char h[] = "&h=";
prog_char m[] = "&m=";
prog_char sd[] = "&sd=";
prog_char sm[] = "&sm=";
prog_char sy[] = "&sy=";
prog_char ed[] = "&ed=";
prog_char em[] = "&em=";
prog_char ey[] = "&ey=";
prog_char sp[] = "&sp=";

char mySSID[] = "";
char myPassword[] = "";

unsigned int operationsOK = 0;
unsigned int operationsNOK = 0;


/**
 * Send the stats of Calduino in XML format via WiFly module
 *
 * @param	mode	0 = full statistics, 1 = basic data.
 *
 * @return	whether the operation has been correctly executed or not.
 */

boolean getCalduinoStats(byte mode)
{
	char printXMLWiFly[SERIAL_BUFFER_SIZE];
	
	wifly.println(F("<Calduino>"));
	if (mode == CALDUINO_FULL_STATISTICS)
	{
		char auxBuffer[20];
		SEND_WIFLY_XML_S(F("MAC"), wifly.getMAC(auxBuffer, sizeof(auxBuffer)), printXMLWiFly);
		SEND_WIFLY_XML_S(F("IP"), wifly.getIP(auxBuffer, sizeof(auxBuffer)), printXMLWiFly);
		SEND_WIFLY_XML_S(F("Gateway"), wifly.getGateway(auxBuffer, sizeof(auxBuffer)), printXMLWiFly);
		SEND_WIFLY_XML_S(F("Netmask"), wifly.getNetmask(auxBuffer, sizeof(auxBuffer)), printXMLWiFly);
		SEND_WIFLY_XML_S(F("SSID"), wifly.getSSID(auxBuffer, sizeof(auxBuffer)), printXMLWiFly);
		SEND_WIFLY_XML_S(F("DeviceID"), wifly.getDeviceID(auxBuffer, sizeof(auxBuffer)), printXMLWiFly);
		SEND_WIFLY_XML_D(F("FreeMemory"), wifly.getFreeMemory(), printXMLWiFly);
	}
	 
	SEND_WIFLY_XML_UL(F("UpTimeCald"), (unsigned long)wifly.getUptime(), printXMLWiFly);				//0
	SEND_WIFLY_XML_D(F("OpRecCald"), operationsOK + operationsNOK, printXMLWiFly);						//1
	SEND_WIFLY_XML_D(F("OpOKCald"), operationsOK, printXMLWiFly);                                       //2
	SEND_WIFLY_XML_D(F("OpNOKCald"), operationsNOK, printXMLWiFly);										//3
	SEND_WIFLY_XML_UL(F("RTC"), (unsigned long)wifly.getRTC(), printXMLWiFly);							//4
	wifly.println(F("</Calduino>"));
	
	return true;
}


/**
 * Gets all EMS monitors
 *
 * @return	True if it succeeds, false if it fails.
 */

boolean getAllMonitors()
{

	wifly.println(F("<AllMonitors>"));

	boolean operationStatus = calduino.printEMSDatagram(EMSDatagramID::UBA_Working_Time);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::UBA_Monitor_Fast);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::UBA_Monitor_Slow);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::UBA_Parameter_DHW);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::UBA_Monitor_DHW);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::Working_Mode_DHW);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::Monitor_HC_1);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_1);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_1, DatagramDataIndex::programNameIdx);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::Monitor_HC_2);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_2);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_2, DatagramDataIndex::programNameIdx);
	operationStatus &= calduino.printEMSDatagram(EMSDatagramID::Monitor_MM_10);
	operationStatus &= getCalduinoStats(0);
	wifly.println(F("</AllMonitors>"));

	return operationStatus;
}


/**
 * Searchs an string in the HTTP request received, captures the next parameterLength characters
 * and casts them to decimal.
 *
 * @param [in]	httpRequest	   	Pointer to the HTTP request received.
 * @param [in]	searchedString 	Pointer to the searched string in Flash Memory.
 * @param 	  	parameterLength	Number of digits to be returned.
 *
 * @return	parameter requested, NO_OPERATION if not found. Range is -128 .. 127.
 */

int8_t getParameterFromHTTPRequest(char* httpRequest, prog_char* searchedString, uint8_t parameterLength)
{
	// search the String
	char *auxP = strstr_P(httpRequest, searchedString);
	byte parameter = NO_OPERATION;
	boolean sign = false;

	if (auxP != 0)
	{
		// put the pointer at the start of the parameter to read
		auxP += strlen_P(searchedString);

		// if the first character is a minus sign, increase the pointer one position and multiple the
		// resulting parameter per -1. 
		if (*auxP == '-')
		{
			auxP++;
			sign = true;
		}

		parameter = 0;

		// get the parameterLength digits, converts them to a byte and return the obtained parameter
		for (; parameterLength > 0; parameterLength--)
		{
			parameter = parameter * 10 + *auxP - '0';
			auxP++;
		}
	}
	
	return (sign ? -parameter : parameter);

}

/**
 * Reset WiFly module by pulling down RESET_PIN.
 *
 * @return	Whether WiFly is joined to the standard WiFi connection configured or not.
 */

boolean restartWifly()
{
	// close any possible connection before restarting
	wifly.close();

	digitalWrite(RESET_PIN, LOW);
	delay(MAIN_LOOP_WAIT_TIME);
	digitalWrite(RESET_PIN, HIGH);
	delay(MAIN_LOOP_WAIT_TIME);

	operationsOK = operationsNOK = 0;

	return wifly.isAssociated();
}


/**
Configure WiFly with the existing parameters and restart the module.

@return Whether WiFly is associated to the WiFi connection or not.
*/
boolean configureWifly()
{
	// close any possible connection before configuring.
	wifly.getConnection();
	wifly.close();

	wifly.leave();
	wifly.setJoin(WIFLY_WLAN_JOIN_AUTO);
	wifly.setDHCP(WIFLY_DHCP_MODE_CACHE);
	wifly.setDeviceID(FPSTR(wiFlyDeviceId));
	wifly.setSSID(mySSID);
	wifly.setPassphrase(myPassword);
	wifly.setTimeEnable(0);
	wifly.save();
	wifly.reboot();

	return wifly.isAssociated();

}


/**
 * Connect WiFly module with the domestic WiFi. If the module fails to associate using the
 * configuration saved, restart the module and reconfigure it.
 *
 * @return	Whether WiFly is joined to the standard WiFi connection configured or not.
 */

boolean connectWiFly()
{
	while (!wifly.isAssociated())
	{
		if (!configureWifly())
		{
			DPRINTLN(F("Setup: Configuration Failed. Restarting Module."));

			restartWifly();
		}
	}

	DPRINTLN(F("Setup: WiFly joined."));
}

/**
* Searchs the operation requested via HTTP.
**
* @param [in]	httpRequest		  	The HTTP request received.
*
* @return	whether the operation has been correctly executed or not.
*/

boolean executeOperation()
{

	boolean operationStatus = false;

	// array to store the HTTP Request 
	char httpRequest[HTTP_BUFFER_SIZE];

	// read characters into the buffer until a carriage-return or newline is reached
	wifly.gets(httpRequest, sizeof(httpRequest));

	// search the structure ?op= and get the two following digits
	byte operationRequested = getParameterFromHTTPRequest(httpRequest, op, 2);
	
	// If the operation requested is a Direct GET EMS Command
	if (operationRequested <= EMSDatagramID::Monitor_MM_10)
	{
		operationStatus = calduino.printEMSDatagram((EMSDatagramID)operationRequested);
	}

	// If the operation requested is a SET
	else
	{
		switch (operationRequested)
		{
			case (GET_ALL_MONITORS):
			{
				operationStatus = getAllMonitors();
				break;
			}			
			case (SET_WORK_MODE_HC):
			{
				operationStatus = calduino.setWorkModeHC(getParameterFromHTTPRequest(httpRequest, hc, 1), getParameterFromHTTPRequest(httpRequest, wm, 1));
				break;
			}
			case (SET_TEMPERATURE_HC):
			{
				operationStatus = calduino.setTemperatureHC(getParameterFromHTTPRequest(httpRequest, hc, 1), getParameterFromHTTPRequest(httpRequest, wm, 1), getParameterFromHTTPRequest(httpRequest, tp, 2));
				break;
			}
			case (SET_PROGRAM_HC):
			{
				operationStatus = calduino.setProgramHC(getParameterFromHTTPRequest(httpRequest, hc, 1), getParameterFromHTTPRequest(httpRequest, pr, 2));
				break;
			}
			case (SET_SW_THRESHOLD_TEMP_HC):
			{
				operationStatus = calduino.setSWThresholdTempHC(getParameterFromHTTPRequest(httpRequest, hc, 1), getParameterFromHTTPRequest(httpRequest, tp, 2));
				break;
			}
			case (SET_NIGHT_SETBACK_MODE_HC):
			{
				operationStatus = calduino.setNightSetbackModeHC(getParameterFromHTTPRequest(httpRequest, hc, 1), getParameterFromHTTPRequest(httpRequest, wm, 1));
				break;
			}
			case (SET_NIGHT_THRESHOLD_OUT_TEMP_HC):
			{
				operationStatus = calduino.setNightThresholdOutTempHC(getParameterFromHTTPRequest(httpRequest, hc, 1), getParameterFromHTTPRequest(httpRequest, tp, 2));
				break;
			}
			case (SET_ROOM_TEMP_OFFSET_HC):
			{
				operationStatus = calduino.setRoomTempOffsetHC(getParameterFromHTTPRequest(httpRequest, hc, 1), getParameterFromHTTPRequest(httpRequest, tp, 2));
				break;
			}
			case (SET_PAUSE_MODE_HC):
			{
				operationStatus = calduino.setPauseModeHC(getParameterFromHTTPRequest(httpRequest, hc, 1), getParameterFromHTTPRequest(httpRequest, h, 2));
				break;
			}
			case (SET_PARTY_MODE_HC):
			{
				operationStatus = calduino.setPartyModeHC(getParameterFromHTTPRequest(httpRequest, hc, 1), getParameterFromHTTPRequest(httpRequest, h, 2));
				break;
			}
			case (SET_HOLIDAY_MODE_HC):
			{
				operationStatus = calduino.setHolidayModeHC(getParameterFromHTTPRequest(httpRequest, hc, 1),
					getParameterFromHTTPRequest(httpRequest, sd, 2), getParameterFromHTTPRequest(httpRequest, sm, 2), getParameterFromHTTPRequest(httpRequest, sy, 2),
					getParameterFromHTTPRequest(httpRequest, ed, 2), getParameterFromHTTPRequest(httpRequest, em, 2), getParameterFromHTTPRequest(httpRequest, ey, 2));
				break;
			}
			case (SET_HOME_HOLIDAY_MODE_HC):
			{
				operationStatus = calduino.setHomeHolidayModeHC(getParameterFromHTTPRequest(httpRequest, hc, 1),
					getParameterFromHTTPRequest(httpRequest, sd, 2), getParameterFromHTTPRequest(httpRequest, sm, 2), getParameterFromHTTPRequest(httpRequest, sy, 2),
					getParameterFromHTTPRequest(httpRequest, ed, 2), getParameterFromHTTPRequest(httpRequest, em, 2), getParameterFromHTTPRequest(httpRequest, ey, 2));
				break;
			}
			case (SET_WORK_MODE_DHW):
			{
				operationStatus = calduino.setWorkModeDHW(getParameterFromHTTPRequest(httpRequest, wm, 1));
				break;
			}
			case (SET_WORK_MODE_PUMP_DHW):
			{
				operationStatus = calduino.setWorkModePumpDHW(getParameterFromHTTPRequest(httpRequest, wm, 1));
				break;
			}
			case (SET_TEMPERATURE_DHW):
			{
				operationStatus = calduino.setTemperatureDHW(getParameterFromHTTPRequest(httpRequest, tp, 2));
				break;
			}
			case (SET_TEMPERATURE_TD_DHW):
			{
				operationStatus = calduino.setTemperatureTDDHW(getParameterFromHTTPRequest(httpRequest, tp, 2));
				break;
			}
			case (SET_PROGRAM_DHW):
			{
				operationStatus = calduino.setProgramDHW(getParameterFromHTTPRequest(httpRequest, pr, 3));
				break;
			}
			case (SET_PROGRAM_PUMP_DHW):
			{
				operationStatus = calduino.setProgramPumpDHW(getParameterFromHTTPRequest(httpRequest, pr, 3));
				break;
			}
			case (SET_ONETIME_DHW):
			{
				operationStatus = calduino.setOneTimeDHW(getParameterFromHTTPRequest(httpRequest, wm, 1));
				break;
			}
			case (SET_WORK_MODE_TD_DHW):
			{
				operationStatus = calduino.setWorkModeTDDHW(getParameterFromHTTPRequest(httpRequest, wm, 3));
				break;
			}
			case (SET_DAY_TD_DHW):
			{
				operationStatus = calduino.setDayTDDHW(getParameterFromHTTPRequest(httpRequest, d, 1));
				break;
			}
			case (SET_HOUR_TD_DHW):
			{
				operationStatus = calduino.setHourTDDHW(getParameterFromHTTPRequest(httpRequest, h, 2));
				break;
			}
			case (SET_PROGRAM_SWITCH_POINT):
			{
				operationStatus = calduino.setProgramSwitchPoint((EMSDatagramID)getParameterFromHTTPRequest(httpRequest, pr, 2),
					getParameterFromHTTPRequest(httpRequest, sp, 2),
					getParameterFromHTTPRequest(httpRequest, e, 1),
					getParameterFromHTTPRequest(httpRequest, d, 1),
					getParameterFromHTTPRequest(httpRequest, h, 2),
					getParameterFromHTTPRequest(httpRequest, m, 2));
				break;
			}
			
			case (GET_CALDUINO_BASIC):
			{
				operationStatus = getCalduinoStats(0);
				break;
			}

			case (GET_CALDUINO_FULL):
			{
				operationStatus = getCalduinoStats(1);
				break;
			}

			case (RESTART_WIFLY):
			{
				operationStatus = restartWifly();
				break;
			}
		}
	}

	DPRINTVALUE(F("Returned"), operationStatus);

	if (operationStatus) operationsOK++;
	else operationsNOK++;

	return operationStatus;
}

void setup()
{
	// Reset pin for RN-XV Module. Active when low
	pinMode(RESET_PIN, OUTPUT);
	digitalWrite(RESET_PIN, HIGH);
	
	// Start the serial connection with the EMS BUS in Serial3
	// Start the serial communication with the RN-VX Module in Serial2
	EMSSerial3.begin(EMS_BUS_UART_RATE);
	EMSSerial2.begin(WIFLY_UART_RATE);

#ifdef DEBUG
	// Start serial communication for DEBUG purposes in Serial0
	EMSSerial0.begin(DEBUG_UART_RATE);
#endif

	// Begin Wifly.
	if (wifly.begin(&EMSSerial2))
	{
		DPRINTLN(F("Setup: WiFly correctly started."));
	}
	else
	{
		DPRINTLN(F("Setup: Unable to start WiFly."));
	}

	connectWiFly();

	// Begin calduino. Asign as debug serial the WiFly serial port, so the output generated by
	// calduino will be redirected to the Wifly module and sent to the opened TCP connection 
	if (calduino.begin(&EMSSerial3, &EMSSerial2))
	{
		DPRINTLN(F("Setup: Calduino correctly started."));
	}
	else
	{
		DPRINTLN(F("Setup: Unable to start Calduino."));
	}

	calduino.printFormat = PrintFormat::XML;

}

void loop()
{
	// check if there is an HTTP Request ready to be read
	if (wifly.isConnected() && wifly.available())
	{
		// execute the specified operation (if any)
		executeOperation();
	}

	// close (if any) opened connections
	wifly.close();
	wifly.flush();

	delay(MAIN_LOOP_WAIT_TIME);
}
