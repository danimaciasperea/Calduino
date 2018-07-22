/**
*	Name:		CalduinoWiFly.ino
*	Created:	4/23/2018 10:16:59 AM
*	Author:	Administrator
*
*	UART ports used
*	UART_0 (Serial) -> Debug
*	UART_3 (Serial3) -> Calduino
*
*/

#include <Calduino.h>

/* Debugging functions */
#define DEBUG

#ifdef DEBUG
#define DPRINT(item) EMSSerial0.print(item)
#define DPRINTLN(item) EMSSerial0.println(item)
#define DPRINTVALUE(item1, item2) EMSSerial0.print(item1);EMSSerial0.print(": ");EMSSerial0.println(item2)
#else
#define DPRINT(item)
#define DPRINTLN(item)
#define DPRINTVALUE(item1)
#endif

#define DEBUG_UART_RATE 9600 // Debug UART rate
#define EMS_BUS_UART_RATE 9700 // EMS Bus - UART Interface rate
#define WAIT_KEY while (!EMSSerial0.available()) delay(100); EMSSerial0.flush();
Calduino calduino;

void setup()
{
	// Start the serial connection with the EMS BUS in Serial3
	EMSSerial3.begin(EMS_BUS_UART_RATE);

#ifdef DEBUG

	// Start serial communication for DEBUG purposes in Serial0. 
	EMSSerial0.begin(DEBUG_UART_RATE);

	// Begin calduino with the two serial UARTs. 
	if (calduino.begin(&EMSSerial3, &EMSSerial0))
	{

		EMSSerial0.println(F("Calduino correctly started."));
	}
	else
	{
		EMSSerial0.println(F("Error starting Calduino."));
	}

	
#else

	// Begin calduino without Debug Serial.
	calduino.begin(&calduinoSerial3);

#endif

}

void loop()
{
	calduino.printFormat = PrintFormat::XML;

	calduino.printEMSDatagram(EMSDatagramID::RC_Datetime);
	WAIT_KEY;
	
	DPRINTVALUE(F("Second"),calduino.getCalduinoByteValue(ByteRequest::second_b));
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::RC_Datetime, DatagramDataIndex::secondIdx);
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::UBA_Working_Time);
	WAIT_KEY;

	DPRINTVALUE(F("uBAWorkingMin"), calduino.getCalduinoUlongValue(ULongRequest::uBAWorkingMin_ul));
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::UBA_Monitor_Fast);
	WAIT_KEY;
	 
	DPRINTVALUE(F("curImptTemp"), calduino.getCalduinoFloatValue(FloatRequest::curImpTemp_f));
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::UBA_Monitor_Slow);
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::UBA_Parameter_DHW);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::UBA_Parameter_DHW, DatagramDataIndex::selTempDHWIdx);
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::UBA_Monitor_DHW);
	WAIT_KEY;

	DPRINTVALUE(F("DayMode"), calduino.getCalduinoBitValue(BitRequest::dayModeDHW_t));
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::Flags_DHW);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_DHW);
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::Program_DHW);
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::Program_Pump_DHW);
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_1);
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::Monitor_HC_1);
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_1);
	WAIT_KEY;

	DPRINTVALUE(F("Switch Point 1 HC 1 Prog 1 Hour"), (calduino.getCalduinoSwitchPoint(EMSDatagramID::Program_1_HC_1, 0)).hour);
	WAIT_KEY;

	calduino.printEMSDatagram(EMSDatagramID::Program_2_HC_1);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_2);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Monitor_HC_2);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_2);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Program_2_HC_2);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_3);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Monitor_HC_3);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_3);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Program_2_HC_3);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_4);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Monitor_HC_4);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_4);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Program_2_HC_4);
	WAIT_KEY;
	
	calduino.printEMSDatagram(EMSDatagramID::Monitor_MM_10);
	WAIT_KEY;
	
	calduino.setWorkModeHC(2, 2);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_2);
	WAIT_KEY;

	calduino.setTemperatureHC(2, 0, 18);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_2);
	WAIT_KEY;

	calduino.setProgramHC(2, 0);
	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_2);
	WAIT_KEY;

	calduino.setSWThresholdTempHC(1, 10);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_1);
	WAIT_KEY;

	calduino.setNightSetbackModeHC(1, 0);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_1);
	WAIT_KEY;

	calduino.setNightThresholdOutTempHC(1, -5);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_1);
	WAIT_KEY;

	calduino.setRoomTempOffsetHC(1, -10);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_HC_1);
	WAIT_KEY;

	calduino.setPauseModeHC(1, 0);
	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_1);
	WAIT_KEY;

	calduino.setPartyModeHC(1, 0);
	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_1);
	WAIT_KEY;

	calduino.setHolidayModeHC(1, 1, 1, 0, 1, 1, 0);
	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_1);
	WAIT_KEY;

	calduino.setHomeHolidayModeHC(1, 1, 1, 0, 1, 1, 0);
	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_1);
	WAIT_KEY;

	calduino.setWorkModeDHW(2);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_DHW);
	WAIT_KEY;

	calduino.setWorkModePumpDHW(0);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_DHW);
	WAIT_KEY;

	calduino.setTemperatureDHW(40);
	calduino.printEMSDatagram(EMSDatagramID::UBA_Parameter_DHW);
	WAIT_KEY;

	calduino.setTemperatureTDDHW(80);
	calduino.printEMSDatagram(EMSDatagramID::UBA_Parameter_DHW);
	WAIT_KEY;

	calduino.setProgramDHW(255);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_DHW);
	WAIT_KEY;

	calduino.setProgramPumpDHW(0);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_DHW);
	WAIT_KEY;

	calduino.setWorkModeTDDHW(false);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_DHW);
	WAIT_KEY;

	calduino.setProgramTDDHW(0,0);
	calduino.printEMSDatagram(EMSDatagramID::Working_Mode_DHW);
	WAIT_KEY;

	calduino.setProgramSwitchPoint(EMSDatagramID::Program_1_HC_1, 0, 1, 0, 16, 0);
	calduino.printEMSDatagram(EMSDatagramID::Program_1_HC_1);
	WAIT_KEY;

	calduino.setOneTimeDHW(false);
	calduino.printEMSDatagram(EMSDatagramID::Flags_DHW);
	calduino.printEMSDatagram(EMSDatagramID::UBA_Monitor_DHW);
	WAIT_KEY;

}

