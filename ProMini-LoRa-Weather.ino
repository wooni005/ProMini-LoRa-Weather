/*
	LoRa Weather node
	Used libraries:
		- Arduino LoRa:	   https://github.com/sandeepmistry/arduino-LoRa
	 	- Arduino BME280:  https://github.com/malokhvii-eduard/arduino-bme280
		 				   https://github.com/adafruit/Adafruit_BME280_Library
		- Arduino PCF8574: https://github.com/RobTillaart/PCF8574
		- JeeLabs JeeLib:  https://github.com/jeelabs/jeelib
*/

#include <JeeLib.h>			//JeeLib: Used for Sleepy class
#include <SPI.h>			// include libraries
#include <LoRa.h>

// I2C Sensors
#include <Wire.h>
#include <BME280I2C.h>
#include "PCF8574.h"


// #define __DEBUG__	// Uncomment this line for debugging
// #define WIND_DIRECTION

#ifdef __DEBUG__
#define DEBUG(...) Serial.print(__VA_ARGS__)
#define DEBUGLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG(...)
#define DEBUGLN(...)
#endif

#define SERIAL_BAUD	 			57600
#define NODE_ID			 		4			// NodeId of this LoRa Node
#define MAX_PACKET_SIZE			10
#define MSG_ID_NODE_STARTUP		1 	 	// Node startup notification
#define MSG_ID_STILL_ALIVE		2  		// Node still alive
#define MSG_ID_CMND_REQUEST		3  		// Node wakeup/cmnd request
#define MSG_ID_MEASUREMENTS		4  		// Send measure temperature
#define MSG_ID_START_RAINING	5  		// Event msg to report the first rain pulse
#define MSG_ID_REBOOT			6  		// node reboot (after a detected fail)

//#define SEND_MSG_EVERY	22		// Watchdog is a timerTick on a avg 8,0 sec timebase
										// SEND_MSG_EVERY=8	-> +- 1min
										// SEND_MSG_EVERY=14 -> +- 2min
										// SEND_MSG_EVERY=23 -> +- 3min
										// SEND_MSG_EVERY=30 -> +- 4min
										// SEND_MSG_EVERY=38 -> +- 5min
										// SEND_MSG_EVERY=228 -> 0,5 hours
										// SEND_MSG_EVERY=1824 -> 4 hours

#ifdef __DEBUG__
#define SEND_DATA				2    	// For testing 16 sec
#else
// #define SEND_DATA				2    	// For testing 16 sec
#define SEND_DATA				37    	// Avg. measured 4:58.541
// #define SEND_DATA				38    	// Avg. measured 5:06.621
#endif
#define SEND_MEASURE_VCC_EVERY	48		// Measure Vcc voltage every N messages
										// MEASURE_EVERY=48 -> +- 4 hour
#define MAX_NR_FAILS			3		// Reset node after 3 fails=15min
#define DRY_PERIOD_BEFORE_RAIN	4		// How long should it not rain before sending message it's raining (again) 4 timer ticks=15-20min
#define WINDSPEED_INPUT 		3		// INT1
#define WINDDIRECTION_POWER		A3		// Switch off the PCF8574 power to reduce power usage in sleep
#define RAINPULSE_INPUT			4		// PCINT2

// Failure bits
#define FAIL_BME280_NOT_FOUND 	0x01  	//bit 0
#define FAIL_PCF8574_NOT_FOUND 	0x02	//bit 1

/* Recommended Modes -
   Based on Bosch BME280I2C environmental sensor data sheet.

Weather Monitoring :
   forced mode, 1 sample/minute
   pressure ×1, temperature ×1, humidity ×1, filter off
   Current Consumption =  0.16 μA
   RMS Noise = 3.3 Pa/30 cm, 0.07 %RH
   Data Output Rate 1/60 Hz
*/

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
);

BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);
float temp(NAN), hum(NAN), pres(NAN);

//I2C
BME280I2C bme(settings);
PCF8574 pcf20(0x20);

volatile bool bme280Found = false;
volatile bool pcf8574Found = false;
volatile bool timerTick = false;
volatile bool rainPulse = false;
volatile bool sendMsg = false;

volatile float fData;

#ifdef __DEBUG__
//Debug values
volatile unsigned int	wakedUpCount = 0; //Unsigned int=16bit=max 65.535
#endif

//Wind
volatile unsigned int	nrOfWindPulses = 0; //Needs to be unsigned int, otherwise an unsigned char max windspeed is 39km/h in 8 sec timeframe
//Wind gust detection
volatile unsigned int	oldNrOfWindPulses = 0;
volatile unsigned int 	maxWindGust = 0;
volatile unsigned int 	diffNrOfWindPulses = 0; 

//Rain
volatile unsigned char	nrOfRainPulses = 0; //Unsigned int=16bit=max 65.535

//Timers
volatile unsigned int	sendMsgTimer = SEND_DATA - 2;
volatile unsigned char 	sendMsgVccLevelTimer = SEND_MEASURE_VCC_EVERY;
volatile unsigned char  resetNodeFailCounter = 0;
volatile unsigned char  dryPeriondCount = 0;

//Message max 30 bytes
struct WeatherPayload {
	byte nodeId;
	byte msgId;
	byte voltageVcc;				//getVcc 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
	byte failure;					//bit 0=BME280 sensor not found
									//bit 1=PCF8574 sensor not found
	int temperature;
	int humidity;
	int pressure;
	byte windDirection;
	unsigned int windPulses;		//Total nr of wind pulses in 5 min: 39294 pulses by 160km/h
	unsigned int windGust; 			//Max nr of measured pulses in an 8 sec frame
	unsigned int rainPulses; 		//Total nr of rain pulses in 5 min
} weatherPayload;

const long loRaFrequency = 866E6;			// LoRa loRaFrequency 866MHz

const int loRaCsPin = 15;						// LoRa radio chip select
const int loRaResetPin = 14;			 		// LoRa radio reset
const int loRaIrqPin = 2;						// change for your board; must be a hardware interrupt pin

void LoRa_rxMode(){
	LoRa.enableInvertIQ();								// active invert I and Q signals
	LoRa.receive();										// set receive mode
}

void LoRa_txMode(){
	LoRa.idle();											// set standby mode
	LoRa.disableInvertIQ();								// normal mode
}

void LoRa_sendMessage(WeatherPayload payload, byte payloadLen) {
	LoRa_txMode();											// set tx mode
	LoRa.beginPacket();								 	// start packet
	LoRa.write((byte*) &payload, payloadLen); 	// add payload
	LoRa.endPacket(true);								// finish packet and send it
}

void onReceive(int packetSize) {
	byte rxPayload [MAX_PACKET_SIZE];

	byte i = 0, rxByte;

	while (LoRa.available()) {
		rxByte = (byte)LoRa.read();
		if (i < MAX_PACKET_SIZE) {
			rxPayload[i] = rxByte;
			i++;
		}
	}

	// Only accept messages with our NodeId
	if (rxPayload[0] == NODE_ID) {
#ifdef __DEBUG__
		DEBUG("Rx packet OK "); // Start received message
		for (char i = 0; i < packetSize; i++) {
				DEBUG(rxPayload[i], DEC);
				DEBUG(' ');
		}
#endif
	}
}

void onTxDone() {
	// DEBUGLN("TxDone");
	LoRa_rxMode();
}

ISR (WDT_vect)
{
	// WDIE & WDIF is cleared in hardware upon entering this ISR
	// wdt_disable();
	Sleepy::watchdogEvent();
	if(!timerTick) { timerTick = true; }
}

//External interrupt 1: windspeed sensor
void windPulseCount () { nrOfWindPulses++; }

//Pin change interrupt routine for PortD0..7, only PD4 is used
ISR(PCINT2_vect) { 
	// PCINT2 triggers on falling and trailing edge.
	// Count only the falling edges
	if (!digitalRead(RAINPULSE_INPUT)) {
		nrOfRainPulses++;
		if(!rainPulse) { rainPulse = true; }
	}
}

// This function will reboot the node
void(* resetFunc) (void) = 0;

static byte vccLevelRead()
{
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Let mux settle a little to get a more stable A/D conversion
  
  // Start a conversion
  ADCSRA |= _BV(ADSC);
  
  // Wait for it to complete
  while (bit_is_set(ADCSRA, ADSC));
  
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  // 1.0V = 0, 1.8V = 40, 3.0V = 100, 3.3V = 115, 5.0V = 200, 6.0V = 250
  return (55U * 1023U) / (ADC + 1) - 50;
}

void setup() {
	bool status;

#ifdef __DEBUG__
	Serial.begin(SERIAL_BAUD);									 // initialize serial
	while(!Serial) {} // Wait
#endif

	DEBUGLN();
	DEBUG("[LORA-NODE.");
	DEBUG(NODE_ID);
	DEBUGLN("]");

	// locate devices on the bus
	DEBUGLN("Locating devices...");

#ifdef WIND_DIRECTION
	pinMode(WINDDIRECTION_POWER, OUTPUT);
	digitalWrite(WINDDIRECTION_POWER, HIGH);
#endif
	Wire.begin();
	DEBUGLN("bme begin");
	if (!bme.begin()) { 
		bme280Found = false;
		DEBUGLN("Could not find a valid BME280 sensor, check wiring!");
	} else {
		bme280Found = true;
		DEBUGLN("BME280 sensor found!");
	}

	if (bme280Found) {
		switch(bme.chipModel())
		{
			case BME280::ChipModel_BME280:
				DEBUGLN("Found BME280 sensor! Success.");
				break;
			case BME280::ChipModel_BMP280:
				DEBUGLN("Found BMP280 sensor! No Humidity available.");
				break;
			default:
				DEBUGLN("Found UNKNOWN sensor! Error!");
		}

		// Change some settings before using.
		settings.tempOSR = BME280::OSR_X4;
		bme.setSettings(settings);
	}

#ifdef WIND_DIRECTION
	status = pcf20.begin();
	if (!status) {
		pcf8574Found = false;
		DEBUGLN("Could not find a valid PCF8574 I/O expander, check wiring!");
	} else {
		pcf8574Found = true;
		DEBUGLN("PCF8574 I/O expander found!");
	}
	digitalWrite(WINDDIRECTION_POWER, LOW);
#endif

	//Windspeed sensor input
	pinMode(WINDSPEED_INPUT, INPUT); 	//Without internal pullup R3=100k C2=100n (RC=10ms)
	attachInterrupt(1, windPulseCount, FALLING);
	bitSet(EICRA, ISC11); //Enable the external interrupt 1

	// Rainpulse sensor input
	pinMode(RAINPULSE_INPUT, INPUT);	//Without internal pullup R4=180k C3=2n2 (RC=0,396ms)
	bitSet(PCICR, PCIE2);				//Enable pin change interrupt for Port D (D0..D7)
	bitSet(PCMSK2, RAINPULSE_INPUT);	 //Choose which pins has to interrupt: D4

	LoRa.setPins(loRaCsPin, loRaResetPin, loRaIrqPin);

	if (!LoRa.begin(loRaFrequency)) {
		DEBUGLN("LoRa init failed. Check your connections.");
		while (true);											 // if failed, do nothing
	}

	//LoRa.setTxPower(20);
	LoRa.enableCrc();
	LoRa.onReceive(onReceive);
	LoRa.onTxDone(onTxDone);
	LoRa_rxMode();
	delay(40);

	// Send Node startup msg
	weatherPayload.nodeId = NODE_ID;
	weatherPayload.msgId = MSG_ID_NODE_STARTUP;
	LoRa_sendMessage(weatherPayload, 2); // send a message
	delay(80); // [ms] Give RFM95W time to send the message
	LoRa.sleep(); 	// Put RFM95W in sleep mode

#ifdef __DEBUG__
	delay(200); // [ms] Give time to print the debug messages before sleep
#endif //DEBUG
	Sleepy::watchdogInterrupts(1); //Start the watchdog timer for time base
}

void loop() {
#ifdef __DEBUG__
	wakedUpCount++;
#endif

	if (timerTick) 
	{ 	
		// There has ben a Watchdog interrupt for time measurement
		timerTick = false;
		// Waked up by periodic wakeup timer (8s)
		sendMsgTimer++;
		DEBUGLN();
		DEBUGLN();
		DEBUGLN("Timer tick");
		//Wind
		diffNrOfWindPulses = nrOfWindPulses - oldNrOfWindPulses;
		oldNrOfWindPulses = nrOfWindPulses; //Store and reset the pulse counter in short time, a new interrupt can come

		if (diffNrOfWindPulses > maxWindGust) {
			//Wind gust detected
			maxWindGust = diffNrOfWindPulses;
			DEBUG("Wind gust detected: ");
			DEBUGLN(maxWindGust);
		}


		if (sendMsgTimer >= SEND_DATA) {
			sendMsgTimer = 0;
			sendMsg = true;

			DEBUGLN();
			DEBUGLN("Send data message");
			weatherPayload.nodeId = NODE_ID;
			weatherPayload.msgId = MSG_ID_MEASUREMENTS;

			sendMsgVccLevelTimer++;
			if (sendMsgVccLevelTimer >= SEND_MEASURE_VCC_EVERY) {
				sendMsgVccLevelTimer = 0;
				weatherPayload.voltageVcc = vccLevelRead();
			}

			//
			//Read out sensors
			//
			weatherPayload.failure = 0; // Reset detected failures
#ifdef WIND_DIRECTION
			digitalWrite(WINDDIRECTION_POWER, HIGH);
#endif
			if (bme280Found) {
				bme.read(pres, temp, hum, tempUnit, presUnit);
				DEBUG("BME280 Sensor : ");
				DEBUG(temp);
				weatherPayload.temperature = 100 * temp + 0.5;
				DEBUG(" °C - ");
				DEBUGLN(weatherPayload.temperature);

				DEBUG(hum);
				weatherPayload.humidity = 100 * hum + 0.5;
				DEBUG(" % - ");
				DEBUGLN(weatherPayload.humidity);

				DEBUG(pres);
				weatherPayload.pressure = (pres / 10) + 5;
				DEBUG(" mbar - ");
				DEBUGLN(weatherPayload.pressure);
			} else {
				weatherPayload.failure |= FAIL_BME280_NOT_FOUND;
				weatherPayload.temperature = 0;
				weatherPayload.humidity = 0;
			}

#ifdef WIND_DIRECTION
			if (pcf8574Found) {
				pcf20.begin();
				DEBUG("PCF8574 Inputs:");
				weatherPayload.windDirection = pcf20.read8() ^ 0xFF;
				DEBUGLN(weatherPayload.windDirection, BIN);
			} else {
				weatherPayload.failure |= FAIL_PCF8574_NOT_FOUND;
				weatherPayload.windDirection = 0;
			}
			Wire.end();
			digitalWrite(WINDDIRECTION_POWER, LOW);
#endif

			//Wind
			DEBUG("nrOfWindPulses: ");
			DEBUGLN(nrOfWindPulses);
			weatherPayload.windPulses = nrOfWindPulses;
			weatherPayload.windGust = maxWindGust;
			nrOfWindPulses = 0;
			oldNrOfWindPulses = nrOfWindPulses;
			maxWindGust = 0;

			//Rain
			DEBUG("nrOfRainPulses: ");
			DEBUGLN(nrOfRainPulses);
			if (dryPeriondCount != 0) {
				if (nrOfRainPulses == 0) {
					// Not raining last 5 min, lower counter
					dryPeriondCount--;
				} else {
					// Raining again, set dry period counter
					dryPeriondCount = DRY_PERIOD_BEFORE_RAIN;
				}
#ifdef __DEBUG__
				DEBUG("dryPeriondCount: ");
				DEBUGLN(dryPeriondCount);
#endif
			}
			weatherPayload.rainPulses = nrOfRainPulses;
			nrOfRainPulses = 0;

#ifdef __DEBUG__
			DEBUG("wakedUpCount: ");
			DEBUGLN(wakedUpCount);
			wakedUpCount = 0;
#endif
			//Turn-off sensors
			//Not needed for I2C sensors, I2C goes default in stand-by/low-power

			//Max packet size is currently 25 (define in STM32_LoRaGateway.ino)
			LoRa_sendMessage(weatherPayload, sizeof(weatherPayload)); // send a message
			delay(40); 		// [ms] Give RFM95W time to send the message

#ifdef __DEBUG__
			delay(200); // [ms] Give time to print the debug messages before sleep
#endif //DEBUG

			if (weatherPayload.failure != 0) {
				resetNodeFailCounter++;
				if (resetNodeFailCounter >= MAX_NR_FAILS) {
					delay(40); // Some extra time for the previous LoRa msg
					weatherPayload.nodeId = NODE_ID;
					weatherPayload.msgId = MSG_ID_REBOOT;
					LoRa_sendMessage(weatherPayload, 2); // send a message
					delay(80); // [ms] Give RFM95W time to send the message
					LoRa.sleep(); 	// Put RFM95W in sleep mode
					
					// // Sleepy::watchdogInterrupts(-1); // Disable sleep timer
					// delay(200);
#ifdef __DEBUG__
					DEBUGLN("Rebooting now");
					delay(200); // [ms] Give serial port time to print the message
#endif
					resetFunc();
				}
			} else {
				// No failure detected
				resetNodeFailCounter = 0; // Reset the fail counter
			}
			LoRa.sleep(); 	// Put RFM95W in sleep mode
		}
	}

	if (rainPulse) {
#ifdef __DEBUG__
		DEBUGLN("Rain pulse");
		delay(200); // [ms] Give RFM95W time to send the message
#endif
		if (dryPeriondCount == 0) {
#ifdef __DEBUG__
			DEBUGLN("Long dry period");
			delay(200); // [ms] Give RFM95W time to send the message
#endif
			// If it was a certain period not raining, send a start raining message
			weatherPayload.nodeId = NODE_ID;
			weatherPayload.msgId = MSG_ID_START_RAINING;
			dryPeriondCount = DRY_PERIOD_BEFORE_RAIN;
			LoRa_sendMessage(weatherPayload, 2); // send a message
			delay(40); // [ms] Give RFM95W time to send the message
			LoRa.sleep(); 	// Put RFM95W in sleep mode
		}
	}
	Sleepy::watchdogInterrupts(9);		// Start the watchdog timer for timerTick 6=5,2sec 9=8,31sec
	Sleepy::powerDown();
}
