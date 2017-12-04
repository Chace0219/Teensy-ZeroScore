
/* */

// Libraries
#include <Nextion.h>
#include <i2c_t3.h>
#include <math.h>

// Cusotm libraries,
#include "QueueArray.h"
#include "ZscoreAlgo.h"

//Nextion page and widget locations
NexPage Main = NexPage(3, 0, "page2");
NexPage Select = NexPage(2, 0, "page1");
NexWaveform Paw = NexWaveform(3, 2, "s0");
NexNumber Ppeak = NexNumber(3, 3, "page2.n0");
NexNumber MaxTV = NexNumber(3, 4, "page2.n2");
NexNumber RRdisplay = NexNumber(3, 6, "page2.n3");
NexText MV = NexText(3, 13, "page2.t0"); // id should be changed

// FIFO Queue for measured sensor values
// It is using for avoid to lose meseaured data, 
// while we calculate algorithm
QueueArray<double> PAWQueue;
QueueArray<double> MFQueue;

// sampling time for every meaure cycle
#define SAMPLECYCLE	8 // millisec unit 

// event flag between interrupt routine and main loop to notice to calcu algorithm
volatile bool bAlgorithm = false;

// 
const uint16_t REALCOUNT = 64; // meaured sampling count in one moving window
// lag  for moving window 
const uint16_t LAGCOUNT = 16; // average count

// buffer for measured data to calculate 
double readingsMF[REALCOUNT];
double readingsPAW[REALCOUNT];

/* 
	for smooth and more accurate Zscore algorithm, we are using DB for signals, 
	for both algorithm instance two sensor variables
*/
// for massflow 
double signals_MF[REALCOUNT];
double filteredY_MF[REALCOUNT];
double avgFilter_MF[REALCOUNT];
double stdFilter_MF[REALCOUNT];

// for PAW
double signals_PAW[REALCOUNT];
double filteredY_PAW[REALCOUNT];
double avgFilter_PAW[REALCOUNT];
double stdFilter_PAW[REALCOUNT];

// Global variables to display on Display
double cmH2OGraph;
double FlowGraph;
uint32_t MaxTVval;
double ratePAW = 0.0;

unsigned long secondPulseStartTime = 0;
unsigned long time_between_pulses = 0;

double RR = 0;
double maxValue = 0.00;

int PawreadIndex = 0;              // the index of the current reading
int Pawtotal = 0;                  // the running total
int Pawaverage = 0;                // the average

int MFreadIndex = 0;              // the index of the current reading
int MFtotal = 0;                  // the running total
int MFaverage = 0;                // the average

/* */
IntervalTimer measuringTimer;

// Interrupt service routine, 
void measureRoutine()
{
	double Pawmeasure;
	double Massflow;

	//Function Reads Both Pressure Sensors into bridge_data#     
	int8_t error, error1 = 0;
	// Check sensor data
	Wire.beginTransmission(0x28);
	error = Wire.endTransmission();
	Wire1.beginTransmission(0x28);
	error1 = Wire1.endTransmission();
	
	// 
	if (error == 0 && error1 == 0)
	{
		Wire.beginTransmission(0x28);
		Wire1.beginTransmission(0x28);
		Wire.write(1);
		Wire1.write(1);
		Wire.endTransmission();
		Wire1.endTransmission();
		Wire.requestFrom(0x28, 2);
		Wire1.requestFrom(0x28, 2);
		while (Wire.available() == 0 && Wire1.available() == 0);

		// 
		byte a = Wire.read();
		byte b = Wire.read();
		byte c = Wire1.read();
		byte d = Wire1.read();

		// Linear Scaling
		int bridge_data = ((a & 0x3f) << 8) + b;
		int bridge_data1 = ((c & 0x3f) << 8) + d;

		// Compute pressures in cmH20
		float pressure = (bridge_data - 1638.4) / (13107.2);
		float pressure1 = (bridge_data1 - 1638.4) / (13107.2);
		
		// 
		float cmH2O = (pressure * 70.3069578296);
		float cmH2O1 = (pressure1 * 70.3069578296);

		// PAW
		if (cmH2O> cmH2O1)
			Pawmeasure = cmH2O;
		else
			Pawmeasure = cmH2O1;

		// Mass
		Massflow = ((0.76 * (68.9475728 * (pressure - pressure1))) - 0.85) * 60;

		// it is implemented moving window for algorithm
		PAWQueue.push(Pawmeasure);
		MFQueue.push(Massflow);
		
		// REALCOUNT, 
		if (PAWQueue.count() >= REALCOUNT)
		{
			for (uint16_t idx = 0; idx < REALCOUNT; idx++)
			{
				readingsMF[idx] = MFQueue.pop();
				MFQueue.push(readingsMF[idx]);

				readingsPAW[idx] = PAWQueue.pop();
				PAWQueue.push(readingsPAW[idx]);
			}

			for (uint16_t idx = 0; idx < (REALCOUNT - LAGCOUNT); idx++)
			{
				MFQueue.pop();
				PAWQueue.pop();
			}

			bAlgorithm = true;
		}

	}
	else // 
		Serial.println("I2C Devices Not found!");
}

/* */
void setup()
{
	// 
	Wire.begin();
	Wire1.begin();

	// 
	Serial2.begin(115200);
	nexInit();

	/* Usb serial */
	Serial.begin(115200);
	// If you leave this code, Teensy will wait for USB connection, 
	// if you work withiout usb, you have to comment it
	while (!Serial || millis() < 10000);

	// start line
	Serial.println("Program started!");

	delay(1000);
	Select.show();

	// interrupt timer init
	measuringTimer.begin(measureRoutine, SAMPLECYCLE * 1000);
}

uint32_t pulseWidth = 0;
bool bCountingPAW = false;

// 
double flowInspiration = 0.0;
double flowBaseline = 0.0;
uint32_t flowCount = 0;

// 
static uint32_t nTimeMVProc = millis();
const uint32_t nMinuteTime = 60000; // ms unit
static uint32_t ventilPerMin = 0;

void loop()
{
	if ((millis() - nTimeMVProc) >= nMinuteTime)
	{
		nTimeMVProc = millis();
		// Display Part
		// if you sue string, String(ventilPerMin, 0);
		Serial.print("Total ventilPerMin is ");
		Serial.println((double)ventilPerMin / (double)1000, 1);

		MV.setText(String((double)ventilPerMin / (double)1000, 1).c_str());
		ventilPerMin = 0;
	}

	/* In current situation, Program is using interrupt every SAMPLECYCLE ms */
	if (bAlgorithm)
	{
		// reset event flag 
		bAlgorithm = false;

		// This is for PAW, calculate breath ore minute 
		// We have tested and found these parameters
		double dThreshold = 10.0;
		double dInfluence = 0.2;
		// run algorithm 
		realtimeZscoreAlgorithm(readingsPAW, REALCOUNT, LAGCOUNT, dThreshold, dInfluence,
			signals_PAW, avgFilter_PAW, filteredY_PAW, stdFilter_PAW);
		// Get time between peak, and detect zero when no peak for 30 secs
		for (uint16_t idx = LAGCOUNT; idx < REALCOUNT; idx++)
		{
			if (signals_PAW[idx] == 1.0 && signals_PAW[idx - 1] == 0.0)
			{
				// display every peak
				Ppeak.setValue(avgFilter_PAW[idx]);

				if (bCountingPAW)
				{
					ratePAW = (double)60000 / (double)(SAMPLECYCLE * pulseWidth);
					if (ratePAW < 100)
						// display breath per minute on display
						RRdisplay.setValue(round(ratePAW));
					pulseWidth = 0;
				}
				bCountingPAW = true;
			}

			if (bCountingPAW)
			{
				pulseWidth++;
				if ((pulseWidth * SAMPLECYCLE) >= 30000)
				{ // None peak, so no signal
					
					// reset variables
					pulseWidth = 0;
					bCountingPAW = false;
					ratePAW = 0.0;
					ventilPerMin = 0;

					// peak widget
					Ppeak.setValue(0);
					RRdisplay.setValue(0);
					
					// flow wdiget
					MaxTV.setValue(0);
					MV.setText("0.0");

					// debug code
					Serial.println();
					Serial.print("Current breath per minute is ");
					Serial.println(ratePAW, 2);
				}
			}
		}

		// MF Zero score calculation part
		// We have tested and found these parameters
		dThreshold = 1.0; 
		dInfluence = 0.05;
		realtimeZscoreAlgorithm(readingsMF, REALCOUNT, LAGCOUNT, dThreshold, dInfluence,
			signals_MF, avgFilter_MF, filteredY_MF, stdFilter_MF);

		// Graph display in nextion, As breath per minute, it will adjsut graph interval
		for (uint16_t idx = LAGCOUNT; idx < REALCOUNT; idx += (ratePAW / 3 + 1))
		{
			cmH2OGraph = mapdouble(avgFilter_PAW[idx], 0, 71, 120, 240);
			FlowGraph = mapdouble(avgFilter_MF[idx], -120, 120, 0, 120) + 25;
			Paw.addValue(0, (uint8_t)cmH2OGraph);
			Paw.addValue(1, (uint8_t)FlowGraph);
		}

		for (uint16_t idx = LAGCOUNT; idx < REALCOUNT; idx++)
		{
			// Get Baseline
			if (signals_MF[idx] == 1.0 && signals_MF[idx - 1] != 1.0)
			{// transition from 0.0 to 1.0, init flow value.
				flowCount = 0;
				flowInspiration = 0.0;
				flowBaseline = avgFilter_MF[idx];

			}
			else if (signals_MF[idx] == 0.0 && signals_MF[idx - 1] != 0.0)
			{// transition from 1.0 to 0.0, integration is done
				flowCount++;
				flowInspiration += ((avgFilter_MF[idx] - flowBaseline) * (double)SAMPLECYCLE);
				double dRealFlow = (flowInspiration / 60.0) * 1.8;
				if (flowCount > 1 && dRealFlow > 2.0)
				{
					// Nextion Display
					Serial.print("Time Length is ");
					Serial.print(flowCount);
					Serial.print("， Measureed Flow is ");
					Serial.println(dRealFlow);

					MaxTV.setValue((uint32_t)dRealFlow);
					ventilPerMin += (uint32_t)dRealFlow;

					Serial.print("Current ventilPerMin is ");
					Serial.println(ventilPerMin);

				}
				flowInspiration = 0.0;
				flowCount = 0;
			}
			// when Score 1.0, integration
			if (signals_MF[idx] == 1.0)
			{ // We have to confirm about thid formula, If we have found peak(signal = 1.0, we are adding average * segment time(CYCLE)
				flowCount++;
				flowInspiration += ((avgFilter_MF[idx] - flowBaseline) * (double)SAMPLECYCLE);
			}
		}

		// in result array, lag count of data will be left for next calulation
		// (avg, filtered, std)
		for (uint16_t idx = 0; idx < LAGCOUNT; idx++)
		{
			// Buffer shift for next calculation
			signals_MF[idx] = signals_MF[idx + (REALCOUNT - LAGCOUNT)];
			avgFilter_MF[idx] = avgFilter_MF[idx + (REALCOUNT - LAGCOUNT)];
			filteredY_MF[idx] = filteredY_MF[idx + (REALCOUNT - LAGCOUNT)];
			stdFilter_MF[idx] = stdFilter_MF[idx + (REALCOUNT - LAGCOUNT)];

			signals_PAW[idx] = signals_PAW[idx + (REALCOUNT - LAGCOUNT)];
			avgFilter_PAW[idx] = avgFilter_PAW[idx + (REALCOUNT - LAGCOUNT)];
			filteredY_PAW[idx] = filteredY_PAW[idx + (REALCOUNT - LAGCOUNT)];
			stdFilter_PAW[idx] = stdFilter_PAW[idx + (REALCOUNT - LAGCOUNT)];
		}
	}
}
