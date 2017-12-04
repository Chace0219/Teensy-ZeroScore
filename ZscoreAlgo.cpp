
#include <Arduino.h>
#include "ZscoreAlgo.h"
#include "Statistic.h"

/* customized map fucntion for double variables */
double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
	if (in_max - in_min == 0.0F)
		return 0;
	if (x < in_min)
		return out_min;
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* 
	Description: 
		Implementation of smooth Z-score algorithm 
	Arguments and parameters:
		double y[] : input array for calculate algorithm
		uint32_t nCount: Array count set for one cycle algorithm
		uint32_t lag: lag count of moving array
		double threshold: 0 ~ 1
		double influence: 0 ~ 1

		double signals_[]: Zscore results buffer
		
		// buffers for calculation, 
		// in order to implement smooth, we are lefting last results for next cycle
		double avgFilter_[]:
		double filteredY_[]:
		double stdFilter_[]:
*/
void realtimeZscoreAlgorithm(double y[], uint32_t nCount, uint32_t lag, double threshold, double influence,
	double signals_[], double avgFilter_[], double filteredY_[], double stdFilter_[])
{
	// excpet previous result, rest buffers will be reset,
	// we can decrease them
	for (uint32_t idx = lag; idx < nCount; idx++)
	{
		avgFilter_[idx] = 0.0F;
		stdFilter_[idx] = 0.0F;
		filteredY_[idx] = y[idx];
	}

	Statistic myStats;
	for (uint32_t idx = lag; idx < nCount; idx++)
	{
		/*
			The principal of dispersion is that if a datapoint 
			is a given number of standard deviations away from some moving mean, 
			
			the algorithm will signal a 1 or -1 (called a z-score) 
			to identify peaks and troughs in a waveform.
		*/
		if (abs(y[idx] - avgFilter_[idx - 1]) > threshold * stdFilter_[idx - 1])
		{
			if (y[idx] > avgFilter_[idx - 1])
				signals_[idx] = 1;
			else
				signals_[idx] = -1;
			// - Threshold  is the z-score at which the algorithm signals 
			// - Influence(between 0 and 1) of new signals on the mean and standard deviation.
			filteredY_[idx] = influence * y[idx] + (1 - influence) * filteredY_[idx - 1];
		}
		else
		{
			signals_[idx] = 0;
			filteredY_[idx] = y[idx];
		}

		myStats.clear();
		// get mean from moving window
		for (uint32_t idx1 = idx - lag; idx1 < (idx + 1); idx1++)
			myStats.add(filteredY_[idx1]);
		// get average from mean
		avgFilter_[idx] = myStats.average();
		// get standard deviation from mean
		stdFilter_[idx] = myStats.pop_stdev();
	}
}