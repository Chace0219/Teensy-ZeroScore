#ifndef ZSCOREALGO_h
#define ZSCOREALGO_h

/* customized map fucntion for double variables */
double mapdouble(double x, double in_min, double in_max, double out_min, double out_max);

void realtimeZscoreAlgorithm(double y[], uint32_t nCount, uint32_t lag, double threshold, double influence,
	double signals_[], double avgFilter_[], double filteredY_[], double stdFilter_[]);

#endif