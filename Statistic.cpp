//
//    FILE: Statistic.cpp
//  AUTHOR: Jinzhouyun
//	Date:	2017.05

#include "Statistic.h"

Statistic::Statistic()
{
    clear();
}

// resets all counters
void Statistic::clear()
{
    _cnt = 0;
    _sum = 0.0;
    _min = 0.0;
    _max = 0.0;
#ifdef STAT_USE_STDEV
    _ssqdif = 0.0;  // not _ssq but sum of square differences
    // which is SUM(from i = 1 to N) of
    // (f(i)-_ave_N)**2
#endif
}

// adds a new value to the data-set
void Statistic::add(double value)
{
    if (_cnt == 0)
    {
        _min = value;
        _max = value;
    } else {
        if (value < _min) _min = value;
        else if (value > _max) _max = value;
    }
    _sum += value;
    _cnt++;
    
#ifdef STAT_USE_STDEV
    if (_cnt > 1)
    {
        _store = (_sum / _cnt - value);
        _ssqdif = _ssqdif + _cnt * _store * _store / (_cnt-1);
    }
#endif
}

// returns the average of the data-set added sofar
double Statistic::average()
{
    if (_cnt == 0) return NAN; // original code returned 0
    return _sum / _cnt;
}

// Population standard deviation = s = sqrt [ S ( Xi - µ )2 / N ]
#ifdef STAT_USE_STDEV

double Statistic::variance()
{
    if (_cnt == 0) return NAN; // otherwise DIV0 error
    return _ssqdif / _cnt;
}

double Statistic::pop_stdev()
{
    if (_cnt == 0) return NAN; // otherwise DIV0 error
    return sqrt( _ssqdif / _cnt);
}

double Statistic::unbiased_stdev()
{
    if (_cnt < 2) return NAN; // otherwise DIV0 error
    return sqrt( _ssqdif / (_cnt - 1));
}

#endif
// END OF FILE