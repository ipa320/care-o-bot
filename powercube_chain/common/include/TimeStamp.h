//-----------------------------------------------
// Neobotix 
// www.neobotix.de
// Copyright (c) 2003. All rights reserved.

// author: 
//-----------------------------------------------

#ifndef _TimeStamp_H
#define _TimeStamp_H

#ifdef __LINUX__
#include <time.h>
#else
#include <windows.h>
#ifdef _DEBUG
//#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif
namespace RTB {

#endif

//-------------------------------------------------------------------

 
/** Measure system time with very high accuracy.
 * Use this class for measure system time accurately. Under Windows, it uses 
 * QueryPerformanceCounter(), which has a resolution of approx. one micro-second.
 * The difference between two time stamps can be calculated.
 */
class TimeStamp
{
public:
	/// Constructor.
	TimeStamp();

	/// Destructor.
	virtual ~TimeStamp() {};

	/// Makes time measurement.
	void SetNow();

	/// Retrieves time difference in seconds.
	double operator-(const TimeStamp& EarlierTime) const;

	/// Increase the timestamp by TimeS seconds.
	/** @param TimeS must be >0!.
	 */
	void operator+=(double TimeS);

	/// Reduces the timestamp by TimeS seconds.
	/** @param TimeS must be >0!.
	 */
	void operator-=(double TimeS);

	/// Checks if this time is after time "Time".
	bool operator>(const TimeStamp& Time);

	/// Checks if this time is before time "Time".
	bool operator<(const TimeStamp& Time);
	
	/**
	 * Gets seconds and nanoseconds of the timestamp.
	 */
	void getTimeStamp(long& lSeconds, long& lNanoSeconds);

	/**
	 * Sets timestamp from seconds and nanoseconds.
	 */
	void setTimeStamp(const long& lSeconds, const long& lNanoSeconds);
	

protected:

	/// Internal time stamp data.
	#ifdef __LINUX__
	timespec m_TimeStamp;
	#else
	LARGE_INTEGER m_TimeStamp;
	#endif
	
private:

	/// Conversion timespec -> double
	#ifdef __LINUX__
	static double TimespecToDouble(const ::timespec& LargeInt);

	/// Conversion double -> timespec
	static ::timespec DoubleToTimespec(double TimeS);
	#else
	static double LargeIntToDouble(const LARGE_INTEGER& LargeInt);

	/// Conversion double -> LARGE_INTEGER.
	static LARGE_INTEGER DoubleToLargeInt(double TimeS);

	/// Size of a Dword field.
	static double m_DwordSize;

	/// Seconds per counts.
 	static double m_SecPerCount;
	#endif

};

#ifdef __LINUX__
#else
}//namespace
#endif
#endif

