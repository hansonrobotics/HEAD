#ifndef __BLOCKTIMER_H
#define __BLOCKTIMER_H


#include <vector>

typedef long long timertype; 

/**
 *\ingroup AuxGroup
 *\brief <tt>Auxilliary Tool:</tt> A class for tracking the absolute runtime of several (possibly overlapping) blocks of code. 
 *
 * The basic philosophy of the BlockTimer class is that we want to arbitrarily invoke (start/stop/reset) several 
 * stop watches, each of which should have reasonable time precision ( < 1 ms) across several platforms. So
 * at any time, one can start stopwatch 3, stop stopwatch 1, and reset stopwatch 42. We track time using the posix-standard
 * functions "gettimeofday" and "localtime" to track the elapsed number of microseconds. 
 *
 * The basic operations of a timer are:
 * \li <tt>Start:</tt> Begin a timer incrementing from its current time.
 * \li <tt>Stop:</tt> Stop a timer, and keep its current time.
 * \li <tt>Reset:</tt> Set a timer's value to zero. 
 *
 * The basic readouts of a timer are: 
 * \li <tt> Current Time: </tt> The amount of elapsed time (in seconds) since the timer was last started. If the timer is 
 * stopped, Current Time is the amount of time that the timer was running from the last time it was started until
 * it was stopped. If the timer has been reset and not started, Current Time is Zero. 
 * \li <tt> Total Time: </tt> The amount of time that the timer has run since it was last reset.
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */

class BlockTimer {
private:
	std::vector<timertype> currTimerProc;
	std::vector<double> totTimerProc; 
	int currsize; 
	void addEls(int newmax);
	
public:
	/**
	 *\brief Constructor: Create a new set of stopwatches to time blocks of code.
	 *
	 *@param numblocks An initial guess at the  maximum number of stopwatches you will need. This is for memory 
	 *allocation. If you attempt to access a stopwatch with a greater index than you have memory for, more 
	 *memory will be allocated at that time. 
	 */
	BlockTimer(int numblocks=100);
	
	/**
	 *\brief Cause a stopwatch to begin incrementing. This will begin increasing both the current time and the 
	 * total time.
	 *
	 *@param block The index of the stopwatch to start (indexed from 0->N-1).
	 */
	void blockStart(int block);
	
	
	/**
	 *\brief Cause a stopwatch to stop incrementing. This will freeze the total time and
	 * reset the current time to 0.
	 *
	 *@param block The index of the stopwatch to stop (indexed from 0->N-1).
	 */
	void blockStop(int block);
	
	/**
	 *\brief Cause a stopwatch to reset. This will set both the current time and the 
	 * total time to 0.
	 *
	 *@param block The index of the stopwatch to reset (indexed from 0->N-1).
	 */
	void blockReset(int block);
	
	/**
	 *\brief Calls blockReset(block) and then blockStart(block).
	 *
	 *@param block The index of the stopwatch to start (indexed from 0->N-1).
	 */
	void blockRestart(int block);
	
	/**
	 *\brief Get a stopwatch's current time, in seconds. Current time is amount of time
	 *from when it was last started until (1) now, or (2) it was stopped. If the stopwatch
	 *has not been started since it was reset, the current time is 0.
	 *
	 *@param block The index of the stopwatch (indexed from 0->N-1).
	 */
	double getCurrTime(int block);
	
	/**
	 *\brief Get a stopwatch's total time, in seconds. Total time is amount of time
	 *that the stopwatch has been running since it was last reset. If the stopwatch
	 *has not been started since it was reset, the current time is 0.
	 *
	 *@param block The index of the stopwatch (indexed from 0->N-1).
	 */
	double getTotTime(int block);
	
	/**
	 *\brief Simple Desturctor. Frees all memory associated with the set of timers.
	 */
	~BlockTimer() ;
	
	
	/**
	 *\brief Use BSD standard system libraries and C-standard libraries to get 
	 * the system current time in  microseconds. 
	 *
	 * Specifically, this uses
	 * gettimeofday, time, and localtime. These are expected to be located in
	 * <sys/time.h> and <time.h>. If the compiler gives you trouble,
	 * try "man gettimeofday", "man time", and "man localtime" to figure out 
	 * what headers you should use on your system, and include them in
	 * BlockTimer.cpp. 
	 */
	static timertype getCurrentTimeInMicroseconds(); 
};


#endif
