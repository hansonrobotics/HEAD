#include "BlockTimer.h"

#include <sys/time.h>
#include <time.h>

BlockTimer::BlockTimer(int numblocks) :
    currTimerProc(numblocks, 0), 
    totTimerProc(numblocks,0.0) {   // totTimerAbs(numblocks,0.0),  currTimerAbs(numblocks,0.0), 
        currsize=numblocks; 
    }

void BlockTimer::addEls(int newmax) {
    currTimerProc.resize(newmax,0); 
    totTimerProc.resize(newmax,0.0); 
    currsize = newmax; 
}

void BlockTimer::blockStart(int block) {
    if (currsize < block+1) addEls(block+1); 
	blockStop(block); 
	timertype start = getCurrentTimeInMicroseconds(); 	
	currTimerProc[block] = start; 
}

void BlockTimer::blockStop(int block) {
    if (currsize < block+1) addEls(block+1); 
    timertype start = currTimerProc[block]; 	

	timertype end = getCurrentTimeInMicroseconds(); 
    if (start > 0 ) totTimerProc[block] += (end-start)*1.0/1000000; 
	
    currTimerProc[block] = 0; 
	
}

void BlockTimer::blockReset(int block) {
    if (currsize < block+1) addEls(block+1); 
    currTimerProc[block] = 0; 
    totTimerProc[block] = 0; 
}

void BlockTimer::blockRestart(int block) {
    blockReset(block);
	blockStart(block); 
}

double BlockTimer::getCurrTime(int block) {
    if (currsize < block+1) addEls(block+1); 
    timertype start = currTimerProc[block]; 
	timertype end = getCurrentTimeInMicroseconds(); 
	
    if (start > 0) return (end-start)*1.0/1000000; 
    return 0; 
}

double BlockTimer::getTotTime(int block) {
    if (currsize < block+1) addEls(block+1); 
    timertype start = currTimerProc[block]; 
	
	timertype end = getCurrentTimeInMicroseconds(); 
    if (start > 0) return (start-end)*1.0/1000000 + totTimerProc[block]; 
    return totTimerProc[block]; 
}

BlockTimer::~BlockTimer() {
	//currTimerProc();
	//~totTimerProc(); 
}

timertype BlockTimer::getCurrentTimeInMicroseconds() {
	timertype now = 0; 
	struct timeval tp; 
	struct timezone tzp; 
	gettimeofday(&tp, &tzp); 
	now += ((timertype) tp.tv_sec)*1000000+(timertype)tp.tv_usec; 
	
	time_t rawtime;
	struct tm * timeinfo;	
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	timertype numdays = timeinfo->tm_yday + 365*timeinfo->tm_year + (timeinfo->tm_year)/4; 
	timertype microsecondsPerDay = 1440*60*1000000; 
	now += numdays*microsecondsPerDay; 	
	
    return now; 
}
