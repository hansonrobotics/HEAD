/*
 *  OpenLoopPolicies.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/9/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include <iostream>
#include "OpenLoopPolicies.h"

using namespace std; 

CvPoint OpenLoopPolicies::getFixationPoint(int fixationNumber, int policyType, CvSize gridSize){
	switch (policyType) {
		case RANDOM:
			return getRandomPoint(gridSize); 
		case ORDERED:
			return getOrderedPoint(fixationNumber, gridSize); 
		case SPIRAL:
			return getSpiralingPoint(fixationNumber, gridSize); 
		default:
			cerr << "Unrecognized Open-Loop Policy; picking middle point." << endl; 
	}
	return cvPoint((int)gridSize.width/2, (int)gridSize.height/2); 
}

CvPoint OpenLoopPolicies::getRandomPoint(CvSize gridSize) {
	double randfloat1 = randomFloat(); 
	double randfloat2 = randomFloat(); 
	int x  = randfloat1 * gridSize.width; 
	int y  = randfloat2 * gridSize.height; 
	return cvPoint(x,y); 
}

CvPoint OpenLoopPolicies::getOrderedPoint(int fixationNumber, CvSize gridSize) {
	return cvPoint(fixationNumber%gridSize.width, fixationNumber / gridSize.width);         
}

CvPoint OpenLoopPolicies::getSpiralingPoint(int fixationNumber, CvSize gridSize) {
	CvMat* distmat = cvCreateMat(gridSize.width, gridSize.height, CV_32SC1); 
	double min; 
	double max; 
	CvPoint searchPoint; 
	CvPoint max_loc; 
	
	for (int i = 0; i < gridSize.height ; i++) {
		for (int j = 0; j < gridSize.width; j++) {
			int dist = abs(i-gridSize.height/2) > abs(j-gridSize.width/2)?
			abs(i-gridSize.height/2):abs(j-gridSize.width/2); 
			cvSetReal2D(distmat, i, j, dist); 
		}
	}
	
	for (int i = 0; i < fixationNumber; i++) {
		cvMinMaxLoc(distmat, &min, &max, &searchPoint, &max_loc);
		cvSetReal2D(distmat, searchPoint.y, searchPoint.x, max+1); 
	}	
	return searchPoint; 
}



float OpenLoopPolicies::randomFloat() {
	long r = random(); 
	double retval = 0; 
	for (int i = 0; i < 16; i++) {
		retval = retval + (1.0/(2<<(i)))*((r&(1<<i))>>i);
	}
	return retval;
}