/*
 *  LQRPointTracker.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/5/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include <opencv2/core/core.hpp>
#include <vector> 
#include "LQR.h"

#ifndef _LQRPOINTTRACKER_H
#define _LQRPOINTTRACKER_H


class PointTracker {	
	virtual void setTrackerTarget(const std::vector<double> &newPosition) = 0; 
	virtual void updateTrackerPosition() = 0;
	virtual std::vector<double> getCurrentPosition() = 0; 
}; 


class LQRPointTracker : PointTracker {
public: 
	LQRPointTracker(int numDims=1, double dt = 0.125, double drag = 0, double move_cost = .015); 
	virtual void setTrackerTarget(const std::vector<double> &newPosition); 
	virtual void updateTrackerPosition();
	virtual std::vector<double> getCurrentPosition(); 
	
private:
    std::vector<double> currentPositions; 
    std::vector<double> desiredPositions; 
    cv::Mat desiredState; 
    std::vector<LQR> states; 
	int numPoints; 
	
};

#endif