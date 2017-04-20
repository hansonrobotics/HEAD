/*
 *  KFPointTracker.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/7/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */



#ifndef _KFPOINTTRACKER_H
#define _KFPOINTTRACKER_H

#include <opencv2/core/core.hpp>
#include <vector> 
#include "LQRPointTracker.h"
#include "KF.h"

class KFPointTracker : PointTracker {
public: 
	KFPointTracker(int numDims=1, double dt=0.0625, double posNoise = .03125, double obsNoise = .125, double initUncertainty = 0); 
	virtual void setTrackerTarget(const std::vector<double> &newPosition); 
	virtual void updateTrackerPosition();
	virtual std::vector<double> getCurrentPosition(); 
	
private:
    std::vector<double> currentPositions; 
    std::vector<double> currentObservations; 
    cv::Mat currObs; 
    std::vector<KF> states; 
	int numPoints; 
	
};

#endif