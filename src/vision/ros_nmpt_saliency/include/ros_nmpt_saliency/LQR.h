//
//  LQR.h
//  OpenCV
//
//  Created by Nicholas Butko on 8/21/10.
//  Copyright (c) 2010 __MyCompanyName__. All rights reserved.
//

#ifndef _LQR_H
#define _LQR_H

#include <opencv2/core/core.hpp>
#include <vector>

class LQR {
public:
	LQR(); 
	LQR(const LQR& copy); 
	LQR(cv::Mat stateDynamics, cv::Mat controlDynamics, cv::Mat stateCost, cv::Mat controlCost, int timeHorizon); 
	
	void setDynamics(cv::Mat stateDynamics, cv::Mat controlDynamics, cv::Mat stateCost, cv::Mat controlCost, int timeHorizon);  
	void setState(cv::Mat newState); 
	cv::Mat getState(); 
	void setTarget(cv::Mat newTarget); 
	void updateState(); 
	void updateSteadyState(); 
	void computeBestAction(); 
	void computeBestAction(int timepoint); 
	void takeAction(); 
	void setTimePoint(int timepoint); 
	int getTimePoint(); 
	
private:
	cv::Mat A, B, Q, R; 
	std::vector<cv::Mat> P, W;
	int T, t; 
	cv::Mat state, action, target; 
	
}; 

#endif