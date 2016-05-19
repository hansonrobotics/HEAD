//
//  KF.h
//  OpenCV
//
//  Created by Nicholas Butko on 8/21/10.
//  Copyright (c) 2010 __MyCompanyName__. All rights reserved.
//

#ifndef _KF_H
#define _KF_H

#include <opencv2/core/core.hpp>

class KF {
public:
	KF(); 
	KF(const KF& copy); 
	KF(cv::Mat mu0, cv::Mat Sigma0, cv::Mat A, cv::Mat B, cv::Mat R, cv::Mat C, cv::Mat Q);
	
	//For a kalman filter with m actions, n states, and o observations: 
	//mu0: state prior mean, [nx1]
	//Sigma0: state prior variance [nxn]
	//A: System Dynamics [nxn]
	//B: Action Dynamics [nxm]
	//R: State Drift [nxn]
	//C: Observation Transform [oxn]
	//Q: Observation Noise [oxo]
	void initialize(cv::Mat mu0, cv::Mat Sigma0, cv::Mat A, cv::Mat B, cv::Mat R, cv::Mat C, cv::Mat Q); 
	
	//z: observation, [ox1]
	//u: action, [mx1]
	void updateFilter(cv::Mat z); 
	void updateFilter(cv::Mat z, cv::Mat u); 
	
	void updateFilterBlind();
	void updateFilterBlind(cv::Mat u); 
	
	//mu: [nx1]
	cv::Mat getStateMean(); 
	//Sigma: [nxn]
	cv::Mat getStateVariance(); 
	
private:
	int numStates, numActions, numObservations; 
	cv::Mat mu, Sigma, A, B, R, C, Q, I, At, Ct;  
	
}; 

#endif