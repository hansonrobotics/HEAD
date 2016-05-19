//
//  LQR.cpp
//  OpenCV
//
//  Created by Nicholas Butko on 8/21/10.
//  Copyright (c) 2010 __MyCompanyName__. All rights reserved.
//

#include "LQR.h"

using namespace cv; 

LQR::LQR() {
}


LQR::LQR(const LQR& copy) {
	A = copy.A.clone(); 
	B = copy.B.clone(); 
	Q = copy.Q.clone(); 
	T = copy.T; 
	t = copy.t; 
	P.clear(); 
	for (size_t i = 0; i < copy.P.size(); i++) {
		P.push_back(copy.P[i].clone()); 
	}
	W.clear(); 
	for (size_t i = 0; i < copy.W.size(); i++) {
		W.push_back(copy.W[i].clone()); 
	}
	state = copy.state.clone(); 
	action = copy.action.clone(); 
	target = copy.target.clone(); 
}

LQR::LQR(Mat stateDynamics, Mat controlDynamics, Mat stateCost, Mat controlCost, int timeHorizon) {
	setDynamics( stateDynamics,  controlDynamics,  stateCost,  controlCost,  timeHorizon); 
}

void LQR::setDynamics(Mat stateDynamics, Mat controlDynamics, Mat stateCost, Mat controlCost, int timeHorizon) {
	T = timeHorizon; 
	P.resize(T); 
	W.resize(T); 
	t = 0; 
	A = stateDynamics; 
	B = controlDynamics; 
	Q = stateCost; 
	R = controlCost; 
	
	state.create(A.cols, 1, CV_64FC1); 
	action.create(B.cols, 1, CV_64FC1); 
	target.create(A.cols, 1, CV_64FC1);
	state.setTo(0); 
	action.setTo(0); 
	target.setTo(0); 
	
	
	P[T-1] = Q; 
	
	Mat Bt = B.t(); 
	Mat At = A.t(); 
	
	
	for (t = T-2; t>= 0; t--) {
		W[t] = Bt * P[t+1] *B + R; 
		W[t] = -(W[t].inv());
		
		W[t] = W[t] * Bt * P[t+1] * A; 
		
		Mat AtP = At*P[t+1]; 
		
		P[t] = Q + AtP * A + AtP * B * W[t]; 
	}
	
}



void LQR::setTarget(Mat newTarget) {
	state = state+target; 
	state = state - newTarget; 
	target = newTarget.clone(); 
}

void LQR::setState(Mat newState) {
	state = newState - target; 
}

Mat LQR::getState() {
	return state + target; 
}

void LQR::updateState() {
	computeBestAction(); 
	takeAction(); 
	t++; 
}

void LQR::updateSteadyState() {
	computeBestAction(0); 
}

void LQR::computeBestAction() {
	computeBestAction(t); 
}

void LQR::computeBestAction(int timepoint) {
	if (timepoint < T-1)
		action = W[timepoint]*state; 
	else
		action.setTo(0); 
}

void LQR::takeAction() {
	if (t < T-1) {
		state = A * state; 
		state = state + (B * action); 
	}
}

void LQR::setTimePoint(int timepoint) {
	t = timepoint; 
}

int LQR::getTimePoint() {
	return t; 
}