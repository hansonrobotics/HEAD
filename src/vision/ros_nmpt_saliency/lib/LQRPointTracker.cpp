/*
 *  LQRPointTracker.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/5/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "LQRPointTracker.h"

using namespace cv; 

LQRPointTracker::LQRPointTracker(int numDims, double dt, double drag, double move_cost) {
	drag = drag < 0 ? 0 : drag; 
	drag = drag > 1 ? 1 : drag; 
	double pdrag = drag * dt * dt / 2; 
	drag = 1-drag; 
	numPoints = numDims; 
	
	
	double stateDynamics[3][3] = { {1, dt - pdrag, dt*dt / 2.0}, { 0, drag, dt}, {0, 0, 0}}; 
	double controlDynamics[3][1] = {{0}, {0}, {1}}; 
	double stateCost[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}; 
	double controlCost[1][1] = {{move_cost}}; 
	double homeState[3][1] = {{0}, {0}, {0}}; 
	
	Mat A(3, 3, CV_64F, stateDynamics); 
	Mat B(3, 1, CV_64F, controlDynamics); 
	Mat Q(3, 3, CV_64F, stateCost); 
	Mat R(1, 1, CV_64F, controlCost); 
	
	LQR oneDTracker(A, B, Q, R, 500); 
	states.clear(); 
	for (int i = 0; i < numPoints; i++) {
		states.push_back(LQR(oneDTracker)); 
	}
	currentPositions.resize(numDims, 0); 
	desiredPositions.resize(numDims, 0); 
	desiredState = Mat(3, 1, CV_64F, homeState).clone(); 
	
	
}

void LQRPointTracker::setTrackerTarget(const vector<double> &newPos) {
	for (size_t i = 0; i < newPos.size(); i++) {
		desiredState.at<double>(0,0)=newPos[i]; 
		states[i].setTarget(desiredState); 
		desiredPositions[i] = newPos[i]; 
	}
}

void LQRPointTracker::updateTrackerPosition() {
	for (int i = 0; i < numPoints; i++) {
		states[i].updateSteadyState(); 
		states[i].takeAction(); 
		Mat state =  states[i].getState(); 
		currentPositions[i] = state.at<double>(0,0); 
	}
}

vector<double> LQRPointTracker::getCurrentPosition() {
	return currentPositions; 
}
