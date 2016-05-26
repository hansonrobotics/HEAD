/*
 *  KFPointTracker.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/7/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "KFPointTracker.h"
#include "KF.h"

using namespace cv; 

KFPointTracker::KFPointTracker(int numDims, double dt, double posNoise, double obsNoise, double initUncertainty) {
	
	
	double stateDynamics[3][3] = { {1, dt, dt*dt / 2.0}, { 0, 1, dt}, {0, 0, 0}}; 
	double actMat[3][1] = {{0},{0},{0}}; 
	double obsMat[1][3] = {{1, 0, 0}}; 
	double stateNoise[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, posNoise}}; 
	double observNoise[1][1] = {{obsNoise}}; 
	double statePrior[3][1] = {{0}, {0}, {0}}; 
	double statePriorNoise[3][3] = { {0, 0, 0}, { 0, 0, 0}, {0, 0, initUncertainty}}; 	
	
	Mat A(3, 3, CV_64F, stateDynamics); 
	Mat B(3, 1, CV_64F, actMat); 
	Mat R(3, 3, CV_64F, stateNoise); 

	Mat C(1, 3, CV_64F, obsMat); 
	Mat Q(1, 1, CV_64F, observNoise); 
		
	Mat mu0(3, 1, CV_64F, statePrior); 
	Mat Sigma0(3, 3, CV_64F, statePriorNoise); 
	
	KF tracker(mu0, Sigma0, A, B, R, C, Q); 
	
	
	numPoints = numDims; 
	states.clear(); 
	for (int i = 0; i < numPoints; i++) {
		states.push_back(KF(tracker)); 
	}
	
	currObs = Mat::zeros(Q.size(), Q.type()); 
	currentPositions.resize(numPoints,0); 
	currentObservations.resize(numPoints,0); 	
}
void KFPointTracker::setTrackerTarget(const vector<double> &newPosition){
	for (int i = 0; i < numPoints; i++) {
		currentObservations[i] = newPosition[i]; 
	}
}
void KFPointTracker::updateTrackerPosition(){
	for (int i = 0; i < numPoints; i++) {
		currObs.at<double>(0,0) = currentObservations[i]; 
		states[i].updateFilter(currObs); 
		currentPositions[i] = states[i].getStateMean().at<double>(0,0); 
	}
}
vector<double> KFPointTracker::getCurrentPosition(){
	return currentPositions; 
}