//
//  KF.cpp
//  OpenCV
//
//  Created by Nicholas Butko on 8/21/10.
//  Copyright (c) 2010 __MyCompanyName__. All rights reserved.
//

#include "KF.h"

using namespace cv; 

KF::KF(){
	this->numStates = 0; 
	this->numActions = 0; 
	this->numObservations = 0; 
}

KF::KF(const KF& copy) {
	initialize(copy.mu, copy.Sigma, copy.A, copy.B, copy.R, copy.C, copy.Q); 
}

KF::KF(Mat mu0, Mat Sigma0, Mat A, Mat B, Mat R, Mat C, Mat Q) {
	initialize(mu0, Sigma0, A, B, R, C, Q); 
}

//For a kalman filter with m actions, n states, and o observations: 
//mu0: state prior mean, [nx1]
//Sigma0: state prior variance [nxn]
//A: System Dynamics [nxn]
//B: Action Dynamics [nxo]
//R: State Drift [nxn]
//C: Observation Transform [oxn]
//Q: Observation Noise [oxo]
void KF::initialize(Mat mu0, Mat Sigma0, Mat A, Mat B, Mat R, Mat C, Mat Q) {
	this->numStates = mu0.cols; 
	this->numActions = B.cols; 
	this->numObservations = C.rows; 
    
	this->mu = mu0.clone(); 
	this->Sigma = Sigma0.clone(); 
	this->A = A.clone(); 
	this->B = B.clone(); 
	this->R = R.clone(); 
	this->C = C.clone(); 
	this->Q = Q.clone(); 
	
	this->At = A.t(); 
	this->Ct = C.t(); 
	this->I = Mat::eye(Sigma.size(), Sigma.type()); 
}

//z: observation, [ox1]
//u: action, [mx1]
void KF::updateFilter(Mat z, Mat u) {
	Mat hatmu = A*mu;
	hatmu += B*u; 
	Mat hatSigma = A*Sigma*At + R; 
	Mat K = hatSigma * Ct * (C * hatSigma * Ct + Q).inv(); 
	mu = hatmu + K * (z - C*hatmu); 
	Sigma = (I - K*C)*hatSigma; 
}


void KF::updateFilter(Mat z) {
	Mat hatmu = A*mu;
	Mat hatSigma = A*Sigma*At + R; 
	Mat K = hatSigma * Ct * (C * hatSigma * Ct + Q).inv(); 
	mu = hatmu + K * (z - C*hatmu); 
	Sigma = (I - K*C)*hatSigma; 
}

void KF::updateFilterBlind() {
	mu = A*mu;
	Sigma = A*Sigma*At; 
	Sigma += R; 
}

void KF::updateFilterBlind(Mat u) {
	mu = A*mu;
	mu += B*u; 
	Sigma = A*Sigma*At; 
	Sigma += R; 
}


//mu: [nx1]
Mat KF::getStateMean() {
	return mu.clone(); 
}

//Sigma: [nxn]
Mat KF::getStateVariance() {
	return Sigma.clone(); 
}