/*
 *  ConvolutionalLogisticPolicy.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/10/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include <math.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "ConvolutionalLogisticPolicy.h"

using namespace std; 

ConvolutionalLogisticPolicy::ConvolutionalLogisticPolicy(CvSize gridSize) {
	this->gridSize.width = gridSize.width; 
	this->gridSize.height = gridSize.height; 
	polnum = GAUSSIAN; 
	softmaxGain = 300; 
	boxSize = 5; 
	actProb = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_32F, 1);			
}

ConvolutionalLogisticPolicy::ConvolutionalLogisticPolicy(ConvolutionalLogisticPolicy* clpToCopy) {
	this->gridSize.width = clpToCopy->gridSize.width; 
	this->gridSize.height = clpToCopy->gridSize.height; 
	polnum = clpToCopy->polnum; 
	softmaxGain = clpToCopy->softmaxGain; 
	boxSize = clpToCopy->boxSize; 
	actProb = cvCloneImage(clpToCopy->actProb);			
}


void ConvolutionalLogisticPolicy::readFromStream(istream& in) {
	in >> polnum; 
	in >> softmaxGain; 
	in >> boxSize;
}

void ConvolutionalLogisticPolicy::addToStream(ostream& out) {
	out << polnum << " " << softmaxGain << " " << boxSize << endl; 
}

void ConvolutionalLogisticPolicy::setPolicy(int policyNumber) {
	this->polnum = policyNumber; 
}

void ConvolutionalLogisticPolicy::setHeuristicPolicyParameters(double softmaxGain, double boxSize) {
	this->softmaxGain = softmaxGain; 
	this->boxSize = boxSize; 
}

ConvolutionalLogisticPolicy::~ConvolutionalLogisticPolicy() {
	cvReleaseImage(&(this->actProb)); 
}

CvPoint ConvolutionalLogisticPolicy::getFixationPoint(IplImage* currentBelief) {

	float targProb; 
	float cum; 
	switch (polnum) {
		case FULL:
			cerr << "FULL Convolutional Logistic Policies not yet implemented. Using Gaussian." << endl; 
			polnum = GAUSSIAN; 
			return getFixationPoint(currentBelief); 
		case BOX:
			cvCopy(currentBelief, actProb); 
			cvSmooth(actProb, actProb, CV_BLUR, boxSize,boxSize); 
			cvScale(actProb, actProb, softmaxGain); 
			cvExp(actProb, actProb); 
			cvNormalize(actProb, actProb, 1, 0, CV_L1, NULL); 
			targProb = randomFloat();  
			cum = 0; 
			for (int y = 0; y < actProb->height; y++) {
				for (int x = 0; x < actProb->width; x++) {				
					
					cum = cum+cvGetReal2D(actProb, y, x); 
					if (cum >= targProb) {
						return cvPoint(x, y); 
					}
				}
			}			
			break;
		case GAUSSIAN:
			cvCopy(currentBelief, actProb); 
			cvSmooth(actProb, actProb, CV_GAUSSIAN, boxSize,boxSize); 
			cvScale(actProb, actProb, softmaxGain); 
			cvExp(actProb, actProb); 
			cvNormalize(actProb, actProb, 1, 0, CV_L1, NULL); 
			targProb = randomFloat();  
			cum = 0; 
			for (int y = 0; y < actProb->height; y++) {
				for (int x = 0; x < actProb->width; x++) {				
					
					cum = cum+cvGetReal2D(actProb, y, x); 
					if (cum >= targProb) {
						return cvPoint(x, y); 
					}
				}
			}			
			break;
		case IMPULSE:
			cvCopy(currentBelief, actProb); 
			cvScale(actProb, actProb, softmaxGain); 
			cvExp(actProb, actProb); 
			cvNormalize(actProb, actProb, 1, 0, CV_L1, NULL); 
			targProb = randomFloat();  
			cum = 0; 
			for (int y = 0; y < actProb->height; y++) {
				for (int x = 0; x < actProb->width; x++) {				
					
					cum = cum+cvGetReal2D(actProb, y, x); 
					if (cum >= targProb) {
						return cvPoint(x, y); 
					}
				}
			}			
			break;
		case MAX:
			double min, max; 
			CvPoint min_loc, max_loc; 
			cvMinMaxLoc( currentBelief, &min, &max, &min_loc, &max_loc );
			return max_loc; 
		default:
			break;
	}
	return cvPoint(-1, -1); 
}

double ConvolutionalLogisticPolicy::getReward(IplImage* currentBelief) {
	double reward = 0;  
	for (int i = 0; i < gridSize.height; i++) {
		for (int j = 0; j < gridSize.width; j++) {
			double val = cvGetReal2D(currentBelief, i, j); 
			val = val * log(val); 
			if (isnan(val) )
				val = 0; 
			reward = reward + val; 
		}
	}
	return reward; 
}

float ConvolutionalLogisticPolicy::randomFloat() {
	long r = random(); 
	double retval = 0; 
	for (int i = 0; i < 16; i++) {
		retval = retval + (1.0/(2<<(i)))*((r&(1<<i))>>i);
	}
	return retval;
}


ostream& operator<< (ostream& ofs, ConvolutionalLogisticPolicy* model) {
	model->addToStream(ofs); 
	return ofs; 
}
istream& operator>> (istream& ifs, ConvolutionalLogisticPolicy* model) {
	model->readFromStream(ifs); 
	return ifs; 
}