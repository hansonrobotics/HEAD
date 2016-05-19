/*
 *  MIPOMDP.cpp
 *  FaceTracker
 *
 *  Created by Nicholas Butko on 10/3/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "MIPOMDP.h"
#include "DebugGlobals.h"
#include <math.h>
#include <stdlib.h>
#include <cassert>
#include <fstream>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <stdio.h>
#include <string>

using namespace std; 


MIPOMDP::MIPOMDP(CvSize inputImageSize, CvSize subImageSize, CvSize gridSize, int numSubImages, CvMat* subImageGridPoints, const char* haarDetectorXMLFile){

	this->gridSize = gridSize; 
	
	OpenCVHaarDetector* detector = new OpenCVHaarDetector(haarDetectorXMLFile); 
	
	ipp = new ImagePatchPyramid(inputImageSize, subImageSize, gridSize, numSubImages, subImageGridPoints, detector); 
	foveaRepresentation = ipp->foveaRepresentation; 
	
	policy = new ConvolutionalLogisticPolicy(gridSize); 
	
	//Allocate image size, type, memory
	currentBelief = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_32F, 1);	
	fixationCount = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_8U, 1);	
	
	//This takes care of a lot of initialization. 
	changeInputImageSize(inputImageSize, subImageSize); 
	
	obsmodel = new MultinomialObservationModel(gridSize); 
	
	dynamicsoff = 0; 
	repeatOff = 0; 
}

MIPOMDP::MIPOMDP() {
	const int numScales = 4; 
	int scaleGridCells[numScales]; 
	int ngrid_pts = 1; 
	for (int i = 0; i < numScales; i++) {
		int inc = ngrid_pts+2; 
		int halfwidth = (numScales - i - 1)*inc + ngrid_pts;
		
		scaleGridCells[i] =  halfwidth*2+1; //,9,15,21};  
		cout << "Scale " << i << ": " << scaleGridCells[i] << endl; 
	}
	int numGridPoints = scaleGridCells[0]; 
	
	CvSize inputImageSize = cvSize (1000, 1000); 
	CvSize subImageSize = cvSize(100, 100); 
	CvSize gridSize = cvSize(numGridPoints, numGridPoints); 
	CvMat* subImageGridPoints = cvCreateMat(numScales, 2, CV_32SC1); 
	for (int i = 0; i < numScales; i++) {
		cvSetReal2D(subImageGridPoints, i, 0, scaleGridCells[i]); 
		cvSetReal2D(subImageGridPoints, i, 1, scaleGridCells[i]); 
	}
	const char* haarDetectorXMLFile = "data/haarcascade_frontalface_alt2.xml";
	
	this->gridSize = gridSize; 
	
	OpenCVHaarDetector* detector = new OpenCVHaarDetector(haarDetectorXMLFile); 
	
	ipp = new ImagePatchPyramid(inputImageSize, subImageSize, gridSize, numScales, subImageGridPoints, detector); 
	foveaRepresentation = ipp->foveaRepresentation; 
	
	policy = new ConvolutionalLogisticPolicy(gridSize); 
	
	//Allocate image size, type, memory
	currentBelief = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_32F, 1);	
	fixationCount = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_8U, 1);	
	
	//This takes care of a lot of initialization. 
	changeInputImageSize(inputImageSize, subImageSize); 
	
	obsmodel = new MultinomialObservationModel(gridSize); 
	
	dynamicsoff = 0; 
	repeatOff = 0; 
}

MIPOMDP::MIPOMDP(CvSize gridSize) {
	this->gridSize = gridSize; 
	currentBelief = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_32F, 1);	
	fixationCount = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_8U, 1);		
	dynamicsoff = 0; 
	repeatOff = 0; 
	ipp = new ImagePatchPyramid(); 
	obsmodel = new MultinomialObservationModel(gridSize); 
	policy = new ConvolutionalLogisticPolicy(gridSize); 
	//Set others to zero just to be clean
	resetPrior(); 
	cvSetZero(fixationCount);  
}

void MIPOMDP::resetPrior() {
	cvSet(currentBelief, cvRealScalar(1.0/(gridSize.width*gridSize.height))); 
}

void MIPOMDP::saveToFile(const char* filename) {
	
	ofstream out; 
	out.open(filename); 
	out << gridSize.width << " " << gridSize.height << endl; 
	out << ipp;
	out << obsmodel; 
	out << policy; 
	out.close(); 	
}

MIPOMDP* MIPOMDP::loadFromFile(const char* filename){
	ifstream in; 
	in.open(filename); 
	int width, height; 
	in >> width; 
	in >> height; 
	
	MIPOMDP* retval = new MIPOMDP(cvSize(width,height)); 
	in >> retval->ipp; 
	in >> retval->obsmodel;	
	in >> retval->policy; 
	in.close(); 
	retval->foveaRepresentation = retval->ipp->foveaRepresentation; 
	return retval; 
}


MIPOMDP::~MIPOMDP() {
	cvReleaseImage(&(this->currentBelief)); 
	cvReleaseImage(&(this->fixationCount)); 
	delete(ipp); 
	delete(obsmodel); 
	delete(policy); 
}



void MIPOMDP::changeInputImageSize(CvSize newInputSize){
	ipp->changeInputImageSize(newInputSize); 
	foveaRepresentation = ipp->foveaRepresentation; 
	
	//Since we're resizing, we probably have a new image, which needs a new prior
	//CurrentBelief is the only thing that really needs to be set to an initial value
	resetPrior(); 
	
	//Set others to zero just to be clean
	cvSetZero(fixationCount); 
}

void MIPOMDP::changeInputImageSize(CvSize newInputSize, CvSize newSubImageSize){
	ipp->changeInputImageSize(newInputSize, newSubImageSize); 
	foveaRepresentation = ipp->foveaRepresentation; 
	
	//Since we're resizing, we probably have a new image, which needs a new prior
	//CurrentBelief is the only thing that really needs to be set to an initial value
	resetPrior(); 
	
	//Set others to zero just to be clean
	cvSetZero(fixationCount); 
}

double MIPOMDP::getProb() {	
	double min, max; 
	CvPoint min_loc, max_loc; 
	cvMinMaxLoc( currentBelief, &min, &max, &min_loc, &max_loc );
	return max; 
}

double MIPOMDP::getReward() {
	return policy->getReward(currentBelief); 
}

CvPoint MIPOMDP::getMostLikelyTargetLocation() {
	if (_MIPOMDP_DEBUG) cout << "getMostLikelyTargetLocation: Finding max of belief distribution." << endl; 
	double min, max; 
	CvPoint min_loc, max_loc; 
	cvMinMaxLoc( currentBelief, &min, &max, &min_loc, &max_loc );
	if (_MIPOMDP_DEBUG) cout << "getMostLikelyTargetLocation: Finding pixel at center of grid cell." << endl; 
	return pixelForGridPoint(max_loc); 
}

void MIPOMDP::runForwardBeliefDynamics() {	
	if (_MIPOMDP_DEBUG) cout << "runForwardBeliefDynaics: Adding a constant probability." << endl; 
	cvAddS(currentBelief, cvRealScalar(0.001), currentBelief); 
	if (_MIPOMDP_DEBUG) cout << "runForwardBeliefDynaics: Smoothing beleif map." << endl; 
	cvSmooth( currentBelief, currentBelief, CV_GAUSSIAN, 5,0,.25 );	
}

CvPoint MIPOMDP::searchNewFrame(IplImage* grayFrame) {
	if (_MIPOMDP_DEBUG) cout << "searchNewFrame: Beginning search of new frame." << endl; 
	return searchNewFrameAtGridPoint(grayFrame, recommendSearchPointForCurrentBelief()); 
}

CvPoint MIPOMDP::searchFrameUntilConfident(IplImage* grayFrame, double confidenceThresh){
	int olddynamics = dynamicsoff; 
	dynamicsoff = 1; 	
	double min, max; 
	CvPoint min_loc, max_loc; 
	max = 0; 
	cvSetZero(fixationCount); 
	ipp->setNewImage(); 
	while (max < confidenceThresh) {
		CvPoint searchPoint = recommendSearchPointForCurrentBelief(); 
		if (cvGetReal2D(fixationCount, searchPoint.y, searchPoint.x) > 0) {
			break; 
		}
		searchFrameAtGridPoint(grayFrame, searchPoint); 
		cvMinMaxLoc( currentBelief, &min, &max, &min_loc, &max_loc );
	}
	
	dynamicsoff = olddynamics; 
	return pixelForGridPoint(max_loc); 
}


CvPoint MIPOMDP::searchFrameRandomlyUntilConfident(IplImage* grayFrame, double confidenceThresh){
	int olddynamics = dynamicsoff; 
	dynamicsoff = 1; 	
	double min, max; 
	CvPoint min_loc, max_loc; 
	max = 0; 
	ipp->setNewImage(); 
	cvSetZero(fixationCount); 
	int count = 0; 
	CvSize gridSize = cvSize(ipp->objectCount->width, ipp->objectCount->height); 
	while (max < confidenceThresh) {
		CvPoint searchPoint = OpenLoopPolicies::getFixationPoint(count++, OpenLoopPolicies::RANDOM, gridSize);  
		if (cvGetReal2D(fixationCount, searchPoint.y, searchPoint.x) > 1) {
			break; 
		}
		searchFrameAtGridPoint(grayFrame, searchPoint); 
		cvMinMaxLoc( currentBelief, &min, &max, &min_loc, &max_loc );
	}
		
	dynamicsoff = olddynamics; 
	return pixelForGridPoint(max_loc); 
}

CvPoint MIPOMDP::searchFrameForNFixationsOpenLoop(IplImage* grayFrame, int numfixations, int OLPolicyType){
	int olddynamics = dynamicsoff; 
	dynamicsoff = 1; 	
	double min, max; 
	CvPoint min_loc, max_loc; 
	cvSetZero(fixationCount); 
	CvSize gridSize = cvSize(ipp->objectCount->width, ipp->objectCount->height); 
	ipp->setNewImage(); 
	for (int count = 0; count < numfixations; count++) {
		CvPoint searchPoint = OpenLoopPolicies::getFixationPoint(count, OLPolicyType, gridSize);  
		searchFrameAtGridPoint(grayFrame, searchPoint); 
	}
	cvMinMaxLoc( currentBelief, &min, &max, &min_loc, &max_loc );
	dynamicsoff = olddynamics; 
	return pixelForGridPoint(max_loc); 
}


CvPoint MIPOMDP::searchFrameForNFixations(IplImage* grayFrame, int numfixations){	
	int olddynamics = dynamicsoff; 
	int oldPreviewFlag= ipp->getGeneratePreview(); 
	ipp->setGeneratePreview(0); 
	dynamicsoff = 1; 	
	cvSetZero(fixationCount); 
	ipp->setNewImage(); 
	for (int i = 0; i < numfixations; i++) {
		CvPoint searchPoint = recommendSearchPointForCurrentBelief(); 
		searchFrameAtGridPoint(grayFrame, searchPoint); 
		if (i == numfixations-1) ipp->setGeneratePreview(oldPreviewFlag); 
	}
	dynamicsoff = olddynamics; 
	return getMostLikelyTargetLocation(); 
}


CvPoint MIPOMDP::searchFrameForNFixationsAndVisualize(IplImage* grayFrame, int numfixations, const char* window, int msec_wait){
	int olddynamics = dynamicsoff; 
	int oldPreviewFlag= ipp->getGeneratePreview(); 
	ipp->setGeneratePreview(1); 
	dynamicsoff = 1; 	
	double min, max; 
	CvPoint min_loc, max_loc; 
	cvSetZero(fixationCount); 
	ipp->setNewImage(); 
	for (int i = 0; i < numfixations; i++) {
		CvPoint searchPoint = recommendSearchPointForCurrentBelief(); 
		searchFrameAtGridPoint(grayFrame, searchPoint); 
		cvShowImage (window, foveaRepresentation);
		cvWaitKey(msec_wait); 
	}
	cvMinMaxLoc( currentBelief, &min, &max, &min_loc, &max_loc );		
	dynamicsoff = olddynamics; 
	ipp->setGeneratePreview(oldPreviewFlag); 
	return pixelForGridPoint(max_loc); 
}

CvPoint MIPOMDP::searchFrameAtAllGridPoints(IplImage* grayFrame){
	int numpts = gridSize.width*gridSize.height; 
	return searchFrameForNFixationsOpenLoop(grayFrame, numpts , OpenLoopPolicies::ORDERED); 
}

CvPoint MIPOMDP::searchHighResImage(IplImage* grayFrame){
	CvPoint retval = ipp->searchHighResImage(grayFrame); 
	double min_val, max_val; 
	cvMinMaxLoc(ipp->objectCount, &min_val, &max_val); 
	cvConvert(ipp->objectCount, currentBelief); 
	cvSubS(currentBelief, cvRealScalar(max_val), currentBelief); 
	cvExp(currentBelief, currentBelief); 
	cvNormalize(currentBelief, currentBelief, 1, 0, CV_L1); 
	return retval; 
} 

CvPoint MIPOMDP::gridPointForPixel(CvPoint pixel){ 
	
	if (_MIPOMDP_DEBUG) cout << "gridPointForPixel: Asking ipp what grid cell contains this pixel." << endl; 
	return ipp->gridPointForPixel(pixel); 
}

CvPoint MIPOMDP::pixelForGridPoint(CvPoint gridPoint) {
	if (_MIPOMDP_DEBUG) cout << "pixelForGridPoint: Asking ipp what pixel is in the middle of this grid point." << endl; 
	return ipp->pixelForGridPoint(gridPoint);  
}

CvPoint MIPOMDP::searchNewFrameAtGridPoint(IplImage* grayFrame, CvPoint searchPoint) {
	if (_MIPOMDP_DEBUG) cout << "searchNewFrameAtGridPoint: Telling ipp that it's a new image." << endl; 
	ipp->setNewImage(); 
	return searchFrameAtGridPoint(grayFrame, searchPoint); 
}

CvPoint MIPOMDP::searchFrameAtGridPoint(IplImage* grayFrame, CvPoint searchPoint) {
	
	if (_MIPOMDP_DEBUG) cout << "searchFrameAtGridPoint: Beginning search of at grid point " << searchPoint.x << ", " << searchPoint.y << endl; 
	if (_MIPOMDP_DEBUG) cout << "searchFrameAtGridPoint: Incrementing fixation count." << endl; 
 	cvSetReal2D(fixationCount, searchPoint.y, searchPoint.x, 
				cvGetReal2D(fixationCount, searchPoint.y, searchPoint.x)+1); 
	
	if (!dynamicsoff) {
		if (_MIPOMDP_DEBUG) cout << "searchFrameAtGridPoint: Running forward belief dynamics." << endl; 
		runForwardBeliefDynamics(); 
	}
	
	if (_MIPOMDP_DEBUG) cout << "searchFrameAtGridPoint: Building ipp representation and searching for face." << endl; 
	ipp->searchFrameAtGridPoint(grayFrame, searchPoint); 
	if (_MIPOMDP_DEBUG) cout << "searchFrameAtGridPoint: Consulting observation model about ipp output." << endl; 
	obsmodel->setObservationProbability(searchPoint, ipp->objectCount); 
	if (_MIPOMDP_DEBUG) cout << "searchFrameAtGridPoint: Updating belief distribution." << endl; 
	cvMul(obsmodel->observationProbability, currentBelief, currentBelief); 
	cvNormalize(currentBelief, currentBelief, 1, 0, CV_L1 ); //Normalize probability map to 1. 
	if (_MIPOMDP_DEBUG) cout << "searchFrameAtGridPoint: Finding most likely target location." << endl; 
	return getMostLikelyTargetLocation(); 
}

void MIPOMDP::resetModel() {	
	obsmodel->resetCounts(); 
}

void MIPOMDP::normalizeProbTables() {	
	obsmodel->normalizeProbTables(); 
}

void MIPOMDP::setObservationProbability(CvPoint searchPoint) {
	obsmodel->setObservationProbability(searchPoint, ipp->objectCount); 
}


void MIPOMDP::combineModels(MIPOMDP* otherPomdp) {
	obsmodel->combineEvidence(otherPomdp->obsmodel); 
}

CvPoint MIPOMDP::recommendSearchPointForCurrentBelief() {
	if (_MIPOMDP_DEBUG) cout << "recommendSearchPointForCurrentBelief: Consulting policy." << endl; 
	return policy->getFixationPoint(currentBelief); 
}


float MIPOMDP::randomFloat() {
	long r = random(); 
	double retval = 0; 
	for (int i = 0; i < 16; i++) {
		retval = retval + (1.0/(2<<(i)))*((r&(1<<i))>>i);
	}
	return retval;
}

void MIPOMDP::saveVisualization(IplImage* grayFrame, CvPoint searchPoint, const char* base_filename) { 	
	ipp->saveVisualization(grayFrame, searchPoint, base_filename); 
	
	
	cvSetReal2D(fixationCount, searchPoint.y, searchPoint.x, 
				cvGetReal2D(fixationCount, searchPoint.y, searchPoint.x)+1); 
	
	if (!dynamicsoff) {
		runForwardBeliefDynamics(); 
	}
	
	char filename[5000]; 
	snprintf(filename, 5000*sizeof(char), "%s-FullInputImage.png", base_filename); 
	cvSaveImage(filename, grayFrame); 

	setObservationProbability(searchPoint); 	
	
	ofstream out; 

	snprintf(filename, 5000*sizeof(char), "%s-ObservationProbability.csv", base_filename); 
	out.open(filename); 
	for (int i = 0; i < ipp->objectCount->height; i++) {
		for (int j = 0; j < ipp->objectCount->width; j++) {
			out << cvGetReal2D(obsmodel->observationProbability, i, j) << ", "; 
		}
		out << endl; 
	}
	out.close(); 
	
	//Here we're making a Naive Bayes assumption that given the presence of a face, the output of consecutive
	//Scales is independent, which is not true. Consecutive scales are likely to have more similar outputs. 
	
	cvMul(obsmodel->observationProbability, currentBelief, currentBelief); 
	
	cvNormalize(currentBelief, currentBelief, 1, 0, CV_L1 ); //Normalize probability map to 1. 

}

void MIPOMDP::setPolicy(int policyNumber) {	policy->setPolicy(policyNumber); }

void MIPOMDP::setHeuristicPolicyParameters(double softmaxGain, double boxSize) {policy->setHeuristicPolicyParameters(softmaxGain, boxSize);} 

void MIPOMDP::setGeneratePreview(int flag) {ipp->setGeneratePreview(flag);} 

void MIPOMDP::setMinSize(CvSize minsize) { ipp->setMinSize(minsize);}

void MIPOMDP::useSameFrameOptimizations(int flag) {ipp->useSameFrameOptimizations(flag);}

CvSize MIPOMDP::getGridSize(){return gridSize;} 

int MIPOMDP::getNumScales(){return ipp->getNumScales(); } 

void MIPOMDP::setHaarCascadeScaleFactor(double factor) { ((OpenCVHaarDetector*)ipp->detector)->setHaarCascadeScaleFactor(factor);}

void MIPOMDP::setHaarCascadeMinSize(int size) {	((OpenCVHaarDetector*)ipp->detector)->setHaarCascadeMinSize(size); }

void MIPOMDP::setTargetCanMove(int flag) {dynamicsoff = !flag;}

int MIPOMDP::getTargetCanMove() {return !dynamicsoff;}

IplImage* MIPOMDP::getCounts() {return ipp->objectCount; } 

void MIPOMDP::trainObservationModel(ImageDataSet* trainingSet){
	obsmodel->resetCounts(); 
	addDataToObservationModel(trainingSet); 
}

void MIPOMDP::addDataToObservationModel(ImageDataSet* trainingSet){	
	int oldPreviewFlag = ipp->getGeneratePreview();
	int oldOptFlag = ipp->getSameFrameOptimizations(); 
	ipp->useSameFrameOptimizations(1); 
	ipp->setGeneratePreview(0);
	CvSize oldImageSize = ipp->getInputImageSize(); 
	CvSize oldSubImageSize = ipp->getSubImageSize(); 
	
	for (int j = 0; j < trainingSet->getNumEntries(); j++) { 
		CvRect objectLocation; 
		string filename = trainingSet->getFileName(j);
		vector<double> labels = trainingSet->getFileLabels(j); 
		objectLocation.x = labels[0]; 
		objectLocation.y = labels[1]; 
		objectLocation.width = labels[2]; 
		objectLocation.height = labels[2]; 
		cout << "Image " << filename << " has Face at " << objectLocation.x << ", " << objectLocation.y << endl; 
		IplImage* current_frame = cvLoadImage( filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE ); 
		
		int w1 = current_frame->width; 
		int h1 = current_frame->height; 
		
		changeInputImageSize(cvSize(w1, h1));//, cvSize(w2,h2)); 
		
		//This calculation makes it so that the smallest scale has 1:1 mapping of pixels. 
		int numScales = ipp->getUsedScales(); 
		int smallWidth = ipp->getGridCellWidthOfScale(numScales-1);
		int smallHeight = ipp->getGridCellHeightOfScale(numScales-1); 
		int w2 = w1 * smallWidth / gridSize.width; //e.g. w1*3/21
		int h2 = h1 * smallHeight / gridSize.height; //e.g. h1*3/21			
		
		
		cout << "Image Size: " << w1 << "x" << h1 << "; Smallest Grid Cells: " << smallWidth << "x" << smallHeight << 
		"; Smallest Patch Pixels: " << w2 << "x" << h2 << endl; 
		
		updateProbTableCounts(current_frame, objectLocation); 
		
		cvReleaseImage(&current_frame); 
		
	}
	
	
	obsmodel->normalizeProbTables(); 
	changeInputImageSize(oldImageSize,oldSubImageSize); 
	
	ipp->setGeneratePreview(oldPreviewFlag);
	ipp->useSameFrameOptimizations(oldOptFlag);
}

void MIPOMDP::updateProbTableCounts(IplImage* grayFrame, CvRect objectLocation) {
	int x = objectLocation.x ;//- objectLocation.width/2; 
	int y = objectLocation.y ;//- objectLocation.height/2; 
	objectLocation.x = x;
	objectLocation.y = y; 
	objectLocation.width = 1; 
	objectLocation.height = 1; 
	ipp->setNewImage(); 
	CvPoint ol = cvPoint(x,y); 
	CvPoint gl = ipp->gridPointForPixel(ol); 
	objectLocation.x = gl.x; 
	objectLocation.y = gl.y; 
	cout << "Face is at grid cell (" << objectLocation.x << "," << objectLocation.y << "), Width is " << objectLocation.width << "; Height is " << objectLocation.height << endl; 
	
	//x1,y1 is where we're looking now
	for (int y1 = 0; y1 < ipp->objectCount->height; y1++) {
		for (int x1 = 0; x1 < ipp->objectCount->width; x1++) {						
			CvPoint searchPoint = cvPoint(x1,y1); 
			ipp->searchFrameAtGridPoint(grayFrame, searchPoint); 
			obsmodel->updateProbTableCounts(searchPoint, ipp->objectCount, objectLocation); 
		}
	}
}

void MIPOMDP::setObjectDetectorSource(string newFileName) {
	ipp->setObjectDetectorSource(newFileName); 
}
