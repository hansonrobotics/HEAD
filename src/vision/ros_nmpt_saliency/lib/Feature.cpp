/*
 *  Feature.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/6/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "Feature.h"
#include "BoxFeature.h"
#include "HaarFeature.h"
#include "BlockTimer.h"
#include "DebugGlobals.h"
#include "NMPTUtils.h"

#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <sstream>

using namespace std; 
using namespace NMPTUtils; 
using namespace cv; 


double Feature::evaluateImagePatch(const ImagePatch &patch) {
	return evaluateImagePatch(&patch); 
}


void Feature::setFeatureParameters(const CvMat* paramVec) {
	const Mat newParamVec = paramVec; 
	setFeatureParameters(newParamVec); 
}
	

Feature::Feature(Size expectedPatchSize) {	
	parameters = NULL;
	minParamVals = NULL;
	maxParamVals = NULL;
	kernel = NULL; 
	parameterType = discrete; 
	patchSize = expectedPatchSize; 
	prefersIntegral = 0; 
	featureName = (char*)malloc(128*sizeof(char)); 
	snprintf(featureName, 128*sizeof(char), "Feature"); 
	rng = cvRNG(BlockTimer::getCurrentTimeInMicroseconds());
}

Feature::~Feature() {
	cvReleaseMat(&parameters); 
	cvReleaseMat(&minParamVals); 
	cvReleaseMat(&maxParamVals); 
	cvReleaseImage(&kernel) ;
	free(featureName); 
}

vector<Feature*> Feature::getSimilarFeatures(int numFeatures) {
	vector<Feature*> retval(numFeatures, (Feature*)NULL); 
	if (_FEATURE_DEBUG) cout<< "Getting " << numFeatures << " similar features. " << endl; 
	for (int i = 0; i < numFeatures; i++) {
		retval[i] = getFeatureOfType(featureName, patchSize); 
		if (_FEATURE_DEBUG) cout<< "New feature has type " << retval[i]->featureName << " ; Allocating some memory. " << endl; 
		CvMat* newParams = cvCloneMat(parameters); 
		CvMat* change = cvCreateMat(newParams->rows, newParams->cols, CV_64FC1); 
		CvMat* scratch = cvCreateMat(newParams->rows, newParams->cols, CV_64FC1); 
		CvMat* cmpmat = cvCreateMat(newParams->rows, newParams->cols, CV_8U); 
		if (_FEATURE_DEBUG) cout<< "Computing feature ranges. " << endl; 
		cvSub(maxParamVals, minParamVals, scratch); //get range
		do {
			if (_FEATURE_DEBUG) cout<< "Getting random features." << endl; 
			cvRandArr(&rng, change, CV_RAND_NORMAL, 
					  cvRealScalar(0), cvRealScalar(1)); 
			if (_FEATURE_DEBUG) cout<< "Scaling features." << endl; 
			cvMul(change, scratch, change, 0.05); //scale each change
			if (parameterType == discrete)
				fix(change); 
			cvAdd(newParams, change, newParams); 
			if (_FEATURE_DEBUG) cout<< "Checking parameter boundaries" << endl; 
			checkParameterBounds(newParams); 
			
			if (_FEATURE_DEBUG) cout<< "Checking that new params are different from old ones." << endl; 
			cvCmp(parameters, newParams, cmpmat,CV_CMP_NE); 
		} while (!any(cmpmat)); 
		retval[i]->setFeatureParameters(newParams); 
		cvReleaseMat(&newParams); 
		cvReleaseMat(&change); 
		cvReleaseMat(&scratch); 
		cvReleaseMat(&cmpmat); 
	}
	return retval; 
}

void Feature::setRandomParameterValues() {	
	if (_FEATURE_DEBUG) cout<< "Allocating some memory." << endl; 
	CvMat* change = cvCreateMat(parameters->rows, parameters->cols, CV_64FC1); 
	CvMat* scratch = cvCreateMat(parameters->rows, parameters->cols, CV_64FC1); 
	
	if (_FEATURE_DEBUG) cout<< "Getting range." << endl; 
	cvSub(maxParamVals, minParamVals, scratch); //get range
	if (_FEATURE_DEBUG) cout<< "Getting Random Array." << endl; 
	cvRandArr(&rng, change, CV_RAND_UNI, 
			  cvRealScalar(0), cvRealScalar(1)); 
	
	if (_FEATURE_DEBUG) cout<< "Making random values suitable." << endl; 
	cvMul(change, scratch, change); 
	cvAdd(change, minParamVals, change);
	if (parameterType == discrete) {
		for (int i = 0; i < change->height; i++) {
			for (int j = 0; j < change->width; j++) {
				cvSetReal2D(change, i, j, cvRound(cvGetReal2D(change, i, j))); 
			}
		}
	}
	
	if (_FEATURE_DEBUG) cout<< "Setting feature parameters." << endl; 
	setFeatureParameters(change); 
	cvReleaseMat(&change); 
	cvReleaseMat(&scratch); 
}

void Feature::checkParameterBounds(CvMat* params) {
	if (_FEATURE_DEBUG) cout << "Checking bounds for parameter vector size (" << params->width << ", " << params->height << ")" << endl; 
	for (int j = 0; j < params->height; j++) {
		for (int k = 0; k < params->width; k++) {
			double pval = cvGetReal2D(params, j, k); 
			double minval = cvGetReal2D(minParamVals, j, k); 
			double maxval = cvGetReal2D(maxParamVals, j,k);
			while ( pval < minval || pval > maxval) {
				if (_FEATURE_DEBUG) cout<< "Parameter (" << j << ", " << k << ") ; Minval: " << minval << " Maxval: " << maxval << "PVal: " << pval<< "" << endl; 
				if (pval < minval)
					pval = 2*minval - pval; 
				if (pval > maxval)
					pval = 2*maxval - pval; 
			}
			cvSetReal2D(params, j, k, pval); 
		}
	}	
	if (_FEATURE_DEBUG) cout << "Parameters are within proper bounds." <<  endl; 
}

Size Feature::getPatchSize() const {
	return this->patchSize; 
}


void Feature::getFeatureVisualization(Mat &dest){
	Mat dummy = (this->kernel); 
	dest = dummy.clone(); 
}

IplImage* Feature::visualizeFeature() {
	return this->kernel; 
}

string Feature::debugInfo() const{
	stringstream out; 
	string retval; 
	out << this->featureName << endl; 
	out << this->patchSize.width << " " << this->patchSize.height << endl; 
	out << this->parameters->width << " " << this->parameters->height << endl; 
	for (int i = 0; i < this->parameters->height; i++) {
		for (int j = 0; j < this->parameters->width; j++) {
			out << cvGetReal2D(this->parameters, i, j) << " "; 
		}
		out << endl; 
	}
	retval = out.str(); 
	return retval; 
}

int Feature::equals(Feature* other) {
	if (other == NULL) return 0; 
	if (strncmp(featureName, other->featureName,128)) {
		if (parameters->rows != other->parameters->rows)
			return 0; 
		if (parameters->cols != other->parameters->cols)
			return 0; 
		for (int i = 0; i < parameters->rows; i++) 
			for (int j = 0; j < parameters->cols; j++)
				if (cvGetReal2D(parameters, i,j) != cvGetReal2D(other->parameters,i,j))
					return 0; 
		return 1; 
	}
	return 0; 
}

Feature* Feature::copy() const{
	
	Feature* patchFeature = this->getFeatureOfSameTypeAndSize();
	patchFeature->setFeatureParameters(this->parameters); 
	return patchFeature; 
}

Feature* Feature::getFeatureOfSameTypeAndSize() const {
	return Feature::getFeatureOfType(this->featureName, this->patchSize); 
}

Feature* Feature::getFeatureOfType(string featureName, Size patchSize) {
	Feature* retval = NULL; 
	if (strncmp(featureName.c_str(),"Feature", 128)==0) {
		cout << "Can't Create Base-Class \"Feature\": Purely Virtual." << endl;
	} else if (strncmp(featureName.c_str(), "BoxFeature", 128)== 0) {
		retval = new BoxFeature(patchSize); 
	} else if (strncmp(featureName.c_str(), "HaarFeature", 128)== 0) {
		retval = new HaarFeature(patchSize); 
	}
	return retval; 
}


void Feature::evaluateImagePatches(const vector<ImagePatch*> &imagePatches, CvMat*& scalarVals){
	if (scalarVals == NULL || (unsigned int)scalarVals->rows != imagePatches.size() 
		|| scalarVals->cols != 1) {
		cvReleaseMat(&scalarVals); 
		scalarVals = cvCreateMat(imagePatches.size(), 1, CV_64FC1); 
		
		if (_FEATURE_DEBUG) cout << "Created scalarVals with size " << scalarVals->rows << "x" << scalarVals->cols << endl; 
		
		cvSetZero(scalarVals); 
	}
	for (unsigned int i = 0; i < imagePatches.size(); i++) {
		cvSetReal2D(scalarVals, i, 0, evaluateImagePatch(imagePatches[i])); 
	}
}

void Feature::evaluateImagePatches(const vector<ImagePatch*> &imagePatches, Mat &scalarVals){
	scalarVals.create(imagePatches.size(), 1, CV_64FC1); 
	scalarVals = 0.; 
	
	for (unsigned int i = 0; i < imagePatches.size(); i++) {
		scalarVals.at<double>( i, 0) = evaluateImagePatch(imagePatches[i]); 
	}
}

void Feature::evaluateImagePatches(const vector<ImagePatch> &imagePatches, Mat &scalarVals){
	scalarVals.create(imagePatches.size(), 1, CV_64FC1); 
	scalarVals = 0.; 
	
	for (unsigned int i = 0; i < imagePatches.size(); i++) {
		scalarVals.at<double>( i, 0) = evaluateImagePatch(imagePatches[i]); 
	}
}

void Feature::evaluateImagePatches(const vector<ImagePatch*> &imagePatches, vector<double>& scalarVals){
	if (scalarVals.size() != imagePatches.size())
		scalarVals.resize(imagePatches.size()); 
	
	for (unsigned int i = 0; i < imagePatches.size(); i++) {
		scalarVals[i] = evaluateImagePatch(imagePatches[i]); 
	}
}

void Feature::evaluateImagePatches(const vector<ImagePatch> &imagePatches, vector<double>& scalarVals){
	if (scalarVals.size() != imagePatches.size())
		scalarVals.resize(imagePatches.size()); 
	
	for (unsigned int i = 0; i < imagePatches.size(); i++) {
		scalarVals[i] = evaluateImagePatch(imagePatches[i]); 
	}
}

ostream& operator<< (ostream& ofs, Feature* model) {
	if (_FEATURE_DEBUG) cout << "Adding Feature to Stream" << endl; 
	ofs << model->featureName << endl; 
	ofs << model->patchSize.width << " " << model->patchSize.height << endl; 
	ofs << model->parameters->width << " " << model->parameters->height << endl; 
	for (int i = 0; i < model->parameters->height; i++) {
		for (int j = 0; j < model->parameters->width; j++) {
			ofs << cvGetReal2D(model->parameters, i, j) << " "; 
		}
		ofs << endl; 
	}
	if (_FEATURE_DEBUG) cout << "Finished outputting feature" << endl; 
	return ofs; 
}

istream& operator>> (istream& ifs, Feature *&model) {
	
	if (_FEATURE_DEBUG) cout << "Reading In Feature" << endl ;
	char name[128]; 
	CvSize patchSize; 
	ifs >> name; 	
	ifs >> patchSize.width >> patchSize.height; 
	int paramheight = 0; 
	int paramwidth = 0; 
	ifs >> paramwidth >> paramheight; 
	
	CvMat* params = cvCreateMat(paramheight, paramwidth, CV_64FC1); 
	
	
	if (_FEATURE_DEBUG) cout << "Created param vec and ready to fill it with parameters." << endl ;
	double val = 0; 
	for (int i = 0; i < params->height; i++) {
		for (int j = 0; j < params->width; j++) {
			ifs >> val; 
			if (_FEATURE_DEBUG) cout << "Read in parameter " << j << " as value " << val << endl; 
			cvSetReal2D(params, i, j, val) ; 
		}
	}
	
	if (_FEATURE_DEBUG) cout << "Setting feature to a new feature of type " << name << endl; 
	model = Feature::getFeatureOfType(name, patchSize); 
	
	if (_FEATURE_DEBUG) cout << "Setting new feature's parameters." << endl; 
	model->setFeatureParameters(params); 
	
	if (_FEATURE_DEBUG) cout << "Done creating new feature." << endl; 
	cvReleaseMat(&params) ; 
	return ifs; 
}
