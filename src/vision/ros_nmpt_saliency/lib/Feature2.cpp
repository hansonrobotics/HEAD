/*
 *  Feature2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/22/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "Feature2.h"
#include "BoxFeature2.h"
#include "HaarFeature2.h"
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



Feature2::Feature2(Size expectedPatchSize) {	
	patchSize = expectedPatchSize; 
	prefersIntegral = 0; 
	featureName = "Feature"; 
	//rng = cvRNG(BlockTimer::getCurrentTimeInMicroseconds());
}

Feature2::~Feature2() {
}

vector<Feature2*> Feature2::getSimilarFeatures(int numFeatures) const {
	vector<Feature2*> retval(numFeatures, (Feature2*)NULL); 
	if (_FEATURE_DEBUG) cout<< "Getting " << numFeatures << " similar features. " << endl; 
	for (int i = 0; i < numFeatures; i++) {
		retval[i] = getFeatureOfType(featureName, patchSize); 
		if (_FEATURE_DEBUG) cout<< "New feature has type " << retval[i]->featureName << " ; Allocating some memory. " << endl; 
		Mat newParams = parameters.clone(); 
		
		Mat stdev = 0.05*(maxParamVals-minParamVals); 
		Mat change(newParams.size(), newParams.type()); 
		
		do {
			if (_FEATURE_DEBUG) cout<< "Getting random features." << endl; 
			randn(change, 0., 1.); 
			change = change.mul(stdev); 
			if (_FEATURE_DEBUG) cout<< "Scaling features." << endl; 
			newParams += change; 
			
			if (_FEATURE_DEBUG) cout<< "Checking parameter boundaries" << endl; 
			checkParameterBounds(newParams); 
			
			if (_FEATURE_DEBUG) cout<< "Checking that new params are different from old ones." << endl; 
		} while (!any<uint8_t>(parameters != newParams)); 
		
		retval[i]->setFeatureParameters(newParams); 
	}
	return retval; 
}

void Feature2::setRandomParameterValues() {	
	if (_FEATURE_DEBUG) cout<< "Getting range." << endl; 
	Mat range = maxParamVals-minParamVals; 
	
	if (_FEATURE_DEBUG) cout<< "Getting Random Array." << endl; 
	Mat change(parameters.size(), parameters.type());
	randu(change, 0., 1.); 
	
	if (_FEATURE_DEBUG) cout<< "Making random values suitable." << endl; 
	change = change.mul(range);
	change += minParamVals; 
	
	if (_FEATURE_DEBUG) cout<< "Setting feature parameters." << endl; 
	setFeatureParameters(change); 
}

void Feature2::checkParameterBounds(Mat &params) const {

	if (_FEATURE_DEBUG) cout << "Checking bounds for parameter vector size (" << params.rows << ", " << params.cols << ")" << endl; 
	
	Mat cmp; 
	bitwise_or( params < minParamVals, params > maxParamVals, cmp); 
	while(any<uint8_t>(cmp)) {
		if (_FEATURE_DEBUG) {
			cout << "minParamVals: " ; 
			NMPTUtils::printMat(minParamVals); 
			cout << "paramVals   : " ; 
			NMPTUtils::printMat(params); 
			cout << "maxParamVals: " ; 
			NMPTUtils::printMat(maxParamVals); 
			cout << "cmp         : " ; 
			NMPTUtils::printMat(cmp); 
		}
		Mat addval; 
		//addval = 2. * minParamVals - params; 
		subtract(2. * minParamVals, params, params, params < minParamVals); 
		//addval = 2. * maxParamVals - params; 
		subtract(2. * maxParamVals, params, params, params > maxParamVals); 
		bitwise_or( params < minParamVals, params > maxParamVals, cmp); 
	} 
	
	if (_FEATURE_DEBUG) cout << "Parameters are within proper bounds." <<  endl; 
}

Size Feature2::getPatchSize() const {
	return patchSize; 
} 

void Feature2::getFeatureVisualization(Mat &dest) const{
	dest = kernel.clone(); 
}

string Feature2::debugInfo() const{
	stringstream out; 
	string retval; 
	out << this->featureName << endl; 
	out << this->patchSize.width << " " << this->patchSize.height << endl; 
	out << this->parameters.rows << " " << this->parameters.cols << endl; 
	for (int i = 0; i < this->parameters.rows; i++) {
		for (int j = 0; j < this->parameters.cols; j++) {
			out << this->parameters.at<double>(i, j) << " "; 
		}
		out << endl; 
	}
	retval = out.str(); 
	return retval; 
}

int Feature2::equals(Feature2* other) {
	if (other == NULL) return 0; 
	if (featureName.compare(other->featureName) ){
		if (parameters.rows != other->parameters.rows) return 0; 
		if (parameters.cols != other->parameters.cols) return 0; 
		return !any<uint8_t>(parameters != other->parameters); 
	}
	return 0; 
}
		
Feature2* Feature2::copy() const{
	Feature2* patchFeature = getFeatureOfSameTypeAndSize();
	patchFeature->setFeatureParameters(parameters); 
	return patchFeature; 
}

Feature2* Feature2::getFeatureOfSameTypeAndSize() const {
	return Feature2::getFeatureOfType(this->featureName, this->patchSize); 
}

Feature2* Feature2::getFeatureOfType(string featureName, Size patchSize) {
	Feature2* retval = NULL; 
	if (featureName.compare("Feature")==0) {
		cout << "Can't Create Base-Class \"Feature\": Purely Virtual." << endl;
	} else if (featureName.compare("BoxFeature")== 0) {
		retval = new BoxFeature2(patchSize); 
	} else if (featureName.compare("HaarFeature")== 0) {
		retval = new HaarFeature2(patchSize); 
	}
	return retval; 
}

void Feature2::evaluateImagePatches(const vector<ImagePatch2> &imagePatches, Mat &scalarVals) const {
	scalarVals.create(imagePatches.size(), 1, CV_64FC1); 
	scalarVals = 0.; 
	for (unsigned int i = 0; i < imagePatches.size(); i++) {
		scalarVals.at<double>( i, 0) = evaluateImagePatch(imagePatches[i]); 
	}
}


ostream& operator<< (ostream& ofs, const Feature2* model) {
	if (_FEATURE_DEBUG) cout << "Adding Feature to Stream" << endl; 
	ofs << model->featureName << endl; 
	ofs << model->patchSize.width << " " << model->patchSize.height << endl; 
	ofs << model->parameters.cols << " " << model->parameters.rows << endl; 
	for (int i = 0; i < model->parameters.rows; i++) {
		for (int j = 0; j < model->parameters.cols; j++) {
			ofs << model->parameters.at<double>(i, j) << " "; 
		}
		ofs << endl; 
	}
	if (_FEATURE_DEBUG) cout << "Finished outputting feature" << endl; 
	return ofs; 
}
 
istream& operator>> (istream& ifs, Feature2 *&model) {
	
	if (_FEATURE_DEBUG) cout << "Reading In Feature" << endl ;
	string name; 
	Size patchSize; 
	ifs >> name; 	
	ifs >> patchSize.width >> patchSize.height; 
	int paramheight = 0; 
	int paramwidth = 0; 
	ifs >> paramwidth >> paramheight; 
	
	Mat params(paramheight, paramwidth, CV_64F); 
	
	
	if (_FEATURE_DEBUG) cout << "Created param vec and ready to fill it with parameters." << endl ;
	double val = 0; 
	for (int i = 0; i < params.rows; i++) {
		for (int j = 0; j < params.cols; j++) {
			ifs >> val; 
			if (_FEATURE_DEBUG) cout << "Read in parameter " << j << " as value " << val << endl; 
			params.at<double>(i, j)= val; 
		}
	}
	
	if (_FEATURE_DEBUG) cout << "Setting feature to a new feature of type " << name << endl; 
	model = Feature2::getFeatureOfType(name, patchSize); 
	
	if (_FEATURE_DEBUG) cout << "Setting new feature's parameters." << endl; 
	model->setFeatureParameters(params); 
	
	if (_FEATURE_DEBUG) cout << "Done creating new feature." << endl; 
	return ifs; 
}

Feature2* Feature2::readFromFile(const cv::FileNode &fs) {
	Feature2* rhs; 
	if (_FEATURE_DEBUG) std::cout << "Reading In Feature" << std::endl ;
	std::string name; 
	fs["name"] >> name; 
	cv::Size size; 
	fs["size_w"] >> size.width;
	fs["size_h"] >> size.height; 
	cv::Mat parameters; 
	fs["parameters"] >> parameters; 
	
	rhs = Feature2::getFeatureOfType(name, size); 
	rhs->setFeatureParameters(parameters); 
	if (_FEATURE_DEBUG) std::cout << "Done creating new feature." << std::endl; 
	return rhs; 
}


void Feature2::writeToFile(cv::FileStorage & fs, const string &varname) const {
	if (_FEATURE_DEBUG) std::cout << "Adding Feature to Storage" << std::endl; 
	fs << varname << "{" << "name" << featureName << "size_w" << patchSize.width
	<< "size_h" << patchSize.height << "parameters" << parameters << "}"; 
	if (_FEATURE_DEBUG) std::cout << "Finished outputting feature" << std::endl; 
}
