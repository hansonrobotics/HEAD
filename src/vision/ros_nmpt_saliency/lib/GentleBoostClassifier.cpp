/*
 *  GentleBoostClassifier.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/26/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "GentleBoostClassifier.h"
#include "DebugGlobals.h" 
#include <opencv2/core/core.hpp>

using namespace cv; 
using namespace std; 

GentleBoostClassifier::GentleBoostClassifier() : trainingPatches(1, (ImagePatch*)NULL), 
trainingFeatureOutputs(1,(CvMat*)NULL), testingPatches(1,(ImagePatch*)NULL),
testingFeatureOutputs(1,(CvMat*)NULL), features(1,(FeatureRegressor*)NULL){
		
	/*Set matrices to dummy values so that they will be filled in later*/
	trainingLabels = NULL; ; 
	trainingFeatureSum = cvCreateMat(1, 1, CV_64FC1); 
	trainingPosteriors = cvCreateMat(1, 1, CV_64FC1); 
	trainingPredictions = cvCreateMat(1, 1, CV_64FC1);
	trainingWeights = cvCreateMat(1, 1, CV_64FC1);
	trainingPerformance = 0; 
	numTrain = 0; 
	
	testingLabels = NULL; 
	testingFeatureSum = cvCreateMat(1, 1, CV_64FC1); 
	testingPosteriors = cvCreateMat(1, 1, CV_64FC1); 
	testingPredictions = cvCreateMat(1, 1, CV_64FC1); 
	testingPerformance = 0; 
	numTest = 0; 
	
	numFeatures= 0; 
	features.clear(); 
	basePatchSize = cvSize(-1,-1); 
}

void GentleBoostClassifier::addToStream(ostream& out) {
	
	if (_GENTLEBOOST_DEBUG) cout << "Adding GB Object To Stream" << endl; 
	out << numFeatures << endl; 
	//cout << "A" << endl; 
	for (int i = 0; i < numFeatures; i++) {
		//cout << "B" << endl; 
		if (_GENTLEBOOST_DEBUG) cout << "Adding Feature "<< i << endl; 
		out << features[i]; 
	}
	//cout << "C" << endl; 
}
void GentleBoostClassifier::readFromStream(istream& in) {
	in >> numFeatures;
	features.clear(); 
	FeatureRegressor* reg; 
	for (int i = 0; i < numFeatures; i++) {
		if (_GENTLEBOOST_DEBUG) cout << "Reading in Regressor " << i << endl; 
		in >> reg; 
		features.push_back(reg); 
	}
	
	if ((int) features.size() > 0 ) {
		basePatchSize = features[0]->getFeaturePatchSize(); 
	}
	
}


cv::Size GentleBoostClassifier::getBasePatchSize() {
	return basePatchSize; 
}

GentleBoostClassifier::~GentleBoostClassifier(){
	for (unsigned int i = 0; i < trainingPatches.size(); i++) 
		if (trainingPatches[i] != NULL)
			delete(trainingPatches[i]); 
	for (unsigned int i = 0; i < testingPatches.size(); i++) 
		if (testingPatches[i] != NULL)
			delete(testingPatches[i]); 
	for (unsigned int i = 0; i < features.size(); i++) 
		if (features[i] != NULL)
			delete(features[i]); 
	for (unsigned int i = 0; i < trainingFeatureOutputs.size(); i++)
		cvReleaseMat(&trainingFeatureOutputs[i]); 
	for (unsigned int i = 0; i < testingFeatureOutputs.size(); i++)
		cvReleaseMat(&testingFeatureOutputs[i]); 
	
	cvReleaseMat(&trainingLabels); 
	cvReleaseMat(&trainingFeatureSum); 
	cvReleaseMat(&trainingPosteriors); 
	cvReleaseMat(&trainingPredictions); 
	cvReleaseMat(&trainingWeights); 
	
	cvReleaseMat(&testingLabels); 
	cvReleaseMat(&testingFeatureSum); 
	cvReleaseMat(&testingPosteriors); 
	cvReleaseMat(&testingPredictions); 
}

void GentleBoostClassifier::searchPatches(const vector<ImagePatch*>& patches,
										  const CvMat* labels,
										  CvMat*& featureSum,
										  CvMat*& posterior,
										  CvMat*& predictions,
										  vector<CvMat*>& featureOutputs,
										  double& perf,
										  CvMat*& weights){
	
	int numPatches = patches.size(); 
	int releaseFeatureSum = featureSum==NULL; 
	
	if (featureSum == NULL || featureSum->rows != numPatches || featureSum->cols != 1) {
		cvReleaseMat(&featureSum); 
		featureSum = cvCreateMat(numPatches, 1, CV_64FC1); 
	}
	if (posterior == NULL || (posterior->rows != numPatches || posterior->cols != 1)) {
		cvReleaseMat(&posterior); 
		posterior = cvCreateMat(numPatches, 1, CV_64FC1); 
	}
	if (predictions == NULL || (predictions->rows != numPatches || predictions->cols != 1)) {
		cvReleaseMat(&predictions); 
		predictions = cvCreateMat(numPatches, 1, CV_64FC1); 
	}
	if (weights != NULL && (weights->rows != numPatches || weights->cols != 1)) {
		cvReleaseMat(&weights); 
		weights = cvCreateMat(numPatches, 1, CV_64FC1); 
	}
	
	
	for (unsigned int i = 0; i < featureOutputs.size(); i++)  cvReleaseMat(&featureOutputs[i]); 
	featureOutputs.clear(); 
	featureOutputs.resize(numFeatures, (CvMat*)NULL); 
	
	cvSetZero(featureSum); 
	if (weights != NULL) cvSet(weights, cvRealScalar(1.0/numPatches)); 
	
	
	CvMat* outputs = NULL; 
	for (int i = 0; i < numFeatures; i++) {
		
		Mat out; 
		CvMat outmat; 
		features[i]->predict(patches, out); 
		outmat = out; 
		outputs = &outmat; 
		
		//features[i]->predict(patches, outputs); 
		featureOutputs[i] = cvCloneMat(outputs); 
		
		cvAdd(featureSum, outputs, featureSum); 
		if (weights != NULL && labels != NULL) {
			//weights = weights.*exp(-featureOutputs(:,i).*labels);
			//weights = weights./sum(weights);
			cvMul(outputs, labels, outputs); 
			cvSubRS(outputs, cvRealScalar(-1), outputs); 
			cvExp(outputs, outputs); 
			cvMul(weights, outputs, weights); 
			cvNormalize(weights, weights, 1, 0, CV_L1); 
		}
	}
	
	//cvReleaseMat(&outputs); 
	
	if (posterior != NULL) {
		//posterior(survived)= 1./(1+exp(-2.*featureSum(survived)));
		cvScale(featureSum, posterior, -2); 
		cvExp(posterior, posterior); 
		cvAddS(posterior, cvRealScalar(1), posterior); 
		cvDiv(NULL,posterior, posterior); 
	}
	
	perf = 0; 
	if (predictions != NULL) {
		for (int i = 0; i < featureSum->height; i++) {
			if (cvGetReal2D(featureSum, i, 0) > .5) cvSetReal2D(predictions,i,0,1); 
			else cvSetReal2D(predictions,i,0,-1); 
		}
	}
	
	if (labels != NULL) {
		perf = 0; 
		for (int i = 0; i < featureSum->height; i++)
			if (cvGetReal2D(featureSum, i, 0) > .5 && cvGetReal2D(labels, i, 0) == 1)
				perf = perf+1; 
		perf = perf / numPatches; 
	}
	
	if (releaseFeatureSum) cvReleaseMat(&featureSum); 
	
}

void GentleBoostClassifier::setTrainingSet(const vector<ImagePatch*>& trainingPatches,
										   const CvMat* trainingLabels){
	if (&this->trainingPatches!= &trainingPatches) {
		this->trainingPatches.clear(); 
		this->trainingPatches.resize(trainingPatches.size(), (ImagePatch*)NULL); 
		for (unsigned int i = 0; i < trainingPatches.size(); i++)
			this->trainingPatches[i] = trainingPatches[i]; 
	}
	if (this->trainingLabels != trainingLabels) {
		cvReleaseMat(&this->trainingLabels); 
		this->trainingLabels = cvCloneMat(trainingLabels); 
	}
	searchPatches(this->trainingPatches,
				  this->trainingLabels,
				  trainingFeatureSum,
				  trainingPosteriors,
				  trainingPredictions,
				  trainingFeatureOutputs,
				  trainingPerformance,
				  trainingWeights); 
} 

void GentleBoostClassifier::setTestingSet(const vector<ImagePatch*>& testingPatches,
										  const CvMat* testingLabels){
	if (&this->testingPatches != &testingPatches) {
		this->testingPatches.clear(); 
		this->testingPatches.resize(testingPatches.size(), (ImagePatch*)NULL); 
		for (unsigned int i = 0; i < testingPatches.size(); i++)
			this->testingPatches[i] = testingPatches[i]; 
	}
	if (this->testingLabels != testingLabels) {
		cvReleaseMat(&this->testingLabels); 
		this->testingLabels = cvCloneMat(testingLabels); 
	}
	
	CvMat* w = NULL; 
	searchPatches(this->testingPatches,
				  this->testingLabels,
				  testingFeatureSum,
				  testingPosteriors,
				  testingPredictions,
				  testingFeatureOutputs,
				  testingPerformance,
				  w); 
} 

void GentleBoostClassifier::addFeature(Feature* nextFeature){
	if (trainingLabels == NULL || (unsigned int)trainingLabels->rows != trainingPatches.size()) {
		cout << "Warning: Must set gentle boost training patches and labels before adding features." << endl;
		return; 
	}
	
	if ((int) features.size() > 0 ) {
		if (nextFeature->getPatchSize().width != basePatchSize.width || 
			nextFeature->getPatchSize().height != basePatchSize.height) {
		cout << "Warning: All features added to classifier must have the same size." << endl;
		return; 
		} 
	}
	else {
		basePatchSize = nextFeature->getPatchSize(); 
	}
	
	
	CvMat* output = NULL;
	
	//cout << "Training RBF" << endl; 
	FeatureRegressor* reg = new FeatureRegressor(nextFeature); 
	
	Mat newTrainingLabels, newTrainingWeights; 
	newTrainingLabels= trainingLabels; 
	newTrainingWeights = trainingWeights; 
	
	reg->train(numBins, trainingPatches, newTrainingLabels, newTrainingWeights); 
	
	Mat out; 
	CvMat outmat; 
	reg->predict(trainingPatches, out); 
	outmat = out; 
	output = &outmat; 
	
	//reg->predict(trainingPatches, output); 
	
	
	//cout << "Adding regressor to list of features" << endl; 

	features.push_back(reg); 
	numFeatures++; 
	
	//cout << "New Number of Features is " << numFeatures << "; features.size() is " << features.size() << endl; 
	
	//cout << "Adding predictions to trainingFeatureOutputs" << endl; 
	trainingFeatureOutputs.push_back(cvCloneMat(output)); 
	
	//cout << "Updating trainingFeatureSum" << endl; 
	cvAdd(trainingFeatureSum, output, trainingFeatureSum); 
	
	//cout << "Updating Training Weights" << endl; 
	cvMul(output, trainingLabels, output); 
	cvSubRS(output, cvRealScalar(-1), output); 
	cvExp(output, output); 
	cvMul(trainingWeights, output, trainingWeights); 
	cvNormalize(trainingWeights, trainingWeights, 1, 0, CV_L1); 
	
	//cout << "Updating training posteriors" << endl; 
	cvScale(trainingFeatureSum, trainingPosteriors, -2); 
	cvExp(trainingPosteriors, trainingPosteriors); 
	cvAddS(trainingPosteriors, cvRealScalar(1), trainingPosteriors); 
	cvDiv(NULL, trainingPosteriors, trainingPosteriors); 
	
	//cout << "Updating training predictions" << endl; 
	for (int i = 0; i < trainingFeatureSum->height; i++) {
		if (cvGetReal2D(trainingFeatureSum, i, 0) > .5) cvSetReal2D(trainingPredictions,i,0,1); 
		else cvSetReal2D(trainingPredictions,i,0,-1); 
	}
	
	
	//cout << "Updating training performance." << endl; 
	trainingPerformance = 0; 
	for (int i = 0; i < trainingFeatureSum->height; i++)
		if (cvGetReal2D(trainingFeatureSum, i, 0) > .5 && cvGetReal2D(trainingLabels, i, 0) == 1)
			trainingPerformance = trainingPerformance+1; 
	trainingPerformance = trainingPerformance / trainingFeatureSum->height; 
	
	//cout << "New Performance: " << trainingPerformance << endl; 
	
	//cout << "Done with training things." << endl; 
	
	if (testingLabels != NULL && (unsigned int) testingLabels->rows != testingPatches.size()) {
		
		//cout << "Starting testing things." << endl; 
		
		Mat out; 
		CvMat outmat; 
		reg->predict(testingPatches, out); 
		outmat = out; 
		output = &outmat; 
		//reg->predict(testingPatches, output); 
		testingFeatureOutputs.push_back(cvCloneMat(output)); 
		cvAdd(testingFeatureSum, output, testingFeatureSum); 
		
		cvScale(testingFeatureSum, testingPosteriors, -2); 
		cvExp(testingPosteriors, testingPosteriors); 
		cvAddS(testingPosteriors, cvRealScalar(1), testingPosteriors); 
		cvDiv(NULL, testingPosteriors, testingPosteriors); 
		
		for (int i = 0; i < testingFeatureSum->height; i++) {
			if (cvGetReal2D(testingFeatureSum, i, 0) > .5) cvSetReal2D(testingPredictions,i,0,1); 
			else cvSetReal2D(testingPredictions,i,0,-1); 
		}
		
		testingPerformance = 0; 
		for (int i = 0; i < testingFeatureSum->height; i++)
			if (cvGetReal2D(testingFeatureSum, i, 0) > .5 && cvGetReal2D(testingLabels, i, 0) == 1)
				testingPerformance = testingPerformance+1; 
		testingPerformance = testingPerformance / testingFeatureSum->height; 
	}
	
} 

int GentleBoostClassifier::getNumFeaturesUsed() {
	return numFeatures; 
}

int GentleBoostClassifier::getNumFeaturesTotal() {
	return (int)features.size(); 
}

void GentleBoostClassifier::setNumFeaturesUsed(int num) {
	if (num < (int)features.size() && num >= 0) 
		numFeatures = num; 
	else 
		numFeatures = features.size();
}

FeatureRegressor* GentleBoostClassifier::getFeatureAndTuningCurveNumber(int num) {
	if (num >= 0 && num < (int)features.size())
		return features[num]; 
	return NULL; 
}

void GentleBoostClassifier::getPerformanceMeasures(Feature* candidate, double& chiSq){
	//Get Performance has three modes depending on the candidate feature: 
	//--If the feature is NULL, calculates the current chi-squared error
	//--If the feature is currently in the classifier, calculates the error if
	//  we remove the feature
	//--Otherwise, calculates the error if we were to add this feature.
	
	if (trainingLabels == NULL) {
		cout << "Warning: Must set training patches & labels before calling getPerformanceMeasures" << endl; 
		return; 
	}
	
	
	CvMat* output = NULL; 
	CvMat* den = NULL; 
	CvMat* num = NULL; 
	
	if (candidate == NULL) { //Case: Current performance
		//cout << "Testing current performance." << endl; 
		output = cvCloneMat(trainingFeatureSum); 
	} else {
		int ind = -1; 
		//cout << "Checking if duplicate feature." << endl; 
		for (int i = 0; i < numFeatures; i++) {
			if (candidate == features[i]->getFeature()) {
				ind = i; 
				break; 
			}
		}
		
		if (ind > -1) { //Case: Performance without this feature.
			//cout << "Found duplicate feature at " << ind << "." << endl; 
			output = cvCloneMat(trainingFeatureSum); 
			cvSub(output, trainingFeatureOutputs[ind], output); 
		} else {
			//cout << "Training feature to see how it would do." << endl; 
			FeatureRegressor* reg = new FeatureRegressor(candidate); 
			
			Mat newTrainingLabels, newTrainingWeights; 
			newTrainingLabels= trainingLabels; 
			newTrainingWeights = trainingWeights; 
			
			reg->train(numBins, trainingPatches, newTrainingLabels, newTrainingWeights); 
			
			Mat out; 
			CvMat outmat; 
			reg->predict(trainingPatches, out); 
			outmat = out; 
			output = &outmat; 
			
			cvAdd(trainingFeatureSum, output, output); 
			
			delete(reg); 
		}
	}
	 
	cvScale(output, output, -2); 
	cvExp(output, output); 
	cvAddS(output, cvRealScalar(1), output); 
	cvDiv(NULL,output, output); 
	
	den = cvCloneMat(output); 
	cvSubRS(den, cvRealScalar(1), den); 
	cvMul(den, output, den); 
	cvPow(den, den, -.5); 
	
	num = cvCloneMat(trainingLabels); 
	cvScale(num, num, .5, .5); 
	cvAbsDiff(num, output, num); 
	cvMul(num, den, num); 
	CvScalar sum = cvSum(num); 
	chiSq = sum.val[0]; 
	
	cvReleaseMat(&num); 
	cvReleaseMat(&den); 
	cvReleaseMat(&output); 
} 

IplImage* GentleBoostClassifier::getProbabilityMap(PatchList* patches) {
	for (int i = 0; i < numFeatures; i++) {
		features[i]->predictPatchList(patches); 
		patches->accumulateAndRemovePatchesBelowThreshold(-INFINITY); 
	}
	cv::Size s = patches->getImageSizeAtScale();
	IplImage* probMap = cvCreateImage(s, IPL_DEPTH_64F, 1);
	Mat m; 
	patches->getAccumImage(m); 
	IplImage accumulatorImage = m; 
	
	cvSetImageROI(&accumulatorImage, cvRect(0,0,s.width,s.height)); 
	cvCopy(&accumulatorImage, probMap); 
	cvResetImageROI(probMap); 
	cvScale(probMap, probMap, -2); 
	cvExp(probMap, probMap); 
	cvAddS(probMap, cvRealScalar(1), probMap); 
	cvDiv(NULL, probMap, probMap); 
	return probMap; 
}

ostream& operator<< (ostream& ofs, GentleBoostClassifier* model) {
	model->addToStream(ofs); 
	return ofs; 
}
istream& operator>> (istream& ifs, GentleBoostClassifier*& model) {
	model->readFromStream(ifs); 
	return ifs; 
}
