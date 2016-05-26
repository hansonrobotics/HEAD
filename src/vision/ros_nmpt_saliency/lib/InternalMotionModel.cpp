/*
 *  InternalMotionModel.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 2/10/10.
 *  Copyright 2010 Apple Inc. All rights reserved.
 *
 */

#include "InternalMotionModel.h"
#include "MatrixKalmanFilter.h"
#include "ImageKalmanFilter.h"
#include "NMPTUtils.h"

using namespace NMPTUtils; 
using namespace std; 
using namespace cv; 

#define _IMM_NUMFRAMES 30

InternalMotionModel& InternalMotionModel::operator=(const InternalMotionModel& copy) {
	copyFrom(copy); 
	return *this; 
}

void InternalMotionModel::copyFrom(const InternalMotionModel& copy) {
	lambda = copy.lambda; 
	eta = copy.eta; 
	alphaDynamics = copy.alphaDynamics; 
	alpha = &(alphaDynamics[alphaDynamics.size()-1]); 
	nuDynamics = copy.nuDynamics; 
	nu = &(nuDynamics[nuDynamics.size()-1]); 
	timeStamps = copy.timeStamps;
	inverseMotionModel = copy.inverseMotionModel; 
	useMotorUncertaintyInLearning = copy.useMotorUncertaintyInLearning; 
	numActuators = copy.numActuators; 
	numDynamicsModels = copy.numDynamicsModels; 
	numMotionFeatures = copy.numMotionFeatures; 
	invCMat = copy.invCMat.clone(); 
	cMat = copy.cMat.clone(); 
	absCMat = copy.absCMat.clone(); 
	features = copy.features.clone(); 
	invFeatures = copy.invFeatures.clone(); 
	matNx1a = copy.matNx1a.clone(); 
	matNx1b = copy.matNx1b.clone(); 
	matNxNa = copy.matNxNa.clone(); 
	matNxNb = copy.matNxNb.clone(); 
}

InternalMotionModel::InternalMotionModel(const InternalMotionModel &copy){
	copyFrom(copy); 
}

void InternalMotionModel::init(int numActuators, Size obsSize, size_t numDynamicsModels) {
	if (_IMM_DEBUG) cout << "IMM Constructor" << endl; 
	
	lambda =  ImageKalmanFilter(obsSize); 
	eta = ImageKalmanFilter(obsSize); 
	
	
	setMuLambdaPrior(); 
	setSigmaSquaredLambdaPrior(); 
	setQSquaredLambda(); 
	setRSquaredLambda(); 
	
	setMuEtaPrior(); 
	setSigmaSquaredEtaPrior(); 
	setQSquaredEta(); 
	setRSquaredEta(); 
	
	this->numDynamicsModels = numDynamicsModels; 
	setNumActuators(numActuators); 
	
	setUseRetinalCoordinates(); 
	setUseMotorUncertaintyInLearning(); 
	setUseSensorUncertaintyInLearning(); 
}

InternalMotionModel::InternalMotionModel(){
	init(); 
}
InternalMotionModel::InternalMotionModel(int numActuators, Size obsSize, size_t numDynamicsModels){
	init(numActuators, obsSize, numDynamicsModels); 
}

InternalMotionModel::~InternalMotionModel() {
	if (_IMM_DEBUG) cout << "IMM Destructor" << endl; 

}

void InternalMotionModel::setUseLearnedInverseModel(int yesorno) {
	if (_IMM_DEBUG) cout << "Setting useLearnedInverseModel to " << yesorno << endl; 
	useLearnedInverseModel = yesorno; 
}

void InternalMotionModel::setUseRetinalCoordinates(int yesorno) {	
	if (_IMM_DEBUG) cout << "Setting useRetinalCoordinates to " << yesorno << endl; 

	lambda.setUseRetinalCoordinates(yesorno); 
	eta.setUseRetinalCoordinates(yesorno); 
}


void InternalMotionModel::setUseMotorUncertaintyInLearning(int yesorno) {
	if (_IMM_DEBUG) cout << "Setting useMotorUncertaintyInLearning to " << yesorno << endl; 

	useMotorUncertaintyInLearning = yesorno; 
}


void InternalMotionModel::setUseSensorUncertaintyInLearning(int yesorno) {
	if (_IMM_DEBUG) cout << "Settign useSensorUncertaintyInLearning to " << yesorno << endl; 

	if (yesorno) {
		 lambda.setRSquared(&(eta.mu)); 
	} else {
		lambda.setRSquared((IplImage**)NULL); 
	}
}

void InternalMotionModel::setNumDynamicsModels(size_t num) {
	if (_IMM_DEBUG) cout << "Setting numDynamicsModels to " << num << endl; 
	int mn = numMotionFeatures*numTransformVals;
	
	alphaDynamics.clear(); 
	nuDynamics.clear(); 
	timeStamps.clear(); 
	
	for (size_t i = 0; i < num; i++) {
		
		alphaDynamics.push_back( MatrixKalmanFilter(mn, numTransformVals)); 
		nuDynamics.push_back(MatrixKalmanFilter(mn, numTransformVals));  
		
		timeStamps.push_back( 0); 
	}
	
	alpha = &(alphaDynamics[getNumDynamicsModels()-1]); 
	nu = &(alphaDynamics[getNumDynamicsModels()-1]); 
	
	setMuAlphaPrior();
	setRAlpha();
	setQAlpha();
	setSigmaAlphaPrior(); 
	
	setMuNuPrior();
	setRNu();
	setQNu();
	setSigmaNuPrior(); 
	
}

void InternalMotionModel::setNumActuators(int num) {
	if (_IMM_DEBUG) cout << "Setting numActuator to " << num << endl; 
	numActuators = num; 
	numMotionFeatures = numActuators; //numActuators+1; //x, y, x^2, y^2, 1
	numTransformVals = lambda.getNumTransformVals();  //tx, ty
	numInvMotionFeatures = numTransformVals;//numTransformVals+1; 
	
	//if (inverseMotionModel != NULL) delete(inverseMotionModel); 
	
	
		
	int mn = numMotionFeatures*numTransformVals;
	int imn = numInvMotionFeatures*numActuators; 
	
	
	cMat.create(numTransformVals, mn, CV_64F);  
	absCMat.create(numTransformVals, mn, CV_64F);  
	features.create(numMotionFeatures, 1, CV_64F);  
	invCMat.create(numActuators, imn, CV_64F); 
	invFeatures.create(numInvMotionFeatures, 1, CV_64F); 
	
	matNx1a.create(numTransformVals, 1, CV_64F); 
	matNx1b.create(numTransformVals, 1, CV_64F); 
	matNxNa.create(numTransformVals, numTransformVals, CV_64F); 
	matNxNb.create(numTransformVals, numTransformVals, CV_64F); 
	
	if (_IMM_DEBUG) cout << "Creating inverse model and setting its parameters. It has state size " << numTransformVals << "x" << numActuators << endl; 
	inverseMotionModel = MatrixKalmanFilter(imn, numActuators); 
	inverseMotionModel.setMuPrior(0.0); 
	inverseMotionModel.setSigmaPrior( 250000.0);
	inverseMotionModel.setR(25.0);
	inverseMotionModel.setQ(400); 
	
	setNumDynamicsModels(getNumDynamicsModels()); 
	/*
	for (size_t i = 0; i < getNumDynamicsModels(); i++) {
				
		if (alphaDynamics[i] != NULL) {
			delete(alphaDynamics[i]);
		}
		if (nuDynamics[i] != NULL) {
			delete(nuDynamics[i]); 
		}
		
		alphaDynamics[i] = new KalmanFilter(mn, numTransformVals); 
		nuDynamics[i] = new KalmanFilter(mn, numTransformVals);  
										  
		timeStamps[i] = 0; 
	}
	
	
	alpha = alphaDynamics[getNumDynamicsModels()-1]; 
	nu = nuDynamics[getNumDynamicsModels()-1]; 
	
	//Note: This sets these values to the defaults for IMM, which are different than the ones for KF.
	setMuAlphaPrior();
	setRAlpha();
	setQAlpha();
	setSigmaAlphaPrior(); 
	
	setMuNuPrior();
	setRNu();
	setQNu();
	setSigmaNuPrior(); 
	 */
}

void InternalMotionModel::setObsSize(Size s) {
	if (_IMM_DEBUG) cout << "Setting observation size to " << s.width << "x" << s.height << endl; 
	lambda.setObsSize(s); 
	eta.setObsSize(s); 
}


void InternalMotionModel::setMuAlphaPrior(double vectorVal) {
	for (size_t i= 0; i < getNumDynamicsModels(); i++) {
		alphaDynamics[i].setMuPrior(vectorVal); 
	}
}

void InternalMotionModel::setMuNuPrior(double vectorVal) {
	for (size_t i= 0; i < getNumDynamicsModels(); i++) {
		nuDynamics[i].setMuPrior(vectorVal); 
	}
}

void InternalMotionModel::setSigmaAlphaPrior(double diagval) {
	for (size_t	 i = 0; i < getNumDynamicsModels(); i++) {
		alphaDynamics[i].setSigmaPrior(diagval); 
	}
}

void InternalMotionModel::setSigmaNuPrior(double diagval) {
	for (size_t	 i = 0; i < getNumDynamicsModels(); i++) {
		nuDynamics[i].setSigmaPrior(diagval); 
	}
}


void InternalMotionModel::setQAlpha(double diagval){	
	for (size_t	 i = 0; i < getNumDynamicsModels(); i++) {
		alphaDynamics[i].setQ(diagval); 
	}
}

void InternalMotionModel::setQNu(double diagval){	
	for (size_t	 i = 0; i < getNumDynamicsModels(); i++) {
		nuDynamics[i].setQ(diagval); 
	}
}


void InternalMotionModel::setRAlpha(double diagval){
	for (size_t	 i = 0; i < getNumDynamicsModels(); i++) {
		alphaDynamics[i].setR(diagval); 
	}
	
}

void InternalMotionModel::setRNu(double diagval){
	for (size_t	 i = 0; i < getNumDynamicsModels(); i++) {
		nuDynamics[i].setR(diagval); 
	}
	
}

void InternalMotionModel::setMuLambdaPrior(double val) {
	lambda.setMuPrior(val); 	
}

void InternalMotionModel::setMuEtaPrior(double val) {
	eta.setMuPrior(val); 	
}


void InternalMotionModel::setSigmaSquaredLambdaPrior(double val) {
	lambda.setSigmaSquaredPrior(val); 
}

void InternalMotionModel::setSigmaSquaredEtaPrior(double val) {
	eta.setSigmaSquaredPrior(val); 
}

void InternalMotionModel::setWorldSizePadFactor(double val) {
	lambda.setWorldSizePadFactor(val); 
	eta.setWorldSizePadFactor(val); 
}


void InternalMotionModel::setRSquaredLambda(double std) {
	lambda.setRSquared(std); 
}
void InternalMotionModel::setRSquaredEta(double std) {
	eta.setRSquared(std); 
}


void InternalMotionModel::setQSquaredLambda(double std) {
	lambda.setQSquared(std); 
}
void InternalMotionModel::setQSquaredEta(double std) {
	eta.setQSquared(std); 
}

void InternalMotionModel::getMotionFeatures(const Mat &action, Mat &features) {
	if (_IMM_DEBUG) cout << "Getting motion features (copying action to features)" << endl ;
	action.copyTo(features);
	
	/*
	int i; 
	int ind = 0; 
	for (i = 0; i < action.rows; i++) {
		features.at<double>(ind, 0) = action.at<double>( i, 0); 
		ind++; 
	}
	features.at<double>(ind,0) = 1;
							  */
}


likelihood InternalMotionModel::motionModelLogLikelihood(const Mat &tau, const Mat &action) {  
	getMotionFeatures(action, features); 
    Mat absFeat = abs(features);
	//getCMatFromFeatures(features, cMat, absCMat); 
	
	//NOTE: set alpha's q-matrix to nu->getObsMean(absCMat, matNx1a) -- square matNx1a
	if (useMotorUncertaintyInLearning) {
		nu->getObsMean(absFeat, matNx1b); 
		matNx1b = matNx1b.mul(matNx1b); 
		alpha->setQDiag(matNx1b);
	}
	return alpha->modelLogLikelihood(features, tau); 
}

likelihood InternalMotionModel::sensorModelLogLikelihoodFast(const Mat &tau, IplImage* seenImage, double scale) {
	return lambda.modelLogLikelihoodFast(tau, seenImage, scale); 
}

likelihood InternalMotionModel::sensorModelLogLikelihood(const Mat &tau, IplImage* seenImage) {
	return lambda.modelLogLikelihood(tau, seenImage); 
}

double InternalMotionModel::obsLogLikelihood(const Mat &tau, IplImage* seenImage, const Mat &action, int frameNum) {
	return getObsLogLikelihood(tau, seenImage, action, frameNum).val; 
}

likelihood InternalMotionModel::getObsLogLikelihood(const Mat &tau, IplImage* seenImage, const Mat &action, int frameNum) {
	setCurrentAlpha(frameNum); 
	likelihood retval; 
	likelihood motion = motionModelLogLikelihood(tau, action); 
	likelihood sensor = sensorModelLogLikelihood(tau, seenImage); 
	retval.grad = motion.grad + sensor.grad; 
	retval.val = motion.val + sensor.val; 
	resetCurrentAlpha(); 
	return retval; 
}


size_t InternalMotionModel::getNumDynamicsModels() const {
	return numDynamicsModels; 
}

void InternalMotionModel::updateSensorModel(IplImage* seenImage, const Mat &tau){	
	IplImage* diffim = cvCreateImage(cvSize(seenImage->width,seenImage->height), seenImage->depth, seenImage->nChannels);
	lambda.updateModel(seenImage, tau, diffim);
	eta.updateModel(diffim, tau); 
	cvReleaseImage(&diffim); 
}

void InternalMotionModel::updateMotionModel(const Mat &action, const Mat &tau) {
	if (_IMM_DEBUG) cout << "Getting motion features." << endl; 
	getMotionFeatures(action, features); 
	//if (_IMM_DEBUG) cout << "Getting cMat." << endl; 
	//getCMatFromFeatures(features, cMat, absCMat); 
    Mat absFeat = abs(features); 
	
	//NOTE: set alpha's q-matrix to nu->getObsMean(absCMat, matNx1a) -- square matNx1a
	
	if (useMotorUncertaintyInLearning) {
		if (_IMM_DEBUG) cout << "Adjusting alpha's Q based on uncertainty model" << endl; 
		nu->getObsMean(absFeat, matNx1b); 
		matNx1b = matNx1b.mul( matNx1b); 
		alpha->setQDiag(matNx1b);
	}
	
	if (_IMM_DEBUG) cout << "Getting mean transform for action." << endl; 

	alpha->getObsMean(features, matNx1a); 
	
	int xdiff = matNx1a.at<double>( 0, 0)-tau.at<double>( 0, 0); 
	int ydiff = matNx1a.at<double>( 1, 0)-tau.at<double>( 1, 0);  
	xdiff = xdiff < 0 ? -xdiff : xdiff; 
	ydiff = ydiff < 0 ? -ydiff : ydiff; 
	
	
	if (_IMM_DEBUG) cout << "Updating motion model kalman filter." << endl; 
	alpha->updateModel(features, tau);
	
	matNx1a.at<double>( 0, 0) = xdiff; 
	matNx1a.at<double>( 1, 0) = ydiff; 
	
	if (_IMM_DEBUG) cout << "Updating noise model kalman filter." << endl; 
	nu->updateModel(absFeat, matNx1a); 
	nu->rectify(); 
	
}

void InternalMotionModel::updateModel(IplImage* seenImage, const Mat &action, const Mat &tau, int frameNum){
	
	setCurrentAlpha(frameNum); 
	if (_IMM_DEBUG) cout << "Updating action model." << endl; 
	updateMotionModel(action, tau); 
	if (_IMM_DEBUG) cout << "Resetting alpha." << endl; 
	resetCurrentAlpha(); 
	
	if (frameNum >= 0 && frameNum <(int)getNumDynamicsModels()-1) return; 
	updateSensorModel(seenImage, tau); 
	
	if (_IMM_DEBUG) cout << "Updating inverse action model." << endl; 
	
	getMotionFeatures(tau, invFeatures); 
	//getCMatFromFeatures(invFeatures, invCMat); 
	inverseMotionModel.updateModel(invFeatures, action); 
}



vector<double> InternalMotionModel::getTauVectorMeanForAction(vector<double> action, int frameNum) {
	vector<double> retval(action.size(), 0); 
	for (int i = 0; i < matNx1b.rows; i++) {
		matNx1b.at<double>(i, 0) = action[i]; 
	}
	getTauMeanForAction(matNx1b, matNx1a, frameNum); 
	
	for (int i = 0; i < matNx1a.rows; i++) {
		retval[i] = matNx1a.at<double>( i, 0); 
	}
	return retval; 
}

void InternalMotionModel::recommendActionForDesiredTransform(const Mat &tau, const vector<actionRecord> &history, double decisionTimeStamp, Mat &action) {
	//vector<double> tauOffset(tau.rows, 0); 
	//vector<double> endPos; 
	//vector<double> currPos1; 
	//vector<double> currPos2; 
	Mat  oldAction, tauOffset, endPos, currPos1, currPos2; 
	tauOffset = Mat::zeros(tau.size(),CV_64F); 
	
	if (_IMM_DEBUG) cout << "Old Tau: " << endl; 
	if (_IMM_DEBUG) printMat(tau); 
	
	//for each action we remember
	for (size_t hist_elem = 0; hist_elem < history.size(); hist_elem++) {
		if (_IMM_DEBUG) cout << "Getting endPos for history " << hist_elem << endl; 
		//get its position at the end
		oldAction = history[hist_elem].actionVals; 
		///oldAction.create(history[hist_elem].actionVals.size(),1, CV_64F); 
		///for (size_t i = 0; i < history[hist_elem].actionVals.size(); i++) {
		///	oldAction.at<double>(i,0) = history[hist_elem].actionVals[i]; 
		///}
		
		getTauMeanForAction(oldAction, endPos); 
		//endPos = getTauVectorMeanForAction(history[hist_elem].actionVals); 
		//get its position now
		double actTime = decisionTimeStamp - history[hist_elem].timeStamp; 
		for (size_t i = 0; i < getNumDynamicsModels(); i++) {			
			if (_IMM_DEBUG) cout << "actTime is " << actTime << "; timeStamp is " << timeStamps[i] << endl; 
			if (actTime < timeStamps[i]) {
				if (i == 0) {
					
					if (_IMM_DEBUG) cout << "Getting mean for this time" << endl; 
					getTauMeanForAction(oldAction, currPos1,i); 
					tauOffset += endPos-currPos1; 
					/*
					currPos1 = getTauVectorMeanForAction(history[hist_elem].actionVals, i); 
					for (size_t j = 0; j < tauOffset.size(); j++) {
						tauOffset[j] = tauOffset[j] + endPos[j] - currPos1[j]; 						
					}*/
					break; 
				} 
				else {
					
					if (_IMM_DEBUG) cout << "Getting mean for this time" << endl; 
					//currPos1 = getTauVectorMeanForAction(history[hist_elem].actionVals, i); 
					getTauMeanForAction(oldAction, currPos1,i); 
					if (_IMM_DEBUG) cout << "Getting mean for last time" << endl; 
					//currPos2 = getTauVectorMeanForAction(history[hist_elem].actionVals, i-1);
					getTauMeanForAction(oldAction, currPos2,i-1); 
					double twoFrac = (timeStamps[i]-actTime)/(timeStamps[i]-timeStamps[i-1]); 
					double oneFrac = 1-twoFrac; 
					
					tauOffset += endPos - oneFrac*currPos1 - twoFrac*currPos2; 
					/*
					if (_IMM_DEBUG) cout << "Calculating adjustment to offset" << endl; 
					for (size_t j = 0; j < tauOffset.size(); j++) {
						tauOffset[j] = tauOffset[j] + endPos[j] - oneFrac*currPos1[j] - twoFrac*currPos2[j]; 						
					}*/
					break; 
				}
			}
		}
		if (_IMM_DEBUG) cout << "New tauOffset is " << endl; 
		if (_IMM_DEBUG) printMat(tauOffset); 
					
	}
	
	if (_IMM_DEBUG) cout << "Adjusting goal" << endl; 
	/*
	for (int i = 0; i < tau.rows; i++) {
		matNx1b.at<double>( i, 0) =tau.at<double>( i, 0)-tauOffset[i]; 
	}*/
	matNx1b = tau-tauOffset; 
	
	if (_IMM_DEBUG) cout << "New Tau: " << endl; 
	if (_IMM_DEBUG) printMat(matNx1b); 
				
	
	if (_IMM_DEBUG) cout << "Getting action" << endl; 
	recommendActionForDesiredTransform(matNx1b, action)	;
	
}

void InternalMotionModel::recommendActionForDesiredTransform(const Mat &tau, Mat &action) {
	if (_IMM_DEBUG) cout << "Getting recommended action" << endl; 
	if (useLearnedInverseModel) {		
		getMotionFeatures(tau, invFeatures); 
		//getCMatFromFeatures(invFeatures, invCMat); 
		inverseMotionModel.getObsMean(invFeatures, action); 
	} else {
		Mat noiseMean = nu->mu.reshape(0,numTransformVals).t(); 
		noiseMean = noiseMean.mul(noiseMean); 
		
		Mat dynamicsMat = alpha->mu.reshape(0,numTransformVals); 
		
		Mat colSum = Mat::ones(1,numTransformVals, CV_64F)*noiseMean.t(); 
		
		Mat colSumDiag = Mat::diag((Mat::ones(1,numTransformVals, CV_64F)*noiseMean.t()).t());
		
		Mat a = (colSumDiag + dynamicsMat.t()*dynamicsMat).inv() * dynamicsMat.t() * tau; 
		a.rowRange(0,numActuators).copyTo(action); 	//Mat noiseMean = nu->mu; 
	}
}



void InternalMotionModel::setCurrentAlpha(int frameNum) {	
	if (_IMM_DEBUG) cout << "Setting current alpha to " << frameNum << endl;  
	if (frameNum >= 0 && frameNum < (int)getNumDynamicsModels()) {
		alpha = &(alphaDynamics[frameNum]); 
		nu = &(nuDynamics[frameNum]); 
	}
}

void InternalMotionModel::resetCurrentAlpha() {
	setCurrentAlpha(getNumDynamicsModels()-1);
}

void InternalMotionModel::getTauMeanForAction(const Mat &action, Mat &tauMat, int frameNum){
	setCurrentAlpha(frameNum); 
	
	getMotionFeatures(action, features); 
	//getCMatFromFeatures(features, cMat, absCMat); 
	alpha->getObsMean(features, tauMat); 
	resetCurrentAlpha(); 
}

Size InternalMotionModel::getEyeMovementUnitsSize() {
	return lambda.getObsSize(); 
}


void InternalMotionModel::updateFrameTime(int frameNum, double frameTime, double timeConst ) {
	if (frameNum < 0 || frameNum >= (int)getNumDynamicsModels()) return; 
	if (timeConst < 0) timeConst = 0; 
	if (timeConst > 1) timeConst = 1; 
	timeStamps[frameNum] = (1-timeConst)*timeStamps[frameNum]+timeConst*frameTime; 	
}

void InternalMotionModel::findMaxLikelihoodTransformFaster(IplImage* seenImage, const Mat &action, Mat &tau, int frameNum){
	setCurrentAlpha(frameNum); 
	
	Size obsSize = Size(seenImage->width, seenImage->height);
	
	
	getTauMeanForAction(action, matNx1a); 
	
	int tau0 = matNx1a.at<double>( 0, 0); 
	int tau1 = matNx1a.at<double>( 1, 0); 
	
	
	double best = -1E30; 
	double besttau0=0; 
	double besttau1=0; 
	
	double searchRad = 1.0/3.0; 
	
	for (int i = -obsSize.height*searchRad; i <= obsSize.height*searchRad; i=i+10) {
		for (int j = -obsSize.width*searchRad; j <= obsSize.width*searchRad; j=j+10) {
			tau.at<double>( 0, 0) =  tau0+j; 
			tau.at<double>( 1, 0) = tau1+i;
			
			if (_IMM_DEBUG) cout << "Getting LL at " << tau0+j << ", " << tau1+i << endl; 
			
			likelihood retval; 
			likelihood motion = motionModelLogLikelihood(tau, action); 
			likelihood sensor = sensorModelLogLikelihoodFast(tau, seenImage, .25); 
			retval.grad = motion.grad + sensor.grad; 
			retval.val = motion.val + sensor.val; 
			
			
			double ll = retval.val; 
			if (ll > best) {
				best = ll; 
				if (_IMM_DEBUG) 
					cout << "Found better one: " << ll << endl; 
				if (_IMM_DEBUG) 
					cout << "At " << tau0+j << ", " << tau1+i << endl;
				besttau0 = tau0+j ; 
				besttau1 = tau1+i ; 
			}
		}
	}
	
	for (int i = besttau1-12; i <= besttau1+12; i++) {
		for (int j = besttau0-12; j <= besttau0+12; j++) {
			tau.at<double>( 0, 0) =  j; 
			tau.at<double>( 1, 0) = i;
			
			if (_IMM_DEBUG) cout << "Getting LL at " << tau0+j << ", " << tau1+i << endl; 
			
			likelihood retval; 
			likelihood motion = motionModelLogLikelihood(tau, action); 
			likelihood sensor = sensorModelLogLikelihoodFast(tau, seenImage, .5); 
			retval.grad = motion.grad + sensor.grad; 
			retval.val = motion.val + sensor.val; 
			
			double ll = retval.val; 
			if (ll > best) {
				best = ll; 
				if (_IMM_DEBUG) 
					cout << "Found better one: " << ll << endl; 
				if (_IMM_DEBUG) 
					cout << "At " << j << ", " << i << endl;
				besttau0 = j ; 
				besttau1 = i ; 
			}
		}
	}
	
	tau.at<double>( 0, 0) =  besttau0; 
	tau.at<double>( 1, 0) = besttau1;
	
	resetCurrentAlpha(); 
}

void InternalMotionModel::findMaxLikelihoodTransform(IplImage* seenImage, const Mat &action, Mat &tau, int frameNum) {
	
	setCurrentAlpha(frameNum); 
	
	Size obsSize = Size(seenImage->width, seenImage->height);
	
	
	getTauMeanForAction(action, matNx1a); 
	
	int tau0 = matNx1a.at<double>( 0, 0); 
	int tau1 = matNx1a.at<double>( 1, 0); 
	
	
	double best = -1E30; 
	double besttau0=0; 
	double besttau1=0; 
	
	double searchRad = 1.0/3.0; 	
	for (int i = -obsSize.height*searchRad; i <= obsSize.height*searchRad; i=i+8) {
		for (int j = -obsSize.width*searchRad; j <= obsSize.width*searchRad; j=j+8) {
			tau.at<double>( 0, 0) =  tau0+j; 
			tau.at<double>( 1, 0) = tau1+i;
			
			if (_IMM_DEBUG) cout << "Getting LL at " << tau0+j << ", " << tau1+i << endl; 
			double ll = obsLogLikelihood(tau, seenImage, action); 
			if (ll > best) {
				best = ll; 
				if (_IMM_DEBUG) 
					cout << "Found better one: " << ll << endl; 
				if (_IMM_DEBUG) 
					cout << "At " << tau0+j << ", " << tau1+i << endl;
				besttau0 = tau0+j ; 
				besttau1 = tau1+i ; 
			}
		}
	}
	
	for (int i = besttau1-16; i <= besttau1+16; i++) {
		for (int j = besttau0-16; j <= besttau0+16; j++) {
			tau.at<double>( 0, 0) =  j; 
			tau.at<double>( 1, 0) = i;
			
			if (_IMM_DEBUG) cout << "Getting LL at " << tau0+j << ", " << tau1+i << endl; 
			double ll = obsLogLikelihood(tau, seenImage, action); 
			if (ll > best) {
				best = ll; 
				if (_IMM_DEBUG) 
					cout << "Found better one: " << ll << endl; 
				if (_IMM_DEBUG) 
					cout << "At " << j << ", " << i << endl;
				besttau0 = j ; 
				besttau1 = i ; 
			}
		}
	}
		
	tau.at<double>( 0, 0) =  besttau0; 
	tau.at<double>( 1, 0) = besttau1;
	
	resetCurrentAlpha(); 
}

void InternalMotionModel::updateModelMAP(IplImage* seenImage, const Mat &action, int frameNum, double frameTime){
	findMaxLikelihoodTransformFaster(seenImage, action, matNx1a, frameNum);
	updateModel(seenImage, action, matNx1a, frameNum); 
	if (frameNum > -1) updateFrameTime(frameNum, frameTime); 
}

void InternalMotionModel::addToStream(ostream& out) const {
	if (_IMM_DEBUG) cout << "Adding IMM Object To Stream" << endl; 
	
	if (_IMM_DEBUG) cout << "Adding Num Actuators To Stream" << endl; 
	out << numActuators << endl; 
	
	if (_IMM_DEBUG) cout << "Adding Lambda To Stream" << endl; 
	out << lambda ; 
	
	if (_IMM_DEBUG) cout << "Adding Eta To Stream" << endl; 
	out << eta ; 
	
	if (_IMM_DEBUG) cout << "Adding Inverse Model To Stream" << endl; 
	out << inverseMotionModel ; 
	
	if (_IMM_DEBUG) cout << "Adding Num Dynamics Models To Stream" << endl; 
	out << getNumDynamicsModels() << endl; 
	for (size_t i = 0; i < getNumDynamicsModels(); i++) {
		
		if (_IMM_DEBUG) cout << "Adding Alpha Model " << i << " To Stream" << endl; 
		out << alphaDynamics[i]; 
		
		if (_IMM_DEBUG) cout << "Adding Nu Model " << i << " To Stream" << endl; 
		out << nuDynamics[i]; 
		
		out << timeStamps[i] << endl; 
	}
	
	if (_IMM_DEBUG) cout << "Adding Learning Parameters To Stream" << endl; 
	out << useMotorUncertaintyInLearning << " " << lambda.getUsesExternalREstimate() << endl; 
}

void InternalMotionModel::readFromStream(istream& in) {
	if (_IMM_DEBUG) cout << "Reading IMM Object from Stream" << endl; 
	int numActs, numDynMods, useR; 
	
	if (_IMM_DEBUG) cout << "Reading numActuators from Stream" << endl; 
	in >> numActs; 
	 
	//if (lambda != NULL) delete(lambda); 
	lambda = ImageKalmanFilter(); 
	
	//if (eta != NULL) delete(eta); 
	eta = ImageKalmanFilter(); 
	if (_IMM_DEBUG) cout << "Reading Lambda from Stream" << endl;
	in >> lambda;
	if (_IMM_DEBUG) cout << "Reading Eta from Stream" << endl; 
	in >> eta; 
	
	if (_IMM_DEBUG) cout << "Resetting numActuators to " << numActs << endl; 
	setNumActuators(numActs); 
	//if (inverseMotionModel != NULL) delete(inverseMotionModel); 
	inverseMotionModel = MatrixKalmanFilter(); 
	
	if (_IMM_DEBUG) cout << "Reading InverseModel from Stream" << endl; 
	in >> inverseMotionModel; 
	
	if (_IMM_DEBUG) cout << "Reading numDynamicsModels from Stream" << endl; 
	in >> numDynMods; 
	
	if (_IMM_DEBUG) cout << "Resetting numDynamicsModels to " << numDynMods<< endl; 
	setNumDynamicsModels(numDynMods); 
	for (size_t i = 0 ; i < getNumDynamicsModels(); i++) {
		//if (alphaDynamics[i] != NULL) delete(alphaDynamics[i]); 
		//if (nuDynamics[i] != NULL) delete(nuDynamics[i]); 
		alphaDynamics[i] = MatrixKalmanFilter(); 
		nuDynamics[i] = MatrixKalmanFilter(); 
		
		if (_IMM_DEBUG) cout << "Reading Alpha " << i << " from Stream" << endl; 
		in >> alphaDynamics[i]; 
		
		if (_IMM_DEBUG) cout << "Reading Nu " << i << " from Stream" << endl; 
		in >> nuDynamics[i]; 
		
		in >> timeStamps[i]; 
	}
	
	if (_IMM_DEBUG) cout << "Resetting current alpha" << endl; 
	resetCurrentAlpha(); 
	
	if (_IMM_DEBUG) cout << "Reading Learning Params from Stream" << endl; 
	in >> useMotorUncertaintyInLearning >> useR; 
		
	if (_IMM_DEBUG) cout << "Setting useSensorUncertaintyInLearning to " << useR << endl; 
	setUseSensorUncertaintyInLearning(useR); 
}



ostream& operator<< (ostream& ofs, const InternalMotionModel &model) {
	model.addToStream(ofs); 
	return ofs; 
}

istream& operator>> (istream& ifs, InternalMotionModel &model) {
	model.readFromStream(ifs); 
	return ifs; 
}
