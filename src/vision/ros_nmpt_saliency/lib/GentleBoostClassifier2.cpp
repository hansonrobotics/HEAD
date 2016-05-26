/*
 *  GentleBoostClassifier2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/21/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "GentleBoostClassifier2.h"
#include "Feature2.h"
#include "DebugGlobals.h" 
#include "NMPTUtils.h"
#include "BlockTimer.h"

using namespace cv; 
using namespace std; 

GentleBoostClassifier2::GentleBoostClassifier2() {
	numTrain = 0; 
	testingPerformance = 0; 
	numTest = 0; 
	numFeatures= 0; 
	useFast = 1; 
	sharingPatchList = 0; 
	featureName="HaarFeature"; 
	basePatchSize = cvSize(-1,-1);
	patchList = NULL; 
	setUseFastPatchList(useFast);
	
	currentBGFileNum = 1; 
	dontTrainRejected = 1; 
	keepNonRejectedBGPatches = 1; 
	maxPatchesPerImage = 50; 
	runningOutOfNegPatches = 0; 
	ranOutOfNegPatches = 0; 
	useNMSInTraining = 1; 
	disableNMSAcrossScales = 0; 
	
}

GentleBoostClassifier2 & GentleBoostClassifier2::operator=(const GentleBoostClassifier2 &rhs) {
	if (this != &rhs) {
		copy(rhs); 
	}
	return *this; 
}

GentleBoostClassifier2::GentleBoostClassifier2(const GentleBoostClassifier2 &rhs) {
//	*this = rhs; 
	if (this != &rhs) {
		copy(rhs);
	} 
}

void GentleBoostClassifier2::copy(const GentleBoostClassifier2 &rhs) {
	features = rhs.features; 
	numFeatures = rhs.numFeatures; 
	useFast = rhs.useFast; 
	setBasePatchSize(rhs.basePatchSize); //creates pl/fpl, which will be overwritten
	pl = rhs.pl; 
	fpl = rhs.fpl; 
	featureName = rhs.featureName;
	sharingPatchList = rhs.sharingPatchList; 
	
	setTrainingSet(rhs.trainingPatches, rhs.trainingLabels); 
	
	trainingFeatureOutputs.resize(rhs.trainingFeatureOutputs.size()); 
	for (size_t i = 0; i < trainingFeatureOutputs.size(); i++) {
		trainingFeatureOutputs[i] = rhs.trainingFeatureOutputs[i].clone(); 
	}
	trainingFeatureSum = rhs.trainingFeatureSum.clone(); 
	trainingPosteriors = rhs.trainingPosteriors.clone(); 
	trainingPredictions = rhs.trainingPredictions.clone(); 
	trainingWeights = rhs.trainingWeights.clone(); 
	trainingPerformance = rhs.trainingPerformance; 
	numTrain = rhs.numTrain; 
	
	
	testingPatches = rhs.testingPatches; 
	testingLabels = rhs.testingLabels.clone(); 
	testingFeatureOutputs.resize(rhs.testingFeatureOutputs.size()); 
	for (size_t i = 0; i < testingFeatureOutputs.size(); i++) {
		testingFeatureOutputs[i] = rhs.testingFeatureOutputs[i].clone(); 
	}
	testingFeatureSum = rhs.testingFeatureSum.clone(); 
	testingPosteriors = rhs.testingPosteriors.clone(); 
	testingPredictions = rhs.testingPredictions.clone(); 
	testingPerformance = rhs.testingPerformance; 
	numTest = rhs.numTest;	
	
	//patchList is no longer valid
	if (sharingPatchList)
		patchList = rhs.patchList; 
	else 
		setUseFastPatchList(useFast); 
	
	
	posImagesDataset = rhs.posImagesDataset; 
	negImagesDataset = rhs.negImagesDataset; 
	patchDataset = rhs.patchDataset; 
	
	currentBGFileNum = rhs.currentBGFileNum; 
	dontTrainRejected = rhs.dontTrainRejected; 
	keepNonRejectedBGPatches = rhs.keepNonRejectedBGPatches; 
	maxPatchesPerImage = rhs.maxPatchesPerImage; 
	runningOutOfNegPatches = rhs.runningOutOfNegPatches; 
	ranOutOfNegPatches = rhs.ranOutOfNegPatches; 
	useNMSInTraining = rhs.useNMSInTraining; 
	disableNMSAcrossScales = rhs.disableNMSAcrossScales; 
}

Size GentleBoostClassifier2::getBasePatchSize() const {
	return basePatchSize; 
}


void GentleBoostClassifier2::setBasePatchSize(Size s) {
	basePatchSize = s; 
	setSearchParams(s); 
}

GentleBoostClassifier2::~GentleBoostClassifier2(){
}

void GentleBoostClassifier2::searchPatches(const vector<ImagePatch2> &patches,
										   const Mat &labels,
										   Mat &featureSum,
										   Mat &posterior,
										   Mat &predictions,
										   vector<cv::Mat> &featureOutputs,
										   double& perf,
										   Mat &weights, 
										   Mat &survived) const{
	int numPatches = patches.size(); 
	
	featureSum = Mat::zeros(numPatches,1, CV_64F); 
	featureOutputs.resize(numFeatures); 
	weights = Mat::zeros(numPatches,1,CV_64F) + 1.0/numPatches; 
	posterior = Mat::zeros(numPatches,1,CV_64F) + 0.5; 
	predictions = Mat::zeros(numPatches,1,CV_64F) - 1.; 
	survived.create(numPatches,1, CV_8U); 
	survived.setTo(255); 
	perf = 0; 
	
	
	for (int i = 0; i < numFeatures; i++) {
		Mat output; 
		updateValuesForFeature(features[i], patches, labels, featureSum,
							   posterior, predictions, output, perf, weights,
							   featureRejectThresholds[i], survived); 
		featureOutputs[i] = output.clone(); 
	}
}

void GentleBoostClassifier2::updateValuesForFeature(const FeatureRegressor2 &feature,
													const vector<ImagePatch2> &patches,
													const Mat &labels,
													Mat &featureSum,
													Mat &posterior,
													Mat &predictions,
													Mat &out,
													double& perf,
													Mat &weights,
													double threshold,
													Mat &survived) const {
	int numPatches = patches.size(); 
	int hasLabels = labels.rows > 0 && labels.cols > 0 && labels.rows == numPatches; 
	
	feature.predict(patches, out); 
	
	accumulateEvidence(out, featureSum, survived); 
	updateSurvivedList(featureSum, threshold, survived); 
	if (hasLabels) updateWeights(out, labels, weights, survived); 
	calcPosterior(featureSum, posterior); 
	makePredictions(featureSum, predictions, survived); 
	if (hasLabels) perf = getFractionCorrect(labels, predictions); 	
}

void GentleBoostClassifier2::updateSurvivedList(const Mat &featureSum, double threshold, Mat &survived) const {
	if (survived.cols > 0 && survived.rows > 0) 
		bitwise_and(survived, featureSum > threshold, survived, survived); 
}

void GentleBoostClassifier2::accumulateEvidence(const Mat &output, Mat &featureSum, 
												const Mat &survived) const {
	if (_GENTLEBOOST_DEBUG) cout << "Accumulating evidence." << endl; 
	add(featureSum, output, featureSum, survived); 
}

void GentleBoostClassifier2::updateWeights(const Mat &output, const Mat &labels, Mat &weights, 
										   const Mat &survived) const {
	if (_GENTLEBOOST_DEBUG) cout << "Updating weights." << endl; 
	int numPatches = output.rows; 
	int hasLabels = labels.rows > 0 && labels.cols > 0 && labels.rows == numPatches; 
	if (hasLabels) {
		Mat out =  output.mul(labels); 
		out = -out; 
		
		//Here we are scaling the log weight so that its max is an arbitrary
		//constant, e.g. 5, so that things should be numerically stable, 
		//even for large datasets. The larger this constant is, the more 
		//precise the very small weights, but the larger risk we run of
		//numerical instability. 
		double minval,maxval; 
		minMaxLoc(weights,&minval,&maxval); 
		weights = weights - (maxval-5); 
		
		exp(out, out); 
		weights = weights.mul(out);
		if (survived.cols > 0 && survived.rows > 0 && dontTrainRejected) {
			Mat notsurvived; 
			bitwise_not(survived, notsurvived); 
			weights.setTo(0., notsurvived);
		}
		normalize(weights, weights, 1, 0, NORM_L1); 
	}
}

void GentleBoostClassifier2::calcPosterior(const Mat &featureSum, Mat &posterior) const {
	if (_GENTLEBOOST_DEBUG) cout << "Getting posterior." << endl; 
	posterior = (-2.)* featureSum; 
	exp(posterior, posterior); 
	posterior = 1./(1.+posterior); 
}

void GentleBoostClassifier2::makePredictions(const Mat &featureSum, Mat &predictions,
											 const Mat &survived) const {
	if (_GENTLEBOOST_DEBUG) cout << "Inferring labels" << endl; 
	predictions.create(featureSum.rows, 1, CV_64F); 
	predictions.setTo(-1.);
	Mat pos; 
	bitwise_and(featureSum > .5, survived, pos);	
	predictions.setTo(1., pos); 
}

double GentleBoostClassifier2::getFractionCorrect(const Mat &labels, const Mat &predictions) const {
	if (_GENTLEBOOST_DEBUG) cout << "Computing %-Correct." << endl; 
	int numPatches = labels.rows; 
	if (labels.rows != predictions.rows || labels.cols != predictions.cols ||
		labels.type() != predictions.type())
		return countNonZero(labels == predictions) *1.0 / numPatches;
	return 0; 
}

double GentleBoostClassifier2::getChiSq(const Mat &labels, const Mat &posterior) const {
	if (_GENTLEBOOST_DEBUG) cout << "Calculating chisq loss." << endl; 
	Mat binLabels = 0.5 * (labels + 1.); 
	Mat numerator, denominator; 
	numerator = binLabels- posterior;
	numerator = numerator.mul(numerator); 
	denominator = posterior.mul(-posterior+1.); 
	sqrt(denominator, denominator); 
	divide(numerator, denominator, numerator) ;
	double chisq = sum(numerator)[0]; 
	if (isnan(chisq)) chisq = INFINITY; 
	return chisq; 
}


void GentleBoostClassifier2::pickRejectThreshold(const cv::Mat &output, 
												 const cv::Mat &labels, 
												 const cv::Mat &survived,
												 double& threshold, 
												 int& totalPosRejects, 
												 int& totalNegRejects) const {
	/*
	Mat out; 
	reg.predict(patches, out); 
	accumulateEvidence(featureSum, out, survived);
	 */
	threshold = -INFINITY; 
	totalPosRejects = 0; 
	totalNegRejects = 0; 
}

void GentleBoostClassifier2::setTrainingSet(const vector<ImagePatch2> &trainingPatches,
										   const Mat &trainingLabels){
	if (basePatchSize.width <= 0 || basePatchSize.height <= 0) 
		setBasePatchSize( trainingPatches[0].getImageSize());
	
	this->trainingPatches = trainingPatches; 
	trainingLabels.convertTo(this->trainingLabels, CV_64F); 
	
	negPatches.clear(); 
	posPatches.clear(); 
	for (size_t i = 0; i < trainingPatches.size(); i++) {
		if (trainingLabels.at<double>(i,0) > 0)
			posPatches.push_back(&this->trainingPatches[i]); 
		else 
			negPatches.push_back(&this->trainingPatches[i]); 
	}
	
	searchPatches(this->trainingPatches,
				  this->trainingLabels,
				  trainingFeatureSum,
				  trainingPosteriors,
				  trainingPredictions,
				  trainingFeatureOutputs,
				  trainingPerformance,
				  trainingWeights,
				  trainingSurvived); 
	
	if (_TRAINING_DEBUG) printDebugState(); 
	if (_CASCADE_DEBUG) cout << "There are " << posPatches.size() 
		<< " pos patches and " << negPatches.size() << " neg patches." << endl;  
} 

void GentleBoostClassifier2::printDebugState() const {
	cout << "GentleBoostClassifier has: " << endl; 
	cout << "  Patch Size: " << basePatchSize.width << "x" << basePatchSize.height << endl; 
	cout << "  Using features: " << numFeatures << " of " << features.size() << endl; 
	int numTraining = trainingPatches.size(); 
	cout << "  # Training: " << numTraining << " training patches." << endl; 
	if (numTraining > 0) {
		int numPrint = numTraining < 10?numTraining: 10;
		cout << "  Some current training labels: " ; 
		NMPTUtils::printMat(trainingLabels.rowRange(0,numPrint).t()); 
		cout << "  Some current training sums: " ; 
		NMPTUtils::printMat(trainingFeatureSum.rowRange(0,numPrint).t()); 
		cout << "  Some current training posteriors: " ; 
		NMPTUtils::printMat(trainingPosteriors.rowRange(0,numPrint).t()); 
		cout << "  Some current training predictions: " ; 
		NMPTUtils::printMat(trainingPredictions.rowRange(0,numPrint).t()); 
		cout << "  Some current training weights: " ; 
		NMPTUtils::printMat(trainingWeights.rowRange(0,numPrint).t()); 
		cout << "  Current training performance: " << trainingPerformance << endl;  
		NMPTUtils::printMat(trainingWeights.rowRange(0,numPrint).t()); 
		cout << "  Current training survived: " ;  
		NMPTUtils::printMat(trainingSurvived.rowRange(0,numPrint).t()); 
	}
	
}

void GentleBoostClassifier2::setTestingSet(const vector<ImagePatch2> &testingPatches,
										  const Mat &testingLabels){
	
	this->testingPatches = testingPatches; 
	testingLabels.convertTo(this->testingLabels, CV_64F); 
	Mat testingWeights; 
	searchPatches(this->testingPatches,
				  this->testingLabels,
				  testingFeatureSum,
				  testingPosteriors,
				  testingPredictions,
				  testingFeatureOutputs,
				  testingPerformance,
				  testingWeights,
				  testingSurvived); 
} 

void GentleBoostClassifier2::addFeature(const Feature2* nextFeature){
	addFeatureBoosted(nextFeature); 
} 

void GentleBoostClassifier2::addFeatureBoosted(const Feature2* nextFeature, int boostRounds){
	
	if (trainingLabels.rows < 0 || trainingLabels.rows != (int)trainingPatches.size()){
		cout << "Warning: Must set gentle boost training patches and labels before adding features." << endl;
		return; 
	}
	
	if (!features.empty() ) {
		if (nextFeature->getPatchSize().width != basePatchSize.width || 
			nextFeature->getPatchSize().height != basePatchSize.height) {
			cout << "Warning: All features added to classifier must have the same size." << endl;
			return; 
		} 
	}
	else {
		setBasePatchSize(nextFeature->getPatchSize()); 
	}
	
	if (_GENTLEBOOST_DEBUG) cout << "Adding feature with " << boostRounds << " rounds of boosting." << endl; 
	if (_GENTLEBOOST_DEBUG) printDebugState();
	
	vector<FeatureRegressor2> regs; 
	
	Mat wts = trainingWeights.clone(), newsum = trainingFeatureSum.clone(); 
	Mat out; 
	
	for (int i = 0; i < boostRounds; i++) {
		if (_GENTLEBOOST_DEBUG) cout << "Creating regressor # " << i << endl; 
		regs.push_back(FeatureRegressor2(nextFeature));
		if (_GENTLEBOOST_DEBUG) cout << "Training" << endl; 
		regs[i].train(numBins, trainingPatches, trainingLabels, trainingWeights); 
		if (_GENTLEBOOST_DEBUG) cout << "Predicting" << endl; 
		regs[i].predict(trainingPatches, out);
		newsum += out; 
		updateWeights(out, trainingLabels, wts, trainingSurvived); 
	}
	
	for (int i = 1; i < boostRounds; i++) {
		if (_GENTLEBOOST_DEBUG) cout << "Combining LUTs " << i << endl; 
		regs[0].combineLUTs(regs[i]); 
	}
	
	int posRejects, negRejects; 
	double threshold; 
	
	features.push_back(regs[0]); 
	
	pickRejectThreshold(newsum, trainingLabels, 
						trainingSurvived, threshold, posRejects, negRejects); 
	
	
	featureRejectThresholds.push_back(threshold); 
	
	numFeatures = features.size();  
	Mat output; 
	updateValuesForFeature(features.back(), trainingPatches, trainingLabels, trainingFeatureSum,
						   trainingPosteriors, trainingPredictions, output, 
						   trainingPerformance, trainingWeights, featureRejectThresholds.back(),
						   trainingSurvived); 
	trainingFeatureOutputs.push_back(output.clone()); 	
	
	if (_GENTLEBOOST_DEBUG) printDebugState();
	
	if (!testingPatches.empty() > 0) {
		Mat testingWeights(testingLabels.size(), CV_64F, 1); 
		updateValuesForFeature(features[numFeatures-1], testingPatches, testingLabels, testingFeatureSum,
							   testingPosteriors, testingPredictions, output, 
							   testingPerformance, testingWeights, featureRejectThresholds.back(),
							   testingSurvived); 
		testingFeatureOutputs.push_back(output.clone()); 
	}
} 

int GentleBoostClassifier2::getNumFeaturesUsed() const{
	return numFeatures; 
}

int GentleBoostClassifier2::getNumFeaturesTotal() const {
	return (int)features.size(); 
}

void GentleBoostClassifier2::setNumFeaturesUsed(int num) {
	if (num < (int)features.size() && num >= 0) 
		numFeatures = num; 
	else 
		numFeatures = features.size();
}

int GentleBoostClassifier2::getNumTrainingPatches()  const {
	return trainingPatches.size(); 
} 

int GentleBoostClassifier2::getNumTestingPatches() const {
	return testingPatches.size(); 
} 

void GentleBoostClassifier2::getImageHeaderForTrainingPatch(cv::Mat & dest, int patchNum) const {
	if (patchNum >= 0 && patchNum < getNumTrainingPatches()) 
		dest = trainingPatches[patchNum].getImageHeader().clone(); 
}

void GentleBoostClassifier2::getImageHeaderForTestingPatch(cv::Mat &dest, int patchNum) const{
	if (patchNum >= 0 && patchNum < getNumTestingPatches()) 
		dest = testingPatches[patchNum].getImageHeader().clone(); 
} 

PerformanceMetrics GentleBoostClassifier2::trainOneRound(int patience, int boostRounds) {
	
	if (_TRAINING_DEBUG) cout << "Adding feature " << getNumFeaturesTotal()+1 << endl; 
	if (_TRAINING_DEBUG) cout << "Replacing easy background examples with harder ones." << endl; 
	
	Feature2* feat = getGoodFeature(patience, featureName); 
	if (_TRAINING_DEBUG) cout << "Done with Feature Tournament." << endl; 
	
	if (_TRAINING_DEBUG) cout<< "Adding feature to classifier." << endl; 
	
	addFeatureBoosted(feat, boostRounds );
	
	
	delete(feat); 
	
	PerformanceMetrics perf; 
	getPerformanceMeasures(NULL, perf); 
	if (_TRAINING_DEBUG) cout << "Chi-Sq: " << perf.chisq  << " ; Time per patch: " << perf.time_per_patch << endl; 
	return perf;  
}

FeatureRegressor2 GentleBoostClassifier2::getFeatureAndTuningCurveNumber(int num) const {
	return features[num]; 
}

int GentleBoostClassifier2::updateSurvivalStats() {
	double maxNeg=-INFINITY, maxPos=-INFINITY, minNeg=INFINITY, minPos=INFINITY; 
	int negDead = 0; 
	posSurvived.clear(); 
	negSurvived.clear(); 
	for (unsigned int i = 0; i < trainingPatches.size(); i++) {
		double value = trainingFeatureSum.at<double>( i, 0); 
		if (trainingLabels.at<double>(i, 0) > 0 ) {
			minPos=minPos>value?value:minPos; 
			maxPos=maxPos<value?value:maxPos; 
			if (!trainingSurvived.at<uint8_t>( i, 0)) posSurvived.push_back(0); 
			else posSurvived.push_back(1); 
		} else {
			minNeg=minNeg>value?value:minNeg; 
			maxNeg=maxNeg<value?value:maxNeg; 
			if (!trainingSurvived.at<uint8_t>( i, 0)) {negSurvived.push_back(0); negDead++;}
			else {negSurvived.push_back(1); } 
		}
	}
	
	if (_BLACKOUT_DEBUG) cout << negDead << " bg patches didn't survive and need replacing." << endl; 
	cout << "Max Pos: " << maxPos << " ; Min Pos: " << minPos << "\t;\t Max Neg: "
	<< maxNeg << " ; Min Neg: " << minNeg << endl; 
	
	return negDead; 
}

void GentleBoostClassifier2::getPerformanceMeasures(const Feature2* candidate, PerformanceMetrics& perf) const {
	
	//Get Performance has three modes depending on the candidate feature: 
	//--If the feature is NULL, calculates the current chi-squared error
	//--If the feature is currently in the classifier, calculates the error if
	//  we remove the feature
	//--Otherwise, calculates the error if we were to add this feature.
	
	perf.pos_rejects = 0; 
	perf.neg_rejects = 0; 
	perf.total_pos = posPatches.size();
	perf.total_neg = negPatches.size(); 
	perf.prev_pos_rejects = 0; 
	perf.prev_neg_rejects = 0; 
	perf.threshold = -INFINITY; 
	
	if (trainingPatches.empty() ||  trainingLabels.rows <= 0) {
		cout << "Warning: Must set training patches & labels before calling getPerformanceMeasures" << endl; 
		return; 
	}
	
	if (_TRAINING_DEBUG) cout << "Getting chisq for feature" << endl; 
	
	Mat newSum; 
	
	perf.time_per_patch = 0; 
	
	if (candidate == NULL) { //Case: Current performance
		if (_TRAINING_DEBUG)  cout << "Testing current performance." << endl; 
		newSum = trainingFeatureSum.clone(); 
	} else {
		int ind = -1; 
		if (_TRAINING_DEBUG) cout << "Checking if duplicate feature." << endl; 
		for (int i = 0; i < numFeatures; i++) {
			if (candidate == features[i].getFeature()) {
				ind = i; 
				break; 
			}
		}
		
		if (ind > -1) { //Case: Performance without this feature.
			if (_TRAINING_DEBUG) cout << "Found duplicate feature at " << ind << "." << endl; 
			newSum = trainingFeatureSum - trainingFeatureOutputs[ind]; 
		} else {
			if (_TRAINING_DEBUG) cout << "Training feature to see how it would do." << endl; 
			FeatureRegressor2 reg(candidate); 
			
			if (_TRAINING_DEBUG) cout << "Calling reg.train()" << endl; 
			reg.train(numBins, trainingPatches, trainingLabels, trainingWeights);
			
			if (reg.getLUTRange() <= 0) {
				perf.chisq = INFINITY; 
				perf.pos_rejects = posPatches.size(); 
				perf.neg_rejects = 0; 
				perf.time_per_patch = INFINITY; 
				perf.threshold = -INFINITY; 
				return; 
			}
			
			if (_TRAINING_DEBUG) cout << "Calling reg.predict()" << endl; 
			BlockTimer bt; 
			bt.blockRestart(0); 
			reg.predict(trainingPatches, newSum);
			perf.time_per_patch = bt.getCurrTime(0) / trainingPatches.size(); 
			
			accumulateEvidence(trainingFeatureSum, newSum, trainingSurvived); 
		}
	}	
	
	if (_TRAINING_DEBUG) cout << "Counting rejections" << endl; 
	for (int i = 0; i < trainingLabels.rows; i++) {
		if (trainingSurvived.at<uint8_t>(i,0)) continue; 
		if (trainingLabels.at<double>(i,0) > 0) perf.prev_pos_rejects++; 
		else perf.prev_neg_rejects++; 
	}
	
	
	if (_TRAINING_DEBUG) cout << "Updating stats" << endl; 
	perf.pos_rejects = perf.prev_pos_rejects; 
	perf.neg_rejects = perf.prev_neg_rejects; 	
	perf.time_per_patch = perf.time_per_patch*1.0/(perf.total_pos-perf.prev_pos_rejects+perf.total_neg-perf.prev_neg_rejects); 

	if (candidate != NULL)  {
		pickRejectThreshold(newSum, trainingLabels, trainingSurvived, 
							perf.threshold, perf.pos_rejects, perf.neg_rejects); 
	} else {
		if (numFeatures > 0)
			perf.threshold = featureRejectThresholds[numFeatures-1]; 
	}
	
	Mat prob; 
	calcPosterior(newSum, prob);
	perf.chisq = getChiSq(trainingLabels, prob); 
	
}

void GentleBoostClassifier2::getProbabilityMap(PatchList2* patches, Mat &dest) const {
	for (int i = 0; i < numFeatures; i++) {
		features[i].predictPatchList(patches); 
		patches->accumulateAndRemovePatchesBelowThreshold(featureRejectThresholds[i]); 
	}
	cv::Size s = patches->getImageSizeAtScale();
	Mat m; 
	patches->getAccumImage(m); 
	calcPosterior(m(Rect(0,0,s.width,s.height)), dest);
}

void GentleBoostClassifier2::setTrainingSet(const PatchDataset2 &dataset) {
	patchDataset = dataset; 
	vector<ImagePatch2> patches = patchDataset.getPatches();
	Mat labels;
	patchDataset.getLabels(labels); 
	
	setTrainingSet(patches, labels); 
	
	posImagesDataset = patchDataset.getPosImagesDataset(); 
	negImagesDataset = patchDataset.getNegImagesDataset(); 
	
	setUseFastPatchList(patchDataset.getUseFastPatchList()); 
}

void GentleBoostClassifier2::setTestingSet(const PatchDataset2 &dataset) {	
	vector<ImagePatch2> patches = dataset.getPatches();
	Mat labels; 
	dataset.getLabels(labels); 
	
	setTestingSet(patches, labels); 	
}

Feature2* GentleBoostClassifier2::getGoodFeature(int patience, const string &featureType) const {
	if (patience < 1) patience = 1; 
	int featurePoolSize = 20*patience;//200; 
	double reduceFraction = .05; 
	int numSimilar = 10*patience;//50;//10; 
	
	return getGoodFeatureViaTournament(featureType, featurePoolSize, numSimilar, reduceFraction); 
}

Feature2* GentleBoostClassifier2::getGoodFeatureViaTournament(const string &featureType,
															 int startPoolSize, 
															 int similarFeatures,
															 double keepPortion) const {	
	
	vector<FeaturePerformance2> candidates; 
	if (_TRAINING_DEBUG) cout << "Getting Initial Pool & performance." << endl; 
	for (int i = 0; i < startPoolSize; i++) {
		if (_TRAINING_DEBUG) cout << "Getting feature " << i <<  " with size " << basePatchSize.width << "x" << basePatchSize.height << endl; 
		FeaturePerformance2 f; 
		f.feat = Feature2::getFeatureOfType(featureType, basePatchSize); //new HaarFeature(basePatchSize);
		
		if (_TRAINING_DEBUG) cout << "Checking Performance of feature " << i << endl; 
		getPerformanceMeasures(f.feat,f.perf);
		candidates.push_back(f); 
	}
	
	if (_TRAINING_DEBUG) cout << "Finished filling pool" << endl; 
	if (_TRAINING_DEBUG) cout << "Size is " << candidates.size() << endl; 
	
	int roundSimilarFeatures = similarFeatures; 
	while(candidates.size() > 1) {
		vector<double> test; 
		if (_TRAINING_DEBUG) cout << "Sorting" << endl; 
		sort(candidates.begin(), candidates.end()); 
		if (_TRAINING_DEBUG) cout << "Reversing" << endl; 
		reverse(candidates.begin(), candidates.end()); 
		if (_TRAINING_DEBUG) cout << "There are " << candidates.size() << " Features in the pool." << endl; 
		if (_TRAINING_DEBUG) cout << "The best has ChiSq " << candidates.front().perf.chisq
			<< " \tPosRejects " << candidates.front().perf.pos_rejects 
			<< " \tNegRejects " << candidates.front().perf.neg_rejects  
			<< " \tTimePerPatch " << candidates.front().perf.time_per_patch 
			<< " \tCost " << featureCost(candidates.front().perf) << endl;  
		if (_TRAINING_DEBUG) cout << "The worst has ChiSq " << candidates.back().perf.chisq
			<< " \tPosRejects " << candidates.back().perf.pos_rejects 
			<< " \tNegRejects " << candidates.back().perf.neg_rejects 
			<< " \tTimePerPatch " << candidates.back().perf.time_per_patch 
			<< " \tCost " << featureCost(candidates.back().perf) << endl; 
		int newsize = keepPortion*candidates.size(); 
		newsize = newsize<1?1:newsize; 
		for (unsigned int i = newsize; i < candidates.size(); i++) {
			delete(candidates[i].feat); 
		}
		candidates.resize(newsize); 
		if (_TRAINING_DEBUG) cout << "Searching " << candidates.size() << " candidates for "
		<< roundSimilarFeatures << " better nearby features." << endl; 
		for (unsigned int i = 0; i < candidates.size() ; i++) {
			for (int j = 0; j < roundSimilarFeatures; j++) {
				if (_TRAINING_DEBUG) cout << "Getting a Similar Feature" << endl; 
				vector<Feature2*> similar = candidates[i].feat->getSimilarFeatures(1);
				if (_TRAINING_DEBUG) cout << "Got a Similar Feature" << endl; 
				compareFeaturesAndDeleteWorse(candidates[i].feat, 
											  similar[0],
											  candidates[i].perf); 
				if (similar[0] != (Feature2*)NULL) {
					cout << "Warning -- compareFeaturesAndDeleteWorse didn't set NULL to worse." << endl; 
				}
			}
		}
		roundSimilarFeatures += similarFeatures; 
		if (_TRAINING_DEBUG) cout << "There are " << candidates.size() << " Features in the pool." << endl; 
		if (_TRAINING_DEBUG) cout << "The best has ChiSq " << candidates.front().perf.chisq
			<< " \tPosRejects " << candidates.front().perf.pos_rejects 
			<< " \tNegRejects " << candidates.front().perf.neg_rejects  
			<< " \tTimePerPatch " << candidates.front().perf.time_per_patch 
			<< " \tCost " << featureCost(candidates.front().perf) << endl; 
		if (_TRAINING_DEBUG) cout << "The worst has ChiSq " << candidates.back().perf.chisq
			<< " \tPosRejects " << candidates.back().perf.pos_rejects 
			<< " \tNegRejects " << candidates.back().perf.neg_rejects 
			<< " \tTimePerPatch " << candidates.back().perf.time_per_patch 
			<< " \tCost " << featureCost(candidates.back().perf) << endl; 
	}
	return candidates[0].feat; 
}

double GentleBoostClassifier2::featureCost(const PerformanceMetrics& a) {
	//double remain = (a.total_neg-a.neg_rejects)*1.0/a.total_neg; 
	
	//double lost = (a.neg_rejects+10)*1.0/a.total_neg; 
	//double cost = 2*log(a.chisq)+log(a.time_per_patch)-log(lost); 
	
	double cost = a.chisq; 
	//double cost = 2*log(a.chisq)+log(a.time_per_patch+5e-7); 
	if (isnan(cost)) cost = INFINITY; 
	return cost; 
}

void GentleBoostClassifier2::compareFeaturesAndDeleteWorse(Feature2*& oldFeat1NewBetterFeat, 
																  Feature2*& oldFeat2NewDeletedFeat,
																  PerformanceMetrics& oldPerf1NewBetterPerf) const {
	if (_TRAINING_DEBUG) cout << "Comparing Features." << endl; 
	PerformanceMetrics perf2; 
	if (oldPerf1NewBetterPerf.chisq < 0 || oldPerf1NewBetterPerf.pos_rejects <0
		|| oldPerf1NewBetterPerf.neg_rejects < 0) {
		if (_TRAINING_DEBUG) cout << "Getting feature 1 Performance." << endl; 
		getPerformanceMeasures(oldFeat1NewBetterFeat,
							   oldPerf1NewBetterPerf); 
	}
	if (_TRAINING_DEBUG) cout << "Getting feature 2 Performance." << endl; 
	getPerformanceMeasures(oldFeat2NewDeletedFeat, perf2); 
	if (perf2 < oldPerf1NewBetterPerf) {
		if (_TRAINING_DEBUG) cout << "Feature 2 was worse -- deleting"<< endl; 
		delete(oldFeat2NewDeletedFeat); 
		oldFeat2NewDeletedFeat = NULL; 
	} else {
		if (_TRAINING_DEBUG) cout << "Feature 1 was worse -- deleting" << endl; 
		Feature2* temp = oldFeat1NewBetterFeat; 
		oldFeat1NewBetterFeat = oldFeat2NewDeletedFeat; 
		oldFeat2NewDeletedFeat = temp; 
		delete(oldFeat2NewDeletedFeat); 
		oldPerf1NewBetterPerf = perf2; 
		oldFeat2NewDeletedFeat = NULL; 
	}
	
}


void GentleBoostClassifier2::setTrainingFeatureType(const std::string &featureType) {
	if (featureType.compare("HaarFeature") == 0 || 
		featureType.compare("BoxFeature") == 0)
		featureName = featureType; 
	else 
		cout << "Warning: Requesting GentleBoost to train with unknown feature " << featureType << endl; 
}

void GentleBoostClassifier2::setUseFastPatchList(int yesorno) {
	useFast = yesorno; 
	patchList = useFast? &fpl : &pl; 
}


void GentleBoostClassifier2::setSearchParams(Size minSize, 
													 Size maxSize,	
													 double scaleInc, 
													 double stepWidth, 
													 int scaleStepWidth) {
	pl = PatchList2(basePatchSize, minSize, maxSize, scaleInc, stepWidth); 
	fpl = FastPatchList2(basePatchSize, minSize, maxSize, scaleInc, stepWidth, scaleStepWidth); 
	if (!sharingPatchList){
		setUseFastPatchList(useFast); 
	}
}


void GentleBoostClassifier2::searchCurrentImageAtScale(vector<SearchResult>& keptPatches, 
															   int scale, 
															   int NMSRadius, 
															   double threshold)  {	
	vector<Rect> blacklistPatches; 
	blacklistPatches.clear(); 
	searchCurrentImageAtScale(keptPatches, scale, NMSRadius, threshold, blacklistPatches); 
	
}

int GentleBoostClassifier2::getNumScales() const{
	if (patchList != NULL) {
		return patchList->getNumScales();
	}
	return -1; 
}


Size GentleBoostClassifier2::getSizeOfScale(int scale) const {
	if (patchList != NULL && scale >= 0 && scale < patchList->getNumScales()) {
		return patchList->getPatchSizeAtScale( scale);
	}
	return cvSize(-1,-1); 
}

void GentleBoostClassifier2::setCurrentImage(const Mat &gray_image) {
	if (features.size() == 0) {
		cout << "Warning: Must have at least one feature before supplying an image to search." << endl; 
		return; 
	}
	patchList->setImage(gray_image); 	
}

void GentleBoostClassifier2::searchCurrentImageAtScale(vector<SearchResult>& keptPatches, 
													   int scale, 
													   int NMSRadius, 
													   double threshold,
													   vector<Rect> blacklistPatches,
													   int spatialRadius, 
													   int scaleRadius)  {	
	keptPatches.clear(); 
	
	if (scale < 0 || scale >= patchList->getNumScales()) return; 
	
	
	//patchlist->setImage(gray_image); 	
	
	if (_CASCADE_DEBUG) cout << "Searching image at scale " << scale << endl;
	patchList->resetListToScale(scale);
	
	if (_CASCADE_DEBUG) cout << patchList->getCurrentListLength() << " remaining patches." << endl; 
	for (int i = 0; i < numFeatures; i++) {
		if (_CASCADE_DEBUG) cout << "Applying feature " << i << endl; 
		features[i].predictPatchList(patchList); 
		
		if (_CASCADE_DEBUG) cout << "Removing Patches" << endl; 
		patchList->accumulateAndRemovePatchesBelowThreshold(featureRejectThresholds[i]); 
		if (_CASCADE_DEBUG) cout << patchList->getCurrentListLength() << " remaining patches." << endl; 
	}
	
	if (NMSRadius > 0)	patchList->keepOnlyLocalMaxima(NMSRadius); 
	
	vector<SearchResult> scalePatches; 
	if (_CASCADE_DEBUG) cout << "Getting remaining patches." << endl; 
	patchList->getRemainingPatches(scalePatches, blacklistPatches, spatialRadius, scaleRadius);
	
	for (unsigned int i = 0; i < scalePatches.size(); i++) {
		if (scalePatches[i].value > threshold) {
			keptPatches.push_back(scalePatches[i]); 
		}
	}
	
	sort(keptPatches.begin(), keptPatches.end()); 
	reverse(keptPatches.begin(), keptPatches.end());	
	
	if (_CASCADE_DEBUG) cout << "Image search at scale " << scale << " kept " 
		<< keptPatches.size() << "/" << patchList->getTotalPatches() << "(" 
		<< 100.0*keptPatches.size()/patchList->getTotalPatches() << "%)"<< endl; 
}

void GentleBoostClassifier2::searchImage(const cv::Mat &gray_image, 
												 vector<SearchResult>& keptPatches, 
												 int NMSRadius, 
												 double threshold)  {
	vector<Rect> blacklistPatches; 
	searchImage(gray_image, keptPatches, NMSRadius, threshold, blacklistPatches); 
	
}


void GentleBoostClassifier2::suppressLocalNonMaximaAcrossScales(vector<SearchResult>& keptPatches, 
																		int NMSRadius, 
																		vector<Point>& centers, 
																		vector<size_t>& nextScaleStart) const {
	vector<int> keep(keptPatches.size(),1); 
	for (unsigned int i = 0; i < keptPatches.size(); i++) { 
		int nextScale = nextScaleStart[keptPatches[i]._scale]; 
		for (unsigned int j = nextScale; j < keptPatches.size(); j++) {
			int xdist = centers[i].x-centers[j].x; 
			int ydist = centers[i].y-centers[j].y; 
			xdist=xdist<0?-xdist:xdist; 
			ydist=ydist<0?-ydist:ydist; 
			
			if (xdist <= NMSRadius && ydist <= NMSRadius) {
				keep[i] = keep[i]&&(keptPatches[i].value >= keptPatches[j].value); 
				keep[j] = keep[j]&&(keptPatches[j].value >= keptPatches[i].value); 
			}
		}
	}
	
	int ind = 0; 
	for (unsigned int i = 0; i < keep.size(); i++) {
		if (keep[i]) {
			keptPatches[ind++] = keptPatches[i]; 
		}
	}
	keptPatches.resize(ind); 
}

void GentleBoostClassifier2::searchImage(const Mat &gray_image, 
												 vector<SearchResult>& keptPatches, 
												 int NMSRadius, 
												 double threshold,
												 vector<Rect> blacklistPatches,
												 int spatialRadius, 
												 int scaleRadius) {
	keptPatches.clear(); 
	vector<SearchResult> scalePatches; 
	vector<Point> centers; 
	vector<size_t>nextScaleStart; 
	
	setCurrentImage(gray_image); 
	for (int j = 0; j < patchList->getNumScales(); j++) {
		searchCurrentImageAtScale(scalePatches, j, NMSRadius, threshold, blacklistPatches); 
		
		for (unsigned int i = 0; i < scalePatches.size(); i++) {
			if (scalePatches[i].value > threshold) {
				keptPatches.push_back(scalePatches[i]); 
				Rect last = keptPatches.back().imageLocation; 
				centers.push_back(cvPoint(last.x+last.width/2, last.y+last.height/2)); 
			}
		}
		nextScaleStart.push_back(keptPatches.size()); 
	}
	
	
	if (NMSRadius > 0 && !disableNMSAcrossScales) {
		suppressLocalNonMaximaAcrossScales(keptPatches, NMSRadius, centers, nextScaleStart); 
	}
	
	sort(keptPatches.begin(), keptPatches.end()); 
	reverse(keptPatches.begin(), keptPatches.end()); 
	
	if (_BLACKOUT_DEBUG) cout << "Image search kept " << keptPatches.size() << "/" << patchList->getTotalPatches() <<
	"(" << 100.0*keptPatches.size()/patchList->getTotalPatches() << "%)"<< endl; 
}

void GentleBoostClassifier2::sharePatchListWithClassifier(const GentleBoostClassifier2 &otherClassifier){
	if (&otherClassifier == this) {
		sharingPatchList = 0; 
		setUseFastPatchList(useFast); 
		
	} else {
		patchList = otherClassifier.patchList; 
		sharingPatchList = 1; 
	}
}



int GentleBoostClassifier2::getNumNegPatches() const {
	return negPatches.size(); 
}


int GentleBoostClassifier2::getNumPosPatches() const {
	return posPatches.size(); 
}
 

void GentleBoostClassifier2::getImageHeaderForNegPatch(cv::Mat& dest, int patchNum) const{
	if (patchNum < 0 || patchNum >= (int)negPatches.size()) 
		cout << "WARNING: Trying to get image header for non existent patch " << patchNum << " out of " << negPatches.size() << endl; 
	else 
		negPatches[patchNum]->getImageHeader().copyTo(dest); 
}

void GentleBoostClassifier2::getImageHeaderForPosPatch(cv::Mat& dest, int patchNum) const {
	if (patchNum < 0 || patchNum >= (int)posPatches.size()) 
		cout << "WARNING: Trying to get image header for non existent patch " << patchNum << " out of " << posPatches.size() << endl; 
	else 
		posPatches[patchNum]->getImageHeader().copyTo(dest); 
}

void GentleBoostClassifier2::writeToFileStorage(cv::FileStorage &fs) const {
	fs << "numRegressorsUsed" << numFeatures 	
	<< "use_fpl" << useFast << "size_w" << basePatchSize.width 
	<< "size_h" << basePatchSize.height << "featureName" << featureName
	<< "numRegressorsTotal" << (int)features.size() << "reject_thresholds" << "["
	<< featureRejectThresholds <<"]" << "regressors" << "["; 
	for (size_t i = 0; i < features.size(); i++) {
		fs << features[i]; 
	}
	fs << "]" ;
}

void GentleBoostClassifier2::readFromFileStorage(const cv::FileNode &fs) {	
	fs["numRegressorsUsed"] >> numFeatures; 
	fs["use_fpl"]>> useFast; 
	fs["size_w"] >> basePatchSize.width;
	fs["size_h"] >> basePatchSize.height; 
	fs["featureName"] >> featureName; 
	int numRegs ; 
	fs["numRegressorsTotal"] >> numRegs; 
	fs["reject_thresholds"] >> featureRejectThresholds;
	//fs["regressors"] >> features; 
	
	FileNode tl = fs["regressors"];
	//cout << tl.type() << endl; 
	CV_Assert(tl.type() == FileNode::SEQ); 
	CV_Assert(tl.size() == (size_t)numRegs); 
	//CV_Assert(tl.type() == FileNode::SEQ && tl.size() == (size_t)numRegs);
	features.clear(); 
	for (int i = 0; i < numRegs; i++) {
		FeatureRegressor2 reg; 
		tl[i] >> reg; 
		features.push_back(reg);
	}
	
	setUseFastPatchList(useFast); 
}


FileStorage& operator << (cv::FileStorage &fs, const GentleBoostClassifier2 &booster) {
	
	if (_GENTLEBOOST_DEBUG) cout << "Saving GentleBoostClassifier" << endl; 
	fs << "{" ;
	booster.writeToFileStorage(fs); 
	fs << "}" ; 
	if (_GENTLEBOOST_DEBUG) cout << "Done Saving GentleBoostClassifier" << endl; 
	return fs; 
}

void operator >> ( const cv::FileNode &fs, GentleBoostClassifier2 &booster) {
	if (_GENTLEBOOST_DEBUG) cout << "Loading GentleBoostClassifier" << endl; 
	booster.readFromFileStorage(fs); 
	if (_GENTLEBOOST_DEBUG) cout << "Done Loading GentleBoostClassifier" << endl; 
}
