/*
 *  GentleBoostCascadedClassifier.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/26/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "GentleBoostCascadedClassifier.h"
#include <list>
#include "BlockTimer.h"
#include "DebugGlobals.h"
#include <limits.h>
#include "HaarFeature.h"
#include "NMPTUtils.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <omp.h>

using namespace cv; 
using namespace std; 

void GentleBoostCascadedClassifier::searchCurrentImageAtScale(vector<SearchResult>& keptPatches, 
															  int scale, 
															  int NMSRadius, 
															  double threshold,
															  vector<Rect> blacklistPatches,
															  int spatialRadius, 
															  int scaleRadius)  {	
	keptPatches.clear(); 
	
	if (scale < 0 || scale >= patchlist->getNumScales()) return; 
	
	
	//patchlist->setImage(gray_image); 	
	
	if (_CASCADE_DEBUG) cout << "Searching image at scale " << scale << endl;
	patchlist->resetListToScale(scale);
	
	if (_CASCADE_DEBUG) cout << patchlist->getCurrentListLength() << " remaining patches." << endl; 
	for (int i = 0; i < numFeatures; i++) {
		if (_CASCADE_DEBUG) cout << "Applying feature " << i << endl; 
		features[i]->predictPatchList(patchlist); 
		
		if (_CASCADE_DEBUG) cout << "Removing Patches" << endl; 
		patchlist->accumulateAndRemovePatchesBelowThreshold(featureRejectThresholds[i]); 
		if (_CASCADE_DEBUG) cout << patchlist->getCurrentListLength() << " remaining patches." << endl; 
	}
	
	if (NMSRadius > 0)	patchlist->keepOnlyLocalMaxima(NMSRadius); 
	
	vector<SearchResult> scalePatches; 
	if (_CASCADE_DEBUG) cout << "Getting remaining patches." << endl; 
	patchlist->getRemainingPatches(scalePatches, blacklistPatches, spatialRadius, scaleRadius);
	
	for (unsigned int i = 0; i < scalePatches.size(); i++) {
		if (scalePatches[i].value > threshold) {
			keptPatches.push_back(scalePatches[i]); 
		}
	}
	
	sort(keptPatches.begin(), keptPatches.end()); 
	reverse(keptPatches.begin(), keptPatches.end());	
	
	if (_CASCADE_DEBUG) cout << "Image search at scale " << scale << " kept " 
		<< keptPatches.size() << "/" << patchlist->getTotalPatches() << "(" 
		<< 100.0*keptPatches.size()/patchlist->getTotalPatches() << "%)"<< endl; 
}


void GentleBoostCascadedClassifier::searchCurrentImageAtScale(vector<SearchResult>& keptPatches, 
															  int scale, 
															  int NMSRadius, 
															  double threshold)  {	
	vector<Rect> blacklistPatches; 
	blacklistPatches.clear(); 
	searchCurrentImageAtScale(keptPatches, scale, NMSRadius, threshold, blacklistPatches); 
	
}

int GentleBoostCascadedClassifier::getNumScales() {
	if (patchlist != NULL) {
		return patchlist->getNumScales();
	}
	return -1; 
}


Size GentleBoostCascadedClassifier::getSizeOfScale(int scale) {
	if (patchlist != NULL && scale >= 0 && scale < patchlist->getNumScales()) {
		return patchlist->getPatchSizeAtScale( scale);
	}
	return cvSize(-1,-1); 
}

void GentleBoostCascadedClassifier::setCurrentImage(IplImage* gray_image) {
	Mat newIm = gray_image; 
	setCurrentImage(newIm); 
}

void GentleBoostCascadedClassifier::setCurrentImage(const Mat &gray_image) {
	if (features.size() == 0) {
		cout << "Warning: Must have at least one feature before supplying an image to search." << endl; 
		return; 
	}
	patchlist->setImage(gray_image); 	
}


void GentleBoostCascadedClassifier::suppressLocalNonMaximaAcrossScales(vector<SearchResult>& keptPatches, 
																	int NMSRadius, 
																	vector<Point>& centers, 
																	vector<unsigned int>& nextScaleStart) {
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

void GentleBoostCascadedClassifier::searchImage(IplImage* gray_image, 
												vector<SearchResult>& keptPatches, 
												int NMSRadius, 
												double threshold,
												vector<Rect> blacklistPatches,
												int spatialRadius, 
												int scaleRadius) {
	Mat newIm = gray_image; 
	searchImage(newIm, keptPatches, NMSRadius, threshold, blacklistPatches, spatialRadius, scaleRadius); 
}

void GentleBoostCascadedClassifier::searchImage(const Mat &gray_image, 
												vector<SearchResult>& keptPatches, 
												int NMSRadius, 
												double threshold,
												vector<Rect> blacklistPatches,
												int spatialRadius, 
												int scaleRadius) {
	keptPatches.clear(); 
	vector<SearchResult> scalePatches; 
	vector<Point> centers; 
	vector<unsigned int>nextScaleStart; 
		
	setCurrentImage(gray_image); 
//#pragma omp parallel for
	for (int j = 0; j < patchlist->getNumScales(); j++) {
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
	
	cout << "Image search kept " << keptPatches.size() << "/" << patchlist->getTotalPatches() <<
	"(" << 100.0*keptPatches.size()/patchlist->getTotalPatches() << "%)"<< endl; 
}


void GentleBoostCascadedClassifier::searchImage(IplImage* gray_image, 
												vector<SearchResult>& keptPatches, 
												int NMSRadius, 
												double threshold) {
	Mat newIm = gray_image; 
	searchImage(newIm, keptPatches, NMSRadius, threshold); 
}

void GentleBoostCascadedClassifier::searchImage(const cv::Mat &gray_image, 
												vector<SearchResult>& keptPatches, 
												int NMSRadius, 
												double threshold)  {
	vector<Rect> blacklistPatches; 
	blacklistPatches.clear(); 
	searchImage(gray_image, keptPatches, NMSRadius, threshold, blacklistPatches); 
	
}

void GentleBoostCascadedClassifier::setSearchParams(int useFast,
													Size minSize, 
													Size maxSize,	
													double scaleInc, 
													double stepWidth, 
													int scaleStepWidth) {
	if (patchlist != NULL) delete(patchlist); 
	if (useFast) {
		patchlist = new FastPatchList(minSize, maxSize, scaleInc, stepWidth, basePatchSize, scaleStepWidth); 
	} else {
		patchlist = new PatchList(minSize, maxSize, scaleInc, stepWidth, basePatchSize); 
	}
}

void GentleBoostCascadedClassifier::setBGTrainingFromImageDataset(const string &datasetFileName){
	if (negImageDataset != NULL) delete(negImageDataset); 
	negImageDataset = ImageDataSet::loadFromFile(datasetFileName); 	
} 


void GentleBoostCascadedClassifier::checkPatchListForMemoryLeaks() {
	vector<SearchResult> v; 
	while (1) {
		//unsigned int currInd = 0; 
		int imNum = 0; 
		//int patchNum = 0; 
		while (imNum < negImageDataset->getNumEntries()) {
			currentBGFileNum = (currentBGFileNum+(1<<19)-1)%negImageDataset->getNumEntries(); 
			cout << "Searching image " << negImageDataset->getFileName(currentBGFileNum) << endl; ; 
			IplImage* im = cvLoadImage( negImageDataset->getFileName(currentBGFileNum).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
			//IplImage* gray_image = cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_8U, 1);
			//searchImage(gray_image, v, 15, -INFINITY); 
			//cvReleaseImage(&gray_image); 
			cvReleaseImage(&im); 
			imNum++; 
		}
	}
}


int GentleBoostCascadedClassifier::updateSurvivalStats() {
	posSurvived.clear(); 
	negSurvived.clear(); 
	double maxNeg=-INFINITY, maxPos=-INFINITY, minNeg=INFINITY, minPos=INFINITY; 
	int negDead = 0; 
	for (unsigned int i = 0; i < trainingPatches.size(); i++) {
		double value = cvGetReal2D(trainingFeatureSum, i, 0); 
		if (cvGetReal2D(trainingLabels, i, 0) == 1) {
			minPos=minPos>value?value:minPos; 
			maxPos=maxPos<value?value:maxPos; 
			if (cvGetReal2D(trainingSurvived, i, 0) == 0) posSurvived.push_back(0); 
			else posSurvived.push_back(1); 
		} else {
			minNeg=minNeg>value?value:minNeg; 
			maxNeg=maxNeg<value?value:maxNeg; 
			if (cvGetReal2D(trainingSurvived, i, 0) == 0) {negSurvived.push_back(0); negDead++;}
			else {negSurvived.push_back(1); } 
		}
	}
	
	cout << negDead << " bg patches didn't survive and need replacing." << endl; 
	cout << "Max Pos: " << maxPos << " ; Min Pos: " << minPos << "\t;\t Max Neg: "
	<< maxNeg << " ; Min Neg: " << minNeg << endl; 
	
	return negDead; 
}


void GentleBoostCascadedClassifier::setHardNegativeTrainingExamplesFromBGImages(){
	if ((negImageDataset == NULL || negImageDataset->getNumEntries() == 0) && posPatchDataset == NULL) {
		if (_CASCADE_DEBUG) cout << "Warning: Must set a bg image dataset before calling setHardNegativeExamples" << endl; 
		return; 
	} else if (numFeatures == 0) {
		if (_CASCADE_DEBUG) cout << "Skipping hard BG patch search because there are no features." << endl; 
		return; 
	}
	
	int negDead = updateSurvivalStats();
	
	int minForSearch = 1; 
	if (negDead < minForSearch) {
		if (_CASCADE_DEBUG) cout << "Not enough patches need replacing, so don't bother for this round." << endl; 
		return; 
	}
	
	unsigned int currInd = 0; 
	int imNum = 0; 
	int patchNum = 0; 
	int numReplaced = 0; 
	int usePosImagesForNegPatches = negImageDataset == NULL; 
	int numFiles = usePosImagesForNegPatches ? posPatchDataset->getNumUniquePositiveImages() : negImageDataset->getNumEntries();
	
	while (currInd < negPatches.size()) {
		while (currInd < negPatches.size() && negSurvived[currInd] && keepNonRejectedBGPatches) currInd++; //Skip survived patches in update		
		if (currInd == negPatches.size()) break; //Accidentally moved off the edge of the world
		
		if (_CASCADE_DEBUG) cout << "Filling index " << currInd << " / " << negPatches.size() << endl; 
		currentBGFileNum = (currentBGFileNum+(1<<19)-1)%numFiles; 
		if (_CASCADE_DEBUG) cout << "Using Image Number " << currentBGFileNum << " of " << numFiles << endl; 
		string fname = usePosImagesForNegPatches ? posPatchDataset->getUniquePosImageName(currentBGFileNum).c_str() : negImageDataset->getFileName(currentBGFileNum); 
		cout << "Searching image " << fname << endl; 
		Mat im = imread(fname, 0); //CV_LOAD_IMAGE_GRAYSCALE);
		
		vector<Rect> blackoutList; 
		blackoutList.clear(); 
		if (usePosImagesForNegPatches)
			blackoutList = posPatchDataset->getObjectLocationsInPosImage(currentBGFileNum); 
		
		//IplImage* gray_image = cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_8U, 1); 
		//cvCvtColor(im, gray_image, CV_RGB2GRAY); 
		vector<SearchResult> keptPatches; 
		disableNMSAcrossScales = 1; 
		if (useNMSInTraining) 
			searchImage(im, keptPatches, basePatchSize.width/2, -INFINITY, blackoutList, basePatchSize.width*.5,2 );
		else
			searchImage(im, keptPatches, 0, -INFINITY, blackoutList,basePatchSize.width*.5,2); 
		disableNMSAcrossScales = 0; 
		patchNum += keptPatches.size(); 
		long long seed = 0; 
		for (unsigned int i = 0; i < maxPatchesPerImage && i < keptPatches.size() && currInd < negPatches.size(); i++) {
			if (useNMSInTraining )
				seed = i; 
			else
				seed = (seed+1<<31-1)%keptPatches.size(); 
			
			Mat p(negPatches[currInd]->getImageSize(), CV_8U,1); 
			
			patchlist->fillImageWithPixelsOfSearchPatch(p, keptPatches[seed]); 
			
			if (!(negSurvived[currInd] && keepNonRejectedBGPatches)) {
				negPatches[currInd]->setImage(p);
				numReplaced++; 
				currInd++; 
				while (currInd < negPatches.size() && negSurvived[currInd] && keepNonRejectedBGPatches) currInd++; 
				
				if (_CASCADE_DEBUG) cout << "Filling index " << currInd << " / " << negPatches.size() << endl; 
				if (currInd == negPatches.size()) break; 
			} else {
				currInd++; 
			}
			
			//cvReleaseImage(&p); 
			//cvResetImageROI(gray_image); 
		}
		//cvReleaseImage(&gray_image); 
		//cvReleaseImage(&im); 
		imNum++; 
		if (imNum >= numFiles) {
			cout << "Warning: Couldn't find enough hard bg patches in image set." << endl; 
			break; 
		}
	}
	if (currInd < negPatches.size()) {
		cout << patchNum << " patches remain and " << negPatches.size() << " were needed." << endl; 
		if (patchNum >= (int) negPatches.size()) {
			maxPatchesPerImage = maxPatchesPerImage*2; 
			useNMSInTraining = 0; 
			cout << "Increasing patches per image to " << maxPatchesPerImage << " and searching again." << endl; 
			setHardNegativeTrainingExamplesFromBGImages(); 
		} else {
			runningOutOfNegPatches = 1; 
			maxPatchesPerImage = negPatches.size(); 
			if (patchNum == 0)	ranOutOfNegPatches = 1; 
		}
		
	}
	
	//cout << "Computing new training weights" << endl; 
	setTrainingSet(trainingPatches, trainingLabels); 
	
	cout << "Replaced a total of " << numReplaced << " bg patches." << endl; 
} 


GentleBoostCascadedClassifier::GentleBoostCascadedClassifier() : GentleBoostClassifier() {
	
	
	trainingSurvived = cvCreateMat(1, 1, CV_8UC1); 
	testingSurvived= cvCreateMat(1, 1, CV_8UC1); 
	featureRejectThresholds = vector<double>(1,0); 
	featureRejectThresholds.clear(); 
	
	maxPosRejectsPerRound = .001; 
	desiredNegRejectsPerRound = 1; 
	
	patchlist = NULL; 
	setSearchParams(); 
	
	posImageDataset = NULL; 
	negImageDataset = NULL; 
	posPatchDataset = NULL; 
	
	currentBGFileNum = 1; 
	
	dontTrainRejected = 1; 
	keepNonRejectedBGPatches = 1; 
	maxPatchesPerImage = 50; 
	
	runningOutOfNegPatches = 0; 
	ranOutOfNegPatches = 0; 
	useNMSInTraining = 1; 
	disableNMSAcrossScales = 0; 
	
	sharingPatchList = 0; 
}


GentleBoostCascadedClassifier::~GentleBoostCascadedClassifier() {
	cvReleaseMat(&trainingSurvived); 
	cvReleaseMat(&testingSurvived); 
	if (posImageDataset != NULL) delete(posImageDataset); 
	if (negImageDataset != NULL) delete(negImageDataset); 
	if (!sharingPatchList) delete(patchlist); 
}


void GentleBoostCascadedClassifier::sharePatchListWithClassifier(GentleBoostCascadedClassifier* otherClassifier){
	if (otherClassifier == this) return; 
	delete(patchlist); 
	patchlist = otherClassifier->patchlist; 
}

void GentleBoostCascadedClassifier::searchPatches(const vector<ImagePatch*>& patches,
												  const CvMat* labels,
												  CvMat*& survived,
												  CvMat*& featureSum,
												  CvMat*& posterior,
												  CvMat*& predictions,
												  vector<CvMat*>& featureOutputs,
												  double& perf,
												  CvMat*& weights) {
	{
		
		int numPatches = patches.size(); 
		int releaseFeatureSum = featureSum==NULL; 
		int releaseSurvived = survived==NULL; 
		if (_CASCADE_DEBUG) cout << "Making data for searchPatches()" << endl; 
		
		if (survived == NULL || survived->rows != numPatches || survived->cols != 1 || survived->type != CV_8UC1) {
			cvReleaseMat(&survived); 
			survived = cvCreateMat(numPatches, 1, CV_8UC1); 
		}
		
		if (featureSum == NULL || featureSum->rows != numPatches || featureSum->cols != 1) {
			cvReleaseMat(&featureSum); 
			featureSum = cvCreateMat(numPatches, 1, CV_64FC1); 
		}
		if (posterior != NULL && (posterior->rows != numPatches || posterior->cols != 1)) {
			cvReleaseMat(&posterior); 
			posterior = cvCreateMat(numPatches, 1, CV_64FC1); 
		}
		if (predictions != NULL && (predictions->rows != numPatches || predictions->cols != 1)) {
			cvReleaseMat(&predictions); 
			predictions = cvCreateMat(numPatches, 1, CV_64FC1); 
		}
		if (weights != NULL && (weights->rows != numPatches || weights->cols != 1)) {
			cvReleaseMat(&weights); 
			weights = cvCreateMat(numPatches, 1, CV_64FC1); 
		}
		
		
		//cout << "Resetting values for case with no features." << endl; 
		
		for (unsigned int i = 0; i < featureOutputs.size(); i++)  cvReleaseMat(&featureOutputs[i]); 
		featureOutputs.clear(); 
		featureOutputs.resize(numFeatures, (CvMat*)NULL); 
		
		cvSetZero(featureSum); 
		cvSet(survived, cvRealScalar(0xff)); 
		if (weights != NULL) cvSet(weights, cvRealScalar(1.0/numPatches)); 
		
		
		CvMat* outputs = NULL; 
		CvMat* cmp = cvCreateMat(numPatches, 1, CV_8UC1); 
		
		if (_CASCADE_DEBUG) cout << "Setting values for features." << endl; 
		
		for (int i = 0; i < numFeatures; i++) {
			Mat out; 
			CvMat outmat; 
			features[i]->predict(patches, out);
			outmat = out; 
			outputs = &outmat; 
			
			featureOutputs[i] = cvCloneMat(outputs); 
			
			cvAdd(featureSum, outputs, featureSum, NULL); 
			cvCmpS(featureSum, featureRejectThresholds[i], cmp, CV_CMP_GE); 
			cvCopy(cmp, survived, survived); 
			
			cvNot(survived, cmp); 
			
			
			int numSurvived, numDied ;
			numSurvived = cvCountNonZero(survived); 
			numDied = cvCountNonZero(cmp); 
			if (numSurvived+numDied !=numPatches)
				cout << "Sanity check failed -- numSurvived+numDied != total)" << endl; 
			
			if (weights != NULL && labels != NULL) {
				//weights = weights.*exp(-featureOutputs(:,i).*labels);
				//weights = weights./sum(weights);
				cvMul(featureSum, labels, weights,-1); 
				//Here we are scaling the log weight so that its max is an arbitrary
				//constant, e.g. 50, so that things should be numerically stable, 
				//even for large datasets. The larger this constant is, the more 
				//precise the very small weights, but the larger risk we run of
				//numerical instability. 
				double minval,maxval; 
				cvMinMaxLoc(weights,&minval,&maxval); 
				cvSubS(weights, cvRealScalar(maxval-50), weights); 
				cvExp(weights,weights); 
				/*
				 cvMul(outputs, labels, outputs); 
				 cvSubRS(outputs, cvRealScalar(-1), outputs); 
				 cvExp(outputs, outputs); 
				 cvMul(weights, outputs, weights); 
				 */
				if (dontTrainRejected) {
					cvSub(weights, weights, weights, cmp); 
					int nzw = cvCountNonZero(weights); 
					if (nzw < numSurvived)
						cout << "Sanity check failed -- non-zero weights < numSurvived" << endl ; 
				}
				
				cvNormalize(weights, weights, 1, 0, CV_L1); 
			}
		}
		
		if (_CASCADE_DEBUG) cout << "Clearing intermediate values." << endl; 
		//cvReleaseMat(&outputs); 
		cvReleaseMat(&cmp); 
		
		if (_CASCADE_DEBUG) cout << "Setting Posterior." << endl; 
		
		if (posterior != NULL) {
			//posterior(survived)= 1./(1+exp(-2.*featureSum(survived)));
			cvScale(featureSum, posterior, -2); 
			cvExp(posterior, posterior); 
			cvAddS(posterior, cvRealScalar(1), posterior); 
			cvDiv(NULL,posterior, posterior); 
		}
		
		
		if (_CASCADE_DEBUG) cout << "Setting Predictions." << endl; 
		
		perf = 0; 
		if (predictions != NULL) {
			for (int i = 0; i < featureSum->height; i++) {
				if (cvGetReal2D(featureSum, i, 0) > .5 && cvGetReal2D(survived,i,0)) cvSetReal2D(predictions,i,0,1); 
				else cvSetReal2D(predictions,i,0,-1); 
			}
		}
		
		if (_CASCADE_DEBUG) cout << "Setting Performance." << endl; 
		
		if (labels != NULL) {
			perf = 0; 
			for (int i = 0; i < featureSum->height; i++)
				if (cvGetReal2D(featureSum, i, 0) > .5 && cvGetReal2D(labels, i, 0) == 1)
					perf = perf+1; 
			perf = perf / numPatches; 
		}
		
		
		if (_CASCADE_DEBUG) cout << "Clearing uncared for values" << endl; 
		
		if (releaseFeatureSum) cvReleaseMat(&featureSum); 
		if (releaseSurvived) cvReleaseMat(&survived); 
		
		if (_CASCADE_DEBUG) cout << "Done with searchPatches" << endl; 
	}
	
	
}

void GentleBoostCascadedClassifier::searchPatches(const std::vector<ImagePatch*>& patches,
												  const cv::Mat &labels,
												  cv::Mat &survived,
												  cv::Mat &featureSum,
												  cv::Mat &posterior,
												  cv::Mat &predictions,
												  std::vector<cv::Mat> &featureOutputs,
												  double& perf,
												  cv::Mat &weights) {
	
	int numPatches = patches.size(); 
	//int releaseFeatureSum = featureSum==NULL; 
	//int releaseSurvived = survived==NULL; 
	
	if (_CASCADE_DEBUG) cout << "Making data for searchPatches()" << endl; 
	
	survived.create(numPatches, 1, CV_8U); 
	featureSum.create(numPatches, 1, CV_64FC1); 
	posterior.create(numPatches, 1, CV_64FC1); 
	predictions.create(numPatches, 1, CV_64FC1); 
	weights.create(numPatches, 1, CV_64FC1); 
		
	//cout << "Resetting values for case with no features." << endl; 
	
	featureOutputs.clear(); 
	featureOutputs.resize(numFeatures); 
	
	featureSum = 0.; 
	survived = 0xff; 
	
	weights = (1.0/numPatches); 
	
	
	Mat outputs; 
	Mat cmp(numPatches, 1, CV_8UC1); 
	
	if (_CASCADE_DEBUG) cout << "Setting values for features." << endl; 
	
	for (int i = 0; i < numFeatures; i++) {
		features[i]->predict(patches, outputs); 
		featureOutputs[i] = outputs.clone(); 
		
		featureSum += outputs; 
		
		compare(featureSum, featureRejectThresholds[i], cmp, CMP_GE); 
		//cmp.copyTo(survived, survived); 
		//cvCopy(cmp, survived, survived); 
		bitwise_and(cmp, survived, survived); 
		bitwise_not(survived, cmp); 
		//cvNot(survived, cmp); 
		
		
		int numSurvived, numDied ;
		numSurvived = countNonZero(survived); 
		numDied = countNonZero(cmp); 
		if (numSurvived+numDied !=numPatches)
			cout << "Sanity check failed -- numSurvived+numDied != total)" << endl; 
		
		//if (weights != NULL && labels != NULL) {
		weights = -featureSum*labels; 
		//cvMul(featureSum, labels, weights,-1); 
		//Here we are scaling the log weight so that its max is an arbitrary
		//constant, e.g. 50, so that things should be numerically stable, 
		//even for large datasets. The larger this constant is, the more 
		//precise the very small weights, but the larger risk we run of
		//numerical instability. 
		double minval,maxval; 
		minMaxLoc(weights,&minval,&maxval); 
		weights = weights - (maxval-50); 
		//cvSubS(weights, cvRealScalar(maxval-50), weights); 
		exp(weights,weights); 
		/*
		 cvMul(outputs, labels, outputs); 
		 cvSubRS(outputs, cvRealScalar(-1), outputs); 
		 cvExp(outputs, outputs); 
		 cvMul(weights, outputs, weights); 
		 */
		if (dontTrainRejected) {
			subtract(weights, weights, weights, cmp); 
			//cvSub(weights, weights, weights, cmp); 
			int nzw = countNonZero(weights); 
			if (nzw < numSurvived)
				cout << "Sanity check failed -- non-zero weights < numSurvived" << endl ; 
		}
		
		normalize(weights, weights, 1, 0, NORM_L1); 
		//}
	}
	
	//if (_CASCADE_DEBUG) cout << "Clearing intermediate values." << endl; 
	//cvReleaseMat(&outputs); 
	//cvReleaseMat(&cmp); 
	
	if (_CASCADE_DEBUG) cout << "Setting Posterior." << endl; 
	
	//if (posterior != NULL) {
		//posterior(survived)= 1./(1+exp(-2.*featureSum(survived)));
	exp(featureSum*(-2), posterior); 
	posterior = 1/(posterior+1); 
	//posterior = 1/(cv::exp(featureSum*-2)+1); 
	
	//	cvScale(featureSum, posterior, -2); 
	//	cvExp(posterior, posterior); 
	//	cvAddS(posterior, cvRealScalar(1), posterior); 
	//	cvDiv(NULL,posterior, posterior); 
	//}
	
	
	if (_CASCADE_DEBUG) cout << "Setting Predictions." << endl; 
	
	perf = 0; 
	//if (predictions != NULL) {
		for (int i = 0; i < featureSum.rows; i++) {
			if (featureSum.at<double>(i, 0) > .5 && survived.at<uint8_t>(i,0)) predictions.at<double>(i,0)= 1; 
			else  predictions.at<double>(i,0)= -1; 
		}
	//}
	
	if (_CASCADE_DEBUG) cout << "Setting Performance." << endl; 
	
	//if (labels != NULL) {
		perf = 0; 
		for (int i = 0; i < featureSum.rows; i++)
			if (featureSum.at<double>(i, 0) > .5 && labels.at<double>(i, 0) == 1)
				perf = perf+1; 
		perf = perf / numPatches; 
	//}
	
	
	//if (_CASCADE_DEBUG) cout << "Clearing uncared for values" << endl; 
	
	//if (releaseFeatureSum) cvReleaseMat(&featureSum); 
	//if (releaseSurvived) cvReleaseMat(&survived); 
	
	if (_CASCADE_DEBUG) cout << "Done with searchPatches" << endl; 
}


void GentleBoostCascadedClassifier::setTrainingSet(string pathToPatchDatasetFile) {
	if (_CASCADE_DEBUG) cout << "Reading dataset " << pathToPatchDatasetFile << endl; 
	if (posPatchDataset != NULL) delete(posPatchDataset); 
	
	posPatchDataset = PatchDataset::readFromFile(pathToPatchDatasetFile);
	
	vector<ImagePatch*> patches = posPatchDataset->getPatches();
	Mat labels;
	posPatchDataset->getLabels(labels); 
	
	setTrainingSet(patches, labels); 
	
	if (negImageDataset != NULL) delete(negImageDataset); 
	negImageDataset = posPatchDataset->getNegImagesDataset(); 	
	
	if (_CASCADE_DEBUG) cout << "negImageDataset has address " << (void*)negImageDataset << endl; 
}

void GentleBoostCascadedClassifier::setTestingSet(string pathToPatchDatasetFile) {
	PatchDataset* dataset = PatchDataset::readFromFile(pathToPatchDatasetFile.c_str());
	vector<ImagePatch*> patches = dataset->getPatches();
	Mat labels; 
	dataset->getLabels(labels); 
	setTestingSet(patches, labels); 	
	delete(dataset); 
}


void GentleBoostCascadedClassifier::setTrainingSet(const vector<ImagePatch*>& trainingPatches,
												   const CvMat* trainingLabels){
	Mat labels = trainingLabels; 
	setTrainingSet(trainingPatches, labels); 
}

void GentleBoostCascadedClassifier::setTrainingSet(const vector<ImagePatch*>& trainingPatches,
												   const Mat &labels){
	CvMat tmp=labels;
	CvMat* trainingLabels = &tmp;
	
	if (&this->trainingPatches != &trainingPatches) {
		this->trainingPatches.clear(); 
		this->trainingPatches.resize(trainingPatches.size(), (ImagePatch*)NULL); 
		for (unsigned int i = 0; i < trainingPatches.size(); i++)
			this->trainingPatches[i] = trainingPatches[i]; 
		negPatches.clear(); 
		posPatches.clear(); 
		for (unsigned int i = 0; i < this->trainingPatches.size(); i++) {
			if (cvGetReal2D(trainingLabels,i,0) == 1)
				posPatches.push_back(this->trainingPatches[i]); 
			else
				negPatches.push_back(this->trainingPatches[i]); 
		}
	}
	if (this->trainingLabels != trainingLabels) {
		cvReleaseMat(&this->trainingLabels); 
		this->trainingLabels = cvCloneMat(trainingLabels); 
	}
	if (getNumFeaturesTotal() < 1) {		
		basePatchSize = trainingPatches[0]->getImageSize();
		if (_CASCADE_DEBUG) cout << "basePatchSize inferred from data set is " << basePatchSize.width << "x" << basePatchSize.height << endl; 
	}
	searchPatches(this->trainingPatches,
				  this->trainingLabels,
				  trainingSurvived,
				  trainingFeatureSum,
				  trainingPosteriors,
				  trainingPredictions,
				  trainingFeatureOutputs,
				  trainingPerformance,
				  trainingWeights); 
	if (_CASCADE_DEBUG) cout << "There are " << posPatches.size() 
		<< " pos patches and " << negPatches.size() << " neg patches." << endl;  
} 

void GentleBoostCascadedClassifier::printState() {
	cout << "GentleBoostCascadedClassifier has values: " << endl; 
	cout << "--trainingLabels: " << endl; 
	Mat m = this->trainingLabels;
	NMPTUtils::printMat(m.t()); 
	cout << "--trainingSurvived: " << endl; 
	m = this->trainingSurvived;
	NMPTUtils::printMat(m.t()); 
	cout << "--trainingFeatureSum: " << endl; 
	m = this->trainingFeatureSum;
	NMPTUtils::printMat(m.t()); 
	cout << "--trainingPosteriors: " << endl; 
	m = this->trainingPosteriors;
	NMPTUtils::printMat(m.t()); 
	cout << "--trainingPredictions: " << endl; 
	m = this->trainingPredictions;
	NMPTUtils::printMat(m.t()); 
	cout << "--trainingWeights: " << endl; 
	m = this->trainingWeights;
	NMPTUtils::printMat(m.t()); 
	
}

void GentleBoostCascadedClassifier::setTestingSet(const vector<ImagePatch*>& testingPatches,
												  const CvMat* testingLabels){
	Mat labels = testingLabels; 
	setTestingSet(testingPatches, labels); 
}

void GentleBoostCascadedClassifier::setTestingSet(const vector<ImagePatch*>& testingPatches,
												  const Mat &labels){
	CvMat tmp = labels; 
	CvMat* testingLabels = &tmp;
	
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
				  testingSurvived,
				  testingFeatureSum,
				  testingPosteriors,
				  testingPredictions,
				  testingFeatureOutputs,
				  testingPerformance,
				  w); 
} 


void GentleBoostCascadedClassifier::pickRejectThreshold(const CvMat* values,
														const CvMat* labels,
														const CvMat* survived,
														double& threshold,
														int& totalPosRejects,
														int& totalNegRejects){
	vector<double>posVals; 
	vector<double>negVals; 
	//list<double> posVals; 
	//list<double> negVals; 
	int numNegRejects = 0; 
	int numPosRejects = 0; 
	int nneg = 0; 
	int npos = 0; 
	for (int i = 0; i < values->rows; i++) {
		
		double v = cvGetReal2D(values, i, 0); 
		double l = cvGetReal2D(labels, i, 0); 
		int s = cvGetReal2D(survived, i, 0); 
		if (s==0 && l == 1)
			numPosRejects++; 
		else if (s == 0 && l == -1)
			numNegRejects++; 
		else if (l == 1) {
			posVals.push_back(v); 
			npos++; 
		}
		else {
			negVals.push_back(v); 
			nneg++; 
		}
	}
	
	
	if (posVals.empty() && negVals.empty()) {
		totalPosRejects = numPosRejects;
		totalNegRejects = numNegRejects; 
		threshold = -INFINITY; 
		return; 
	}
	if (posVals.empty()) {
		sort(negVals.begin(), negVals.end()); 
		totalPosRejects = numPosRejects; 
		totalNegRejects = numNegRejects+nneg; 
		threshold = negVals.back()+.00001;
		return; 
	} 
	if (negVals.empty()) {		
		sort(posVals.begin(), posVals.end()); 
		totalPosRejects = numPosRejects; 
		totalNegRejects = numNegRejects; 
		threshold = posVals.front(); 
		return; 
	}
	
	sort(posVals.begin(), posVals.end()); 
	sort(negVals.begin(), negVals.end()); 
	
	int maxPosRejcts = (int)(maxPosRejectsPerRound*npos);
	int negRejectTarget = (int)( desiredNegRejectsPerRound*nneg); 
	
	if (_CASCADE_DEBUG) cout << "npos: " << npos << " ; maxPosRejects " << maxPosRejcts << " nneg: " << nneg
		<< " ; negRejectTarget " << negRejectTarget << endl;
	
	int newPosRejects = 0; 
	int newNegRejects = 0; 
	
	int done = 0; 
	unsigned int posind = 0; 
	unsigned int negind = 0; 
	
	while (posind < posVals.size() && !done) {
		threshold = posVals[posind]; //get the next positive value
		//find out which is the first negative value more than this positive 
		//value.
		while (negind < negVals.size() && negVals[negind] < threshold) negind++;
		newPosRejects = posind; 
		newNegRejects = negind; 
		done = (newPosRejects >= maxPosRejcts || newNegRejects >= negRejectTarget); 
		
		posind++; 
	}
	
	//split the difference -- put a buffer between the accepted region and
	//the rejected region
	
	if (newNegRejects == 0) threshold = -INFINITY; 
	else threshold = (threshold+negVals[newNegRejects-1])/2.0; 
	
	/*
	 while (!posVals.empty()) {
	 threshold = posVals.front();
	 posVals.pop_front();
	 while (!negVals.empty()) {
	 double neg = negVals.front(); 
	 if (neg >=threshold)
	 break; 
	 negVals.pop_front(); 
	 newNegRejects++; 
	 }
	 if (newPosRejects >= maxPosRejcts || newNegRejects >= negRejectTarget) break; 
	 newPosRejects++; 
	 }*/
	
	totalNegRejects = numNegRejects+newNegRejects; 
	totalPosRejects = numPosRejects+newPosRejects; 
	
}


void GentleBoostCascadedClassifier::addFeatureBoosted(Feature* nextFeature, int boostRounds){
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
		patchlist->setBasePatchSize(basePatchSize); 
	}
	
	
	CvMat* output = NULL;
	Mat out; 
	CvMat outmat; 
	
	vector<FeatureRegressor*> reg; 
	//cout << "a" << endl; 
	CvMat* newSum = cvCloneMat(trainingFeatureSum);
	CvMat* newWts = cvCloneMat(trainingWeights); 
	
	if (_CASCADE_DEBUG) cout << "Training RBF" << endl;
	for (int i = 0; i < boostRounds; i++) {
		//cout << "b" << endl; 
		reg.push_back( new FeatureRegressor(nextFeature)); 
		Mat tl = trainingLabels; 
		Mat nw = newWts; 
		reg[i]->train(numBins, trainingPatches, tl, nw); 
		reg[i]->predict(trainingPatches, out); 
		outmat = out; 
		output = &outmat; 
		//cout << "c" << endl; 
		cvAdd(newSum, output, newSum, NULL);
		cvMul(output, trainingLabels, output); 
		cvSubRS(output, cvRealScalar(-1), output); 
		cvExp(output, output); 
		cvMul(newWts, output, newWts); 
		cvNormalize(newWts, newWts, 1, 0, CV_L1); 
		//cout << "d" << endl; 
	}
	for (int i = 1; i < boostRounds; i++) {
		//cout << "e" << endl; 
		reg[0]->combineLUTs(reg[i]); 
		delete(reg[i]); 
	}
	
	
	
	if (_CASCADE_DEBUG) cout << "Adding regressor to list of features" << endl; 
	
	features.push_back(reg[0]); 
	numFeatures++; 
	
	if (_CASCADE_DEBUG) cout << "New Number of Features is " << numFeatures << "; features.size() is " << features.size() << endl; 
	
	if (_CASCADE_DEBUG) cout << "Adding predictions to trainingFeatureOutputs" << endl; 
	reg[0]->predict(trainingPatches, out); 
	outmat = out; 
	output = &outmat; 
	trainingFeatureOutputs.push_back(cvCloneMat(output)); 
	
	if (_CASCADE_DEBUG) cout << "Updating trainingFeatureSum" << endl; 
	cvAdd(trainingFeatureSum, output, trainingFeatureSum, NULL); 
	
	
	
	double newthresh; 
	int	 negRejects, posRejects; 
	
	pickRejectThreshold(trainingFeatureSum, trainingLabels, trainingSurvived, 
						newthresh, posRejects, negRejects); 
	
	cout << "After adding Feature " << numFeatures << " the reject threshold was " << newthresh <<
	" meaning " << posRejects << " Positive Training Examples and " << negRejects << 
	" negative training examples were rejected." << endl; 
	
	featureRejectThresholds.push_back(newthresh); 
	
	CvMat* cmp = cvCreateMat(trainingLabels->rows, 1, CV_8UC1); 
	cvCmpS(trainingFeatureSum, featureRejectThresholds[numFeatures-1], cmp, CV_CMP_GE); 
	cvCopy(cmp, trainingSurvived, trainingSurvived); 
	cvNot(trainingSurvived, cmp); 
	int numSurvived, numDied ;
	numSurvived = cvCountNonZero(trainingSurvived); 
	numDied = cvCountNonZero(cmp); 
	if (numSurvived+numDied !=trainingFeatureSum->rows)
		cout << "Sanity check failed -- numSurvived+numDied != total)" << endl; 
	
	if (_CASCADE_DEBUG) cout << "Updating Training Weights" << endl; 
	
	//log_weights = featureSum.*labels.*-1
	cvMul(trainingFeatureSum, trainingLabels, trainingWeights,-1); 
	//Here we are scaling the log weight so that its max is an arbitrary
	//constant, e.g. 50, so that things should be numerically stable, 
	//even for large datasets. The larger this constant is, the more 
	//precise the very small weights, but the larger risk we run of
	//numerical instability. 
	double minval,maxval; 
	cvMinMaxLoc(trainingWeights,&minval,&maxval); 
	cvSubS(trainingWeights, cvRealScalar(maxval-50), trainingWeights); 
	cvExp(trainingWeights,trainingWeights); 
	/*
	 cvMul(output, trainingLabels, output); 
	 cvSubRS(output, cvRealScalar(-1), output); 
	 cvExp(output, output); 
	 cvMul(trainingWeights, output, trainingWeights); 
	 */
	if (dontTrainRejected) {
		cvSub(trainingWeights, trainingWeights, trainingWeights, cmp); 
		int nzw = cvCountNonZero(trainingWeights);
		if (nzw < numSurvived)
			cout << "Sanity check failed -- non-zero weights < numSurvived" << endl ; 
	}
	cvNormalize(trainingWeights, trainingWeights, 1, 0, CV_L1); 
	
	if (_CASCADE_DEBUG) cout << "Updating training posteriors" << endl; 
	cvScale(trainingFeatureSum, trainingPosteriors, -2); 
	cvExp(trainingPosteriors, trainingPosteriors); 
	cvAddS(trainingPosteriors, cvRealScalar(1), trainingPosteriors); 
	cvDiv(NULL, trainingPosteriors, trainingPosteriors); 
	
	if (_CASCADE_DEBUG) cout << "Updating training predictions" << endl; 
	for (int i = 0; i < trainingFeatureSum->height; i++) {
		if (cvGetReal2D(trainingFeatureSum, i, 0) > .5 && cvGetReal2D(trainingSurvived, i, 0)) 
			cvSetReal2D(trainingPredictions,i,0,1); 
		else cvSetReal2D(trainingPredictions,i,0,-1); 
	}
	
	
	if (_CASCADE_DEBUG) cout << "Updating training performance." << endl; 
	trainingPerformance = 0; 
	for (int i = 0; i < trainingFeatureSum->height; i++)
		if (cvGetReal2D(trainingFeatureSum, i, 0) > .5 && cvGetReal2D(trainingLabels, i, 0) == 1)
			trainingPerformance = trainingPerformance+1; 
	trainingPerformance = trainingPerformance / trainingFeatureSum->height; 
	
	if (_CASCADE_DEBUG) cout << "New Performance: " << trainingPerformance << endl; 
	
	if (_CASCADE_DEBUG) cout << "Done with training things." << endl; 
	
	if (testingLabels != NULL && (unsigned int) testingLabels->rows != testingPatches.size()) {
		
		if (_CASCADE_DEBUG) cout << "Starting testing things." << endl; 
		reg[0]->predict(testingPatches, out);		
		outmat = out; 
		output = &outmat; 
		testingFeatureOutputs.push_back(cvCloneMat(output)); 
		cvAdd(testingFeatureSum, output, testingFeatureSum, testingSurvived); 
		
		
		cvReleaseMat(&cmp); 
		CvMat* cmp = cvCreateMat(testingLabels->rows, 1, CV_8UC1); 
		
		cvCmpS(testingFeatureSum, featureRejectThresholds[numFeatures-1], cmp, CV_CMP_GE); 
		cvCopy(cmp, testingSurvived, testingSurvived); 
		cvNot(testingSurvived, cmp); 
		int numSurvived, numDied ;
		numSurvived = cvCountNonZero(testingSurvived); 
		numDied = cvCountNonZero(cmp); 
		if (numSurvived+numDied !=testingFeatureSum->rows)
			cout << "Sanity check failed -- numSurvived+numDied != total)" << endl; 
		
		
		cvScale(testingFeatureSum, testingPosteriors, -2); 
		cvExp(testingPosteriors, testingPosteriors); 
		cvAddS(testingPosteriors, cvRealScalar(1), testingPosteriors); 
		cvDiv(NULL, testingPosteriors, testingPosteriors); 
		
		for (int i = 0; i < testingFeatureSum->height; i++) {
			if (cvGetReal2D(testingFeatureSum, i, 0) > .5 && cvGetReal2D(testingSurvived,i,0)) 
				cvSetReal2D(testingPredictions,i,0,1); 
			else cvSetReal2D(testingPredictions,i,0,-1); 
		}
		
		testingPerformance = 0; 
		for (int i = 0; i < testingFeatureSum->height; i++)
			if (cvGetReal2D(testingFeatureSum, i, 0) > .5 && cvGetReal2D(testingLabels, i, 0) == 1)
				testingPerformance = testingPerformance+1; 
		testingPerformance = testingPerformance / testingFeatureSum->height; 
	}
	
	cvReleaseMat(&cmp); 
} 


void GentleBoostCascadedClassifier::addFeature(Feature* nextFeature){
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
		patchlist->setBasePatchSize(basePatchSize); 
	}
	
	
	CvMat* output = NULL;
	Mat out; 
	CvMat outmat; 
	
	if (_CASCADE_DEBUG) cout << "Training RBF" << endl; 
	FeatureRegressor* reg = new FeatureRegressor(nextFeature); 
	Mat tl = trainingLabels; 
	Mat tw = trainingWeights; 
	reg->train(numBins, trainingPatches, tl, tw); 
	reg->predict(trainingPatches, out); 
	outmat = out; 
	output = &outmat; 
	
	
	if (_CASCADE_DEBUG) cout << "Adding regressor to list of features" << endl; 
	
	features.push_back(reg); 
	numFeatures++; 
	
	if (_CASCADE_DEBUG) cout << "New Number of Features is " << numFeatures << "; features.size() is " << features.size() << endl; 
	
	if (_CASCADE_DEBUG) cout << "Adding predictions to trainingFeatureOutputs" << endl; 
	trainingFeatureOutputs.push_back(cvCloneMat(output)); 
	
	if (_CASCADE_DEBUG) cout << "Updating trainingFeatureSum" << endl; 
	cvAdd(trainingFeatureSum, output, trainingFeatureSum, trainingSurvived); 
	
	double newthresh; 
	int	 negRejects, posRejects; 
	
	pickRejectThreshold(trainingFeatureSum, trainingLabels, trainingSurvived, 
						newthresh, posRejects, negRejects); 
	
	cout << "After adding Feature " << numFeatures << " the reject threshold was " << newthresh <<
	" meaning " << posRejects << " Positive Training Examples and " << negRejects << 
	" negative training examples were rejected." << endl; 
	
	featureRejectThresholds.push_back(newthresh); 
	
	CvMat* cmp = cvCreateMat(trainingLabels->rows, 1, CV_8UC1); 
	cvCmpS(trainingFeatureSum, featureRejectThresholds[numFeatures-1], cmp, CV_CMP_GE); 
	cvCopy(cmp, trainingSurvived, trainingSurvived); 
	cvNot(trainingSurvived, cmp); 
	int numSurvived, numDied ;
	numSurvived = cvCountNonZero(trainingSurvived); 
	numDied = cvCountNonZero(cmp); 
	if (numSurvived+numDied !=trainingFeatureSum->rows)
		cout << "Sanity check failed -- numSurvived+numDied != total)" << endl; 
	
	if (_CASCADE_DEBUG) cout << "Updating Training Weights" << endl; 
	cvMul(output, trainingLabels, output); 
	cvSubRS(output, cvRealScalar(-1), output); 
	cvExp(output, output); 
	cvMul(trainingWeights, output, trainingWeights); 
	
	if (dontTrainRejected) {
		cvSub(trainingWeights, trainingWeights, trainingWeights, cmp); 
		int nzw = cvCountNonZero(trainingWeights);
		if (nzw < numSurvived)
			cout << "Sanity check failed -- non-zero weights < numSurvived" << endl ; 
	}
	cvNormalize(trainingWeights, trainingWeights, 1, 0, CV_L1); 
	
	if (_CASCADE_DEBUG) cout << "Updating training posteriors" << endl; 
	cvScale(trainingFeatureSum, trainingPosteriors, -2); 
	cvExp(trainingPosteriors, trainingPosteriors); 
	cvAddS(trainingPosteriors, cvRealScalar(1), trainingPosteriors); 
	cvDiv(NULL, trainingPosteriors, trainingPosteriors); 
	
	if (_CASCADE_DEBUG) cout << "Updating training predictions" << endl; 
	for (int i = 0; i < trainingFeatureSum->height; i++) {
		if (cvGetReal2D(trainingFeatureSum, i, 0) > .5 && cvGetReal2D(trainingSurvived, i, 0)) 
			cvSetReal2D(trainingPredictions,i,0,1); 
		else cvSetReal2D(trainingPredictions,i,0,-1); 
	}
	
	
	if (_CASCADE_DEBUG) cout << "Updating training performance." << endl; 
	trainingPerformance = 0; 
	for (int i = 0; i < trainingFeatureSum->height; i++)
		if (cvGetReal2D(trainingFeatureSum, i, 0) > .5 && cvGetReal2D(trainingLabels, i, 0) == 1)
			trainingPerformance = trainingPerformance+1; 
	trainingPerformance = trainingPerformance / trainingFeatureSum->height; 
	
	if (_CASCADE_DEBUG) cout << "New Performance: " << trainingPerformance << endl; 
	
	if (_CASCADE_DEBUG) cout << "Done with training things." << endl; 
	
	if (testingLabels != NULL && (unsigned int) testingLabels->rows != testingPatches.size()) {
		
		if (_CASCADE_DEBUG) cout << "Starting testing things." << endl; 
		reg->predict(testingPatches, out); 
		outmat = out; 
		output = &outmat; 
		testingFeatureOutputs.push_back(cvCloneMat(output)); 
		cvAdd(testingFeatureSum, output, testingFeatureSum, testingSurvived); 
		
		
		cvReleaseMat(&cmp); 
		CvMat* cmp = cvCreateMat(testingLabels->rows, 1, CV_8UC1); 
		
		cvCmpS(testingFeatureSum, featureRejectThresholds[numFeatures-1], cmp, CV_CMP_GE); 
		cvCopy(cmp, testingSurvived, testingSurvived); 
		cvNot(testingSurvived, cmp); 
		int numSurvived, numDied ;
		numSurvived = cvCountNonZero(testingSurvived); 
		numDied = cvCountNonZero(cmp); 
		if (numSurvived+numDied !=testingFeatureSum->rows)
			cout << "Sanity check failed -- numSurvived+numDied != total)" << endl; 
		
		
		cvScale(testingFeatureSum, testingPosteriors, -2); 
		cvExp(testingPosteriors, testingPosteriors); 
		cvAddS(testingPosteriors, cvRealScalar(1), testingPosteriors); 
		cvDiv(NULL, testingPosteriors, testingPosteriors); 
		
		for (int i = 0; i < testingFeatureSum->height; i++) {
			if (cvGetReal2D(testingFeatureSum, i, 0) > .5 && cvGetReal2D(testingSurvived,i,0)) 
				cvSetReal2D(testingPredictions,i,0,1); 
			else cvSetReal2D(testingPredictions,i,0,-1); 
		}
		
		testingPerformance = 0; 
		for (int i = 0; i < testingFeatureSum->height; i++)
			if (cvGetReal2D(testingFeatureSum, i, 0) > .5 && cvGetReal2D(testingLabels, i, 0) == 1)
				testingPerformance = testingPerformance+1; 
		testingPerformance = testingPerformance / testingFeatureSum->height; 
	}
	
	cvReleaseMat(&cmp); 
} 


int GentleBoostCascadedClassifier::getNumNegPatches() {
	return negPatches.size(); 
}


int GentleBoostCascadedClassifier::getNumPosPatches() {
	return posPatches.size(); 
}


void GentleBoostCascadedClassifier::getImageHeaderForNegPatch(cv::Mat& dest, int patchNum){
	if (patchNum < 0 || patchNum >= (int)negPatches.size()) 
		cout << "WARNING: Trying to get image header for non existent patch " << patchNum << " out of " << negPatches.size() << endl; 
	else 
		negPatches[patchNum]->getImageHeader(dest); 
}

IplImage* GentleBoostCascadedClassifier::getImageHeaderForNegPatch(int patchNum){
	if (patchNum < 0 || patchNum >= (int)negPatches.size()) return NULL; 
		
	Mat imgData; 
	getImageHeaderForNegPatch(imgData,patchNum); 
	IplImage* retval = cvCreateImageHeader( imgData.size(), IPL_DEPTH_8U, 1 ); 
	cvSetData(retval, imgData.data, imgData.step); 
	return retval; 
}

void GentleBoostCascadedClassifier::getImageHeaderForPosPatch(cv::Mat& dest, int patchNum) {
	if (patchNum < 0 || patchNum >= (int)posPatches.size()) 
		cout << "WARNING: Trying to get image header for non existent patch " << patchNum << " out of " << posPatches.size() << endl; 
	else 
		posPatches[patchNum]->getImageHeader(dest); 
}

IplImage* GentleBoostCascadedClassifier::getImageHeaderForPosPatch(int patchNum){
	if (patchNum < 0 || patchNum >= (int)posPatches.size()) return NULL; 
	
	Mat imgData; 
	getImageHeaderForPosPatch(imgData,patchNum); 
	IplImage* retval = cvCreateImageHeader( imgData.size(), IPL_DEPTH_8U, 1 ); 
	cvSetData(retval, imgData.data, imgData.step); 
	return retval; 
}



void GentleBoostCascadedClassifier::getPerformanceMeasures(Feature* candidate, 
														   double& chiSq, 
														   int& posRejects, 
														   int& negRejects) {
	//Get Performance has three modes depending on the candidate feature: 
	//--If the feature is NULL, calculates the current chi-squared error
	//--If the feature is currently in the classifier, calculates the error if
	//  we remove the feature
	//--Otherwise, calculates the error if we were to add this feature.
	
	if (trainingLabels == NULL) {
		cout << "Warning: Must set training patches & labels before calling getPerformanceMeasures" << endl; 
		return; 
	}
	
	Mat out; 
	CvMat outmat; 
	CvMat* output = NULL; 
	CvMat* den = NULL; 
	CvMat* num = NULL; 
	
	if (candidate == NULL) { //Case: Current performance
		if (_CASCADE_DEBUG) cout << "Testing current performance." << endl; 
		output = cvCloneMat(trainingFeatureSum); 
	} else {
		int ind = -1; 
		if (_CASCADE_DEBUG) cout << "Checking if duplicate feature." << endl; 
		for (int i = 0; i < numFeatures; i++) {
			if (candidate == features[i]->getFeature()) {
				ind = i; 
				break; 
			}
		}
		
		if (ind > -1) { //Case: Performance without this feature.
			if (_CASCADE_DEBUG) cout << "Found duplicate feature at " << ind << "." << endl; 
			output = cvCloneMat(trainingFeatureSum); 
			cvSub(output, trainingFeatureOutputs[ind], output); 
		} else {
			if (_CASCADE_DEBUG) cout << "Training feature to see how it would do." << endl; 
			FeatureRegressor* reg = new FeatureRegressor(candidate); 
			Mat tl = trainingLabels, tw=trainingWeights; 
			reg->train(numBins, trainingPatches, tl, tw); 
			reg->predict(trainingPatches, out); 
			outmat = out; 
			output = &outmat; 
			
			cvAdd(trainingFeatureSum, output, output, trainingSurvived); 
			
			delete(reg); 
		}
	}
	
	double threshold; 
	
	if (_CASCADE_DEBUG) cout << "Starting pickRejectThreshold" << endl; 
	if (candidate == NULL) {
		posRejects = 0; 
		negRejects = 0; 
		for (int i = 0; i < trainingLabels->height; i++) {
			if (cvGetReal2D(trainingLabels, i, 0) == 1 &&
				cvGetReal2D(trainingSurvived,i,0) == 0)
				posRejects++; 
			else if (cvGetReal2D(trainingLabels,i,0)==-1 &&
					 cvGetReal2D(trainingSurvived,i,0)==0)
				negRejects++; 
		}
		threshold = featureRejectThresholds[numFeatures-1]; 
	} else {
		pickRejectThreshold(output, trainingLabels, trainingSurvived, 
							threshold, posRejects, negRejects); 
	}
	if (_CASCADE_DEBUG) cout << "Done picking reject threshold." << endl; 
	
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
	//cvReleaseMat(&output); 
} 


bool GentleBoostCascadedClassifier::exhaustedAllNegPatches() {
	return ranOutOfNegPatches; 
}


void GentleBoostCascadedClassifier::setTrainingParams(double maxPosRejects, double desiredNegRejects) {
	maxPosRejectsPerRound = maxPosRejects; 
	desiredNegRejectsPerRound = desiredNegRejects; 
}


void GentleBoostCascadedClassifier::getPerformanceMeasures(const Feature* candidate, PerformanceMetrics& perf){
	//Get Performance has three modes depending on the candidate feature: 
	//--If the feature is NULL, calculates the current chi-squared error
	//--If the feature is currently in the classifier, calculates the error if
	//  we remove the feature
	//--Otherwise, calculates the error if we were to add this feature.
	
	perf.chisq = -1; 
	perf.pos_rejects = -1; 
	perf.neg_rejects = -1; 
	perf.total_pos = -1; 
	perf.total_neg = -1; 
	perf.prev_pos_rejects = -1; 
	perf.prev_neg_rejects = -1; 
	perf.time_per_patch = -1; 
	perf.threshold = -1; 
	
	
	if (trainingLabels == NULL) {
		cout << "Warning: Must set training patches & labels before calling getPerformanceMeasures" << endl; 
		return; 
	}
	
	BlockTimer bt(1); 
	perf.time_per_patch = 0; 
	perf.total_pos = posPatches.size();
	perf.total_neg = negPatches.size(); 
	
	Mat out; 
	CvMat outmat; 
	CvMat* output = NULL; 
	CvMat* den = NULL; 
	CvMat* num = NULL; 
	
	if (candidate == (Feature*)NULL) { //Case: Current performance
		if (_CASCADE_DEBUG) cout << "Testing current performance." << endl; 
		output = cvCloneMat(trainingFeatureSum); 
	} else {
		int ind = -1; 
		if (_CASCADE_DEBUG) cout << "Checking if duplicate feature." << endl; 
		for (int i = 0; i < numFeatures; i++) {
			if (candidate == features[i]->getFeature()) {
				ind = i; 
				break; 
			}
		}
		
		if (ind > -1) { //Case: Performance without this feature.
			if (_CASCADE_DEBUG) cout << "Found duplicate feature at " << ind << "." << endl; 
			output = cvCloneMat(trainingFeatureSum); 
			cvSub(output, trainingFeatureOutputs[ind], output); 
		} else {
			if (_CASCADE_DEBUG) cout << "Training feature to see how it would do." << endl; 
			FeatureRegressor* reg = new FeatureRegressor(candidate); 
			if (_CASCADE_DEBUG) cout << "Train" << endl; 
			
			Mat tl = trainingLabels, tw=trainingWeights; 
			reg->train(numBins, trainingPatches, tl, tw); 
			
			if (reg->getLUTRange() <= 0) {
				perf.chisq = INFINITY; 
				
				perf.pos_rejects = INT_MAX; 
				perf.neg_rejects = 0; 
				perf.time_per_patch = INFINITY; 
				perf.threshold = -INFINITY; 
				delete(reg); 
				//cvReleaseMat(&output); 
				cvReleaseMat(&num); 
				cvReleaseMat(&den);  
				return; 
			}
			if (_CASCADE_DEBUG) cout << "Predict" << endl; 
			if (_CASCADE_DEBUG) cout << "Training patches has size " << trainingPatches.size() << "; output has address " << (void*)output << endl; 
			bt.blockStart(0); 
			reg->predict(trainingPatches, out); 
			outmat = out; 
			output = &outmat; 
			
			bt.blockStop(0); 
			if (_CASCADE_DEBUG) cout << "Training patches has size " << trainingPatches.size() << "; output has size " << output->width << "x" << output->height << endl; 
			perf.time_per_patch = bt.getTotTime(0); 
			if (_CASCADE_DEBUG) cout << "Add" << endl; 
			cvAdd(trainingFeatureSum, output, output, trainingSurvived); 
			
			if (_CASCADE_DEBUG) {
				Mat m = output; 
				cout << "This feature had outputs: " << endl; 
				NMPTUtils::printMat(m.t()); 
			}
			
			delete(reg); 
		}
	}
	
	if (_CASCADE_DEBUG) cout << "Starting pickRejectThreshold" << endl; 
	
	perf.prev_pos_rejects = 0; 
	perf.prev_neg_rejects = 0; 
	for (int i = 0; i < trainingLabels->height; i++) {
		if (cvGetReal2D(trainingLabels, i, 0) == 1 &&
			cvGetReal2D(trainingSurvived,i,0) == 0)
			perf.prev_pos_rejects++; 
		else if (cvGetReal2D(trainingLabels,i,0)==-1 &&
				 cvGetReal2D(trainingSurvived,i,0)==0)
			perf.prev_neg_rejects++; 
	}
	perf.threshold = featureRejectThresholds[numFeatures-1]; 
	perf.pos_rejects = perf.prev_pos_rejects; 
	perf.neg_rejects = perf.prev_neg_rejects; 
	perf.time_per_patch = perf.time_per_patch*1.0/(perf.total_pos-perf.prev_pos_rejects+perf.total_neg-perf.prev_neg_rejects); 
	
	
	if (candidate != NULL)  {
		pickRejectThreshold(output, trainingLabels, trainingSurvived, 
							perf.threshold, perf.pos_rejects, perf.neg_rejects); 
	}
	if (_CASCADE_DEBUG) cout << "Done picking reject threshold." << endl; 
	
	cvScale(output, output, -2); 
	cvExp(output, output); 
	cvAddS(output, cvRealScalar(1), output); 
	cvDiv(NULL,output, output); 
	
	
	if (_CASCADE_DEBUG) cout << "Turned output into probability." << endl; 
	
	den = cvCloneMat(output); 
	cvSubRS(den, cvRealScalar(1), den); 
	cvMul(den, output, den); 
	cvPow(den, den, -.5); 
	
	if (_CASCADE_DEBUG) cout << "Got chisq den." << endl; 
	
	num = cvCloneMat(trainingLabels); 
	cvScale(num, num, .5, .5); 
	cvAbsDiff(num, output, num); 
	cvMul(num, den, num); 
	CvScalar sum = cvSum(num); 
	
	
	if (_CASCADE_DEBUG) cout << "Got sum of all chisq." << endl; 
	if (_CASCADE_DEBUG) cout << sum.val[0] << endl; 
	
	perf.chisq = sum.val[0]; 
	
	
	if (_CASCADE_DEBUG) cout << "set perf.chisq" << endl; 
	
	if (isnan(perf.chisq)) perf.chisq = INFINITY; 
	
	
	if (_CASCADE_DEBUG) cout << "Checked NAN, perf.chisq is " << perf.chisq << endl; 
	
	cvReleaseMat(&num); 
	if (_CASCADE_DEBUG) cout << "Released num" << endl; 
	cvReleaseMat(&den); 
	if (_CASCADE_DEBUG) cout << "Released den" << endl; 
	//cvReleaseMat(&output); 
	if (_CASCADE_DEBUG) cout << "Released output" << endl; 
}


IplImage* GentleBoostCascadedClassifier::getProbabilityMap(PatchList* patches) {
	
	Size s = patches->getImageSizeAtScale();
	Mat m ; 
	getProbabilityMap(m, patches); 
	IplImage im = m; 
	return cvCloneImage(&im); 
	/*
	for (int i = 0; i < numFeatures; i++) {
		features[i]->predictPatchList(patches); 
		patches->accumulateAndRemovePatchesBelowThreshold(featureRejectThresholds[i]); 
	}
	if (_CASCADE_DEBUG) cout << "Getting probability map -- just accumulated all feature values." << endl; 
	Size s = patches->getImageSizeAtScale();
	IplImage* probMap = cvCreateImage(s, IPL_DEPTH_64F, 1);
	
	Mat m ; 
	patches->getAccumImage(m); 
	IplImage accumulatorImage; 
	cvSetImageROI(&accumulatorImage, cvRect(0,0,s.width,s.height)); 
	cvCopy(&accumulatorImage, probMap); 
	cvResetImageROI(&accumulatorImage); 
	cvScale(probMap, probMap, -2); 
	cvExp(probMap, probMap); 
	cvAddS(probMap, cvRealScalar(1), probMap); 
	cvDiv(NULL, probMap, probMap); 
	return probMap; 
	 */
}

void GentleBoostCascadedClassifier::getProbabilityMap(Mat &dest, PatchList* patches) {
	for (int i = 0; i < numFeatures; i++) {
		features[i]->predictPatchList(patches); 
		patches->accumulateAndRemovePatchesBelowThreshold(featureRejectThresholds[i]); 
	}
	if (_CASCADE_DEBUG) cout << "Getting probability map -- just accumulated all feature values." << endl; 
	Size s = patches->getImageSizeAtScale();
	
	dest.create(s,CV_64F);
	
	Mat m; 
	patches->getAccumImage(m); 
	m(Rect(0,0,s.width,s.height)).copyTo(dest); 
	exp(dest*-2,dest); 
	dest = 1.0/(dest+1.); 	
}

void GentleBoostCascadedClassifier::readFromStream(istream& in) {
	if (_CASCADE_DEBUG) cout << "Reading booster from stream" << endl; 
	GentleBoostClassifier::readFromStream(in); 
	if (_CASCADE_DEBUG) cout << "Reading Thresholds from Stream" << endl; 
	in >> maxPosRejectsPerRound >> desiredNegRejectsPerRound; 
	featureRejectThresholds.resize(numFeatures,0); 
	for (int i = 0; i < numFeatures; i++) {
		in >> featureRejectThresholds[i]; 
	}
	if ((int) features.size() > 0) {		
		if (_CASCADE_DEBUG) cout << "GentleBoostCascadedClassifier is setting its patchlist's patchsize to "
			<< basePatchSize.width << "x" << basePatchSize.height << endl; 
		patchlist->setBasePatchSize(basePatchSize); 
		//checkPatchListSize(); 
	}
}



void GentleBoostCascadedClassifier::addToStream(ostream& out) {
	//cout << "Hey" << endl; 
	if (_CASCADE_DEBUG) cout << "Cascaded classifier addToStream" << endl; 
	GentleBoostClassifier::addToStream(out); 
	//cout << "Here" << endl; 
	
	if (_CASCADE_DEBUG) cout << "Cascaded classifier thresholds" << endl; 
	out << maxPosRejectsPerRound << " " << desiredNegRejectsPerRound << endl; 
	for (int i =0; i < numFeatures; i++) {
		out << featureRejectThresholds[i] << " " << endl; 
	}
}


ostream& operator<< (ostream& ofs, GentleBoostCascadedClassifier* model) {
	if (_CASCADE_DEBUG) cout << "Cascaded classifier ostream" << endl; 
	model->addToStream(ofs); 
	return ofs; 
}


istream& operator>> (istream& ifs, GentleBoostCascadedClassifier*& model) {
	if (_CASCADE_DEBUG) cout << "Cascaded classifier istream" << endl; 
	model->readFromStream(ifs); 
	return ifs; 
}




double GentleBoostCascadedClassifier::featureCost(const PerformanceMetrics& a) {
	//double remain = (a.total_neg-a.neg_rejects)*1.0/a.total_neg; 
	
	double lost = (a.neg_rejects+10)*1.0/a.total_neg; 
	double cost = 2*log(a.chisq)+log(a.time_per_patch)-log(lost); 
	
	//double cost = a.chisq; 
	//double cost = 2*log(a.chisq)+log(a.time_per_patch+5e-7); 
	if (isnan(cost)) cost = INFINITY; 
	return cost; 
}




void GentleBoostCascadedClassifier::compareFeaturesAndDeleteWorse(Feature*& oldFeat1NewBetterFeat, 
								   Feature*& oldFeat2NewDeletedFeat,
								   PerformanceMetrics& oldPerf1NewBetterPerf) {
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
		Feature* temp = oldFeat1NewBetterFeat; 
		oldFeat1NewBetterFeat = oldFeat2NewDeletedFeat; 
		oldFeat2NewDeletedFeat = temp; 
		delete(oldFeat2NewDeletedFeat); 
		oldPerf1NewBetterPerf = perf2; 
		oldFeat2NewDeletedFeat = NULL; 
	}
	
}


PerformanceMetrics GentleBoostCascadedClassifier::trainOneRound(int patience, int boostRounds) {
	
	if (_TRAINING_DEBUG) cout << "Adding feature " << getNumFeaturesTotal()+1 << endl; 
	if (_TRAINING_DEBUG) cout << "Replacing easy background examples with harder ones." << endl; 
	
	setHardNegativeTrainingExamplesFromBGImages(); 
	
	Feature* feat = getGoodFeature(patience); 
	if (_TRAINING_DEBUG) cout << "Done with Feature Tournament." << endl; 
	
	if (_TRAINING_DEBUG) cout<< "Adding feature to classifier." << endl; 
	if (boostRounds > 0)
		addFeatureBoosted(feat, boostRounds );
	else 
		addFeature(feat); 
	delete(feat); 
	
	PerformanceMetrics perf; 
	//booster->getPerformanceMeasures(NULL, chisq, posRej, negRej); 
	getPerformanceMeasures(NULL, perf); 
	if (_TRAINING_DEBUG) cout << "Chi-Sq: " << perf.chisq  << " ; Pos Rejects: " << perf.pos_rejects 
	<< " ; Neg Rejects: " << perf.neg_rejects << endl; 	
	return perf;  
}

Feature* GentleBoostCascadedClassifier::getGoodFeature(int patience, string featureType) {
	if (patience < 1) patience = 1; 
	int featurePoolSize = 20*patience;//200; 
	double reduceFraction = .05; 
	int numSimilar = 10*patience;//50;//10; 
	
	return getGoodFeatureViaTournament(featureType, featurePoolSize, numSimilar, reduceFraction); 
}

Feature* GentleBoostCascadedClassifier::getGoodFeatureViaTournament(string featureType,
																	int startPoolSize, 
																	int similarFeatures,
																	double keepPortion) {	
	
	vector<FeaturePerformance> candidates; 
	if (_TRAINING_DEBUG) cout << "Getting Initial Pool & performance." << endl; 
	for (int i = 0; i < startPoolSize; i++) {
		if (_TRAINING_DEBUG) cout << "Getting feature " << i <<  " with size " << basePatchSize.width << "x" << basePatchSize.height << endl; 
		FeaturePerformance f; 
		f.feat = Feature::getFeatureOfType(featureType.c_str(), basePatchSize); //new HaarFeature(basePatchSize);
		
		if (_TRAINING_DEBUG) cout << "Checking Performance of feature " << i << endl; 
		getPerformanceMeasures(f.feat,f.perf);
		candidates.push_back(f); 
	}
	
	if (_TRAINING_DEBUG) cout << "Finished filling pool" << endl; 
	if (_TRAINING_DEBUG) cout << "Size is " << candidates.size() << endl; 
	
	int roundSimilarFeatures = similarFeatures; 
	while(candidates.size() > 1) {
		vector<double> test; 
		/*
		 for(int i = 0; i < (int)candidates.size(); i++) {
		 cout << "Candidate " << i << "'s cost is " << cost(candidates[i].perf) << "(" << (bool)(cost(candidates[i].perf)<=INFINITY) <<")" << endl ; 
		 test.push_back(cost(candidates[i].perf)); 
		 }
		 cout << "Sorting test vector of doubles." << endl; 
		 sort(test.begin(), test.end()); 
		 cout << "Reversing test vector of doubles." << endl; 
		 reverse(test.begin(), test.end()); 
		 
		 cout << "Done sorting test in descending order." << endl; 
		 */
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
		cout << "Searching " << candidates.size() << " candidates for "
		<< roundSimilarFeatures << " better nearby features." << endl; 
		for (unsigned int i = 0; i < candidates.size() ; i++) {
			for (int j = 0; j < roundSimilarFeatures; j++) {
				if (_TRAINING_DEBUG) cout << "Getting a Similar Feature" << endl; 
				vector<Feature*> similar = candidates[i].feat->getSimilarFeatures(1);
				if (_TRAINING_DEBUG) cout << "Got a Similar Feature" << endl; 
				compareFeaturesAndDeleteWorse(candidates[i].feat, 
											  similar[0],
											  candidates[i].perf); 
				if (similar[0] != (Feature*)NULL) {
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



