/*
 *  FeatureRegressor.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/8/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include <math.h>
#include "FeatureRegressor.h"
#include "DebugGlobals.h"
#include "NMPTUtils.h"
#include <opencv2/core/core.hpp>

using namespace cv;  
using namespace std; 

double FeatureRegressor::TAU = .05; 
double FeatureRegressor::EPS = .001; 

FeatureRegressor::FeatureRegressor() {
	patchFeature = NULL; 
}

FeatureRegressor::FeatureRegressor(const Feature* feature) {
	patchFeature = NULL; 
	setFeature(feature); 
}

FeatureRegressor & FeatureRegressor::operator=(const FeatureRegressor &rhs) {
	if (this != &rhs) {
		setFeature(rhs.patchFeature); 
		lookUpTableMin = rhs.lookUpTableMin; 
		lookUpTableMax = rhs.lookUpTableMax; 
		lookUpTable = rhs.lookUpTable.clone(); 
	}
	return *this; 
}

FeatureRegressor::FeatureRegressor(const FeatureRegressor &rhs) {
	if (this != &rhs) {
		patchFeature = NULL; 
		setFeature(rhs.patchFeature); 
		lookUpTableMin = rhs.lookUpTableMin; 
		lookUpTableMax = rhs.lookUpTableMax; 
		lookUpTable = rhs.lookUpTable.clone(); 
	} 
}

void FeatureRegressor::setFeature(const Feature* feature) {
	if (patchFeature != NULL) delete (patchFeature); 
	patchFeature = feature->copy(); 
	if (_REGRESSOR_DEBUG) {
		cout << feature->debugInfo(); 
	}
	lookUpTableMin = 0; 
	lookUpTableMax = 0; 
	lookUpTable = Mat(); 
}

FeatureRegressor::~FeatureRegressor() {	
	if (patchFeature != NULL) 
		delete(patchFeature); 
}

Feature* FeatureRegressor::getFeature() const {
	return patchFeature; 
}

cv::Size FeatureRegressor::getFeaturePatchSize() const {
	return patchFeature->getPatchSize(); 
}

double FeatureRegressor::getLUTRange() const {
	return lookUpTableMax-lookUpTableMin; 
}

void FeatureRegressor::train(int numTableElements, const vector<ImagePatch*> &data, const CvMat* labels, const CvMat* dataWeights) {
	cv::Mat newLabels = labels; 
	cv::Mat newDataWeights = dataWeights; 
	train(numTableElements, data, newLabels, newDataWeights); 
}

void FeatureRegressor::train(int numTableElements, const vector<ImagePatch> &data, const cv::Mat &labels, const cv::Mat &dataWeights) {
	
	/* input: NxM, N data points, M Dims*/	 
	
	if (_REGRESSOR_DEBUG) cout << "Getting scalar values for all patches." << endl; 
	
	Mat vals ; 
	patchFeature->evaluateImagePatches(data, vals); 
	
	minMaxLoc(vals, &lookUpTableMin, &lookUpTableMax); 
	
	if (_REGRESSOR_DEBUG) cout << "Look Up Table Min: " << lookUpTableMin << " ; Max: " << lookUpTableMax << endl; 
	
	if (lookUpTableMax - lookUpTableMin <= 0) {
		if (_REGRESSOR_DEBUG) cout << "Set LUT to NULL" << endl; 
		lookUpTable = Mat(); 
		return; 
	}
	
	Mat bins(numTableElements, 1, CV_64F); 
	
	for (int i = 0; i < numTableElements; i++) {
		bins.at<double>( i, 0) = i*1.0/(numTableElements-1)*(lookUpTableMax - lookUpTableMin)+lookUpTableMin; 
	}
	
	if (_REGRESSOR_DEBUG) cout << "Filling lookup table with RBF values." << endl; 
	
	lookUpTable = NMPTUtils::RBF(vals, labels, dataWeights, bins, 
					  (lookUpTableMax-lookUpTableMin)*TAU, EPS).clone();
	
	lookUpTable.setTo(-1.,lookUpTable < -1); 
	lookUpTable.setTo(1.,lookUpTable > 1); 
	
	if (_REGRESSOR_DEBUG) {
		cout << "Look Up Table vals: " << endl; 
		NMPTUtils::printMat(lookUpTable.t()); 
	}
	if (_REGRESSOR_DEBUG) cout << "Finished FeatureRegressor train." << endl; 
	
	
}

void FeatureRegressor::train(int numTableElements, const vector<ImagePatch*> &data, const cv::Mat &labels, const cv::Mat &dataWeights) {
	
	/* input: NxM, N data points, M Dims*/	 
	
	if (_REGRESSOR_DEBUG) cout << "Getting scalar values for all patches." << endl; 
	
	Mat vals ; 
	patchFeature->evaluateImagePatches(data, vals); 
	
	minMaxLoc(vals, &lookUpTableMin, &lookUpTableMax); 
	
	if (_REGRESSOR_DEBUG) cout << "Look Up Table Min: " << lookUpTableMin << " ; Max: " << lookUpTableMax << endl; 
	
	if (lookUpTableMax - lookUpTableMin <= 0) {
		if (_REGRESSOR_DEBUG) cout << "Set LUT to NULL" << endl; 
		lookUpTable = Mat(); 
		return; 
	}
	
	Mat bins(numTableElements, 1, CV_64F); 
	
	for (int i = 0; i < numTableElements; i++) {
		bins.at<double>( i, 0) = i*1.0/(numTableElements-1)*(lookUpTableMax - lookUpTableMin)+lookUpTableMin; 
	}
	
	if (_REGRESSOR_DEBUG) cout << "Filling lookup table with RBF values." << endl; 
	
	lookUpTable = NMPTUtils::RBF(vals, labels, dataWeights, bins, 
								 (lookUpTableMax-lookUpTableMin)*TAU, EPS).clone();
	
	lookUpTable.setTo(-1.,lookUpTable < -1); 
	lookUpTable.setTo(1.,lookUpTable > 1); 
	
	if (_REGRESSOR_DEBUG) {
		cout << "Look Up Table vals: " << endl; 
		NMPTUtils::printMat(lookUpTable.t()); 
	}
	if (_REGRESSOR_DEBUG) cout << "Finished FeatureRegressor train." << endl; 
	
	
}

void FeatureRegressor::combineLUTs(const FeatureRegressor *other) {
	lookUpTable += other->lookUpTable; 
}

void FeatureRegressor::combineLUTs(const FeatureRegressor &other) {
	lookUpTable += other.lookUpTable; 
}

void FeatureRegressor::predict(const vector<ImagePatch> &patches, vector<double> &scalarVals) const {	
	scalarVals.resize(patches.size()); 
	
	if (_REGRESSOR_DEBUG) cout << "Regressor is predicting Patch List, vector vals" << endl; 
	if (lookUpTableMax-lookUpTableMin <= 0) {
		for (unsigned int i = 0; i < scalarVals.size(); i++) {
			scalarVals[i] = 0; 
		}
		return; 
	}
	
	if (_REGRESSOR_DEBUG) cout << "Got feature output, doing table lookup" << endl; 
	patchFeature->evaluateImagePatches(patches, scalarVals); 
	double scale = lookUpTable.rows*1.0/(lookUpTableMax-lookUpTableMin); 
	if (_REGRESSOR_DEBUG) cout << "Getting table entries" << endl; 
	for (size_t i = 0; i < scalarVals.size(); i++) {
		int ind = (int)((scalarVals[i]-lookUpTableMin)*scale);
		ind = (ind<0)?0:ind;
		ind = (ind>lookUpTable.rows-1)?lookUpTable.rows-1:ind; 
		scalarVals[i] = lookUpTable.at<double>(ind,0); 
	}
}

void FeatureRegressor::predict(const vector<ImagePatch*> &patches, vector<double>& scalarVals) const {
	scalarVals.resize(patches.size()); 
	
	if (_REGRESSOR_DEBUG) cout << "Regressor is predicting Patch List, vector vals" << endl; 
	if (lookUpTableMax-lookUpTableMin <= 0) {
		for (unsigned int i = 0; i < scalarVals.size(); i++) {
			scalarVals[i] = 0; 
		}
		return; 
	}
	
	if (_REGRESSOR_DEBUG) cout << "Got feature output, doing table lookup" << endl; 
	patchFeature->evaluateImagePatches(patches, scalarVals); 
	double scale = lookUpTable.rows*1.0/(lookUpTableMax-lookUpTableMin); 
	if (_REGRESSOR_DEBUG) cout << "Getting table entries" << endl; 
	for (size_t i = 0; i < scalarVals.size(); i++) {
		int ind = (int)((scalarVals[i]-lookUpTableMin)*scale);
		ind = (ind<0)?0:ind;
		ind = (ind>lookUpTable.rows-1)?lookUpTable.rows-1:ind; 
		scalarVals[i] = lookUpTable.at<double>(ind,0); 
	}
}

void FeatureRegressor::predict(const vector<ImagePatch> patches, Mat &scalarVals) const {
	if (_REGRESSOR_DEBUG) cout << "Regressor is predicting Patch List, matrix vals" << endl; 
	patchFeature->evaluateImagePatches(patches, scalarVals); //We probably shouldn't do this before the next check, but we actually want to take extra time.
	
	if (_REGRESSOR_DEBUG) { 
		Mat m = scalarVals; 
		cout << "Feature outputs: " << endl; 
		NMPTUtils::printMat(m.t()); 
		
	}
	
	if (_REGRESSOR_DEBUG) cout << "Lookup table has range " << (lookUpTableMax-lookUpTableMin) << "; " << (lookUpTableMax-lookUpTableMin) << "==" << INFINITY << "? " << ((lookUpTableMax-lookUpTableMin) == INFINITY) << endl; 
	if (lookUpTableMax-lookUpTableMin <=0 || ((lookUpTableMax-lookUpTableMin) == INFINITY)) {
		scalarVals.create(patches.size(), 1, CV_64FC1); 
		scalarVals = 0.; 
		return; 
	}
	
	if (_REGRESSOR_DEBUG) cout << "After feature output, scalarVals has size " << scalarVals.rows << "x" << scalarVals.cols << endl; 
	
	if (_REGRESSOR_DEBUG) cout << "Got feature output, doing table lookup" << endl; 
	scalarVals = (scalarVals - lookUpTableMin) * (1.0*lookUpTable.rows / (lookUpTableMax - lookUpTableMin)); 
	
	min(scalarVals, (double)lookUpTable.rows-1., scalarVals); 
	max(scalarVals, 0., scalarVals); 
	
	if (_REGRESSOR_DEBUG) {
		cout << "Getting table entries" << endl; 
		double min_ind, max_ind; 
		minMaxLoc(scalarVals, &min_ind, &max_ind); 
		cout << "LUT has size " << lookUpTable.rows << "x" << lookUpTable.cols << "; The min ind is " << min_ind << "; max ind is " << max_ind << endl; 
	}
	
	for (int i = 0; i < scalarVals.rows; i++) {
		int ind = scalarVals.at<double>( i, 0);
		double lut = lookUpTable.at<double>(ind, 0);
		scalarVals.at<double>( i, 0) =  lut ; 
	}	
	if (_REGRESSOR_DEBUG) {
		Mat m = scalarVals; 
		cout << "Regressor predictions: " << endl; 
		NMPTUtils::printMat(m.t()); 
	}
	if (_REGRESSOR_DEBUG) cout << "Done with predict." <<  endl; 
}

void FeatureRegressor::predict(const vector<ImagePatch*> patches, Mat &scalarVals) const {
	if (_REGRESSOR_DEBUG) cout << "Regressor is predicting Patch List, matrix vals" << endl; 
	patchFeature->evaluateImagePatches(patches, scalarVals); //We probably shouldn't do this before the next check, but we actually want to take extra time.
	
	if (_REGRESSOR_DEBUG) {
		Mat m = scalarVals; 
		cout << "Feature outputs: " << endl; 
		NMPTUtils::printMat(m.t()); 
		
	}
	
	if (_REGRESSOR_DEBUG) cout << "Lookup table has range " << (lookUpTableMax-lookUpTableMin) << "; " << (lookUpTableMax-lookUpTableMin) << "==" << INFINITY << "? " << ((lookUpTableMax-lookUpTableMin) == INFINITY) << endl; 
	if (lookUpTableMax-lookUpTableMin <=0 || ((lookUpTableMax-lookUpTableMin) == INFINITY)) {
		scalarVals.create(patches.size(), 1, CV_64FC1); 
		scalarVals = 0.; 
		return; 
	}
	 
	if (_REGRESSOR_DEBUG) cout << "After feature output, scalarVals has size " << scalarVals.rows << "x" << scalarVals.cols << endl; 
	
	if (_REGRESSOR_DEBUG) cout << "Got feature output, doing table lookup" << endl; 
	scalarVals = (scalarVals - lookUpTableMin) * (1.0*lookUpTable.rows / (lookUpTableMax - lookUpTableMin)); 
	
	min(scalarVals, (double)lookUpTable.rows-1., scalarVals); 
	max(scalarVals, 0., scalarVals); 
	
	if (_REGRESSOR_DEBUG) {
		cout << "Getting table entries" << endl; 
		double min_ind, max_ind; 
		minMaxLoc(scalarVals, &min_ind, &max_ind); 
		cout << "LUT has size " << lookUpTable.rows << "x" << lookUpTable.cols << "; The min ind is " << min_ind << "; max ind is " << max_ind << endl; 
	}
	
	for (int i = 0; i < scalarVals.rows; i++) {
		int ind = scalarVals.at<double>( i, 0);
		double lut = lookUpTable.at<double>(ind, 0);
		scalarVals.at<double>( i, 0) =  lut ; 
	}	
	if (_REGRESSOR_DEBUG) {
		Mat m = scalarVals; 
		cout << "Regressor predictions: " << endl; 
		NMPTUtils::printMat(m.t()); 
	}
	if (_REGRESSOR_DEBUG) cout << "Done with predict." <<  endl; 
}

void FeatureRegressor::predict(const vector<ImagePatch*> patches, CvMat*& scalarVals) const {
	Mat vals; 
	predict(patches, vals); 
	CvMat m = vals;
	scalarVals = cvCloneMat(&m); 
}

void FeatureRegressor::predictPatchList( PatchList* patches) const {
	if (_REGRESSOR_DEBUG) cout << "Regressor is predicting Patch List" << endl; 
	int size = patches->getCurrentListLength(); 
	double** dInds = &patches->destInds[0]; 
	if (_REGRESSOR_DEBUG) cout << "Look Up Table Min: " << lookUpTableMin << " ; Max: " << lookUpTableMax << endl; 
	
	if (lookUpTableMax-lookUpTableMin <= 0) {
		for (int i = 0; i < size; i++) {
			*dInds[i] = 0; 
		}
		return; 
	}
	
	patchFeature->filterPatchList(patches);
	
	if (_REGRESSOR_DEBUG) cout << "Got feature output, doing table lookup" << endl; 
	
	double scale = 1.0*lookUpTable.rows/(lookUpTableMax-lookUpTableMin); 
	int maxind = lookUpTable.rows-1; 
	
	if (_REGRESSOR_DEBUG) cout << "Getting table entries" << endl; 
	for (int i = 0; i < size; i++) {
		int index = (*dInds[i]-lookUpTableMin)*scale; 
		index = min(index, maxind); 
		index = max(index, 0); 
		*dInds[i] = lookUpTable.at<double>(index,0); 
	}
	if (_REGRESSOR_DEBUG) cout << "Finished predicting Patch List" << endl; 
}

void FeatureRegressor::applyLUTToImage(IplImage* image) const {
	cv::Mat newImage = image; 
	applyLUTToImage(newImage); 
}

void FeatureRegressor::applyLUTToImage(cv::Mat &image) const {
	if (lookUpTableMax-lookUpTableMin <= 0) {
		image = 0.; 
		return; 
	}
	MatIterator_<double> it = image.begin<double>(), it_end = image.end<double>(); 
	for (;it != it_end; it++) {
		int index = (*it - lookUpTableMin) / (lookUpTableMax-lookUpTableMin) * lookUpTable.rows; 
		index = min(index, lookUpTable.rows-1); 
		index = max(index, 0); 
		*it = lookUpTable.at<double>(index,0); 
	}
}
 
/* input: NxM, N data points, M Dims
 * labels: NxO, N data points, O outputs
 * weights: Nx1, 1 weight per data point
 * xqueries: QxM, Q query points, M Dims
 * tau: variance of gaussian weighting window
 * eps: minimal attention paid to all points
 *
 * predictions: QxO, Q query points, O output dimensions. 
 */
/*
CvMat* FeatureRegressor::RBF(CvMat* input, CvMat* labels, CvMat* weights, CvMat* xqueries, double tau, double eps) {
	int N = input->rows; 
	int M = input->cols; 
	int O = labels->cols; 
	int Q = xqueries->rows; 

	
	
	CvMat* predictions = cvCreateMat(Q, O, CV_64FC1); 
	double negt2inv = -1.0/(2.0*tau*tau); 
	
	CvMat* xdiff = cvCreateMat(1, M, CV_64FC1); 
	CvMat* wtrow = cvCreateMat(1, N, CV_64FC1); 
	
	CvMat currx1; 
	CvMat currx2; 
	CvMat curry; 
	
	for (int i = 0; i < Q; i++) {
		cvGetRow(xqueries, &currx1, i); 
		cvGetRow(predictions, &curry, i); 
		for (int j = 0; j < N; j++) {
			cvGetRow(input, &currx2, j); 
			cvSub(&currx1, &currx2, xdiff); 
			double sqdist = cvDotProduct(xdiff, xdiff);
			cvSetReal2D(wtrow, 0, j, exp(sqdist*negt2inv+eps)*cvGetReal2D(weights, j, 0));
		}
		
		
		CvScalar norm = cvSum(wtrow); 
		 
		//cout << "Norm is " << norm.val[0] << endl; 
		if (norm.val[0] > 0)  {
			norm.val[0] = 1.0/norm.val[0]; 
			cvScale(wtrow, wtrow, norm.val[0]); 
		}
		else {
			cvSetZero(wtrow); 
		}
		 
		
		//cvNormalize(wtrow, wtrow, 1, 0, CV_L1); 
		cvMatMul(wtrow, labels, &curry); 
		for(int j = 0; j < curry.cols; j++) {
			if (cvIsNaN(cvGetReal2D(&curry, 0, j)) || cvIsInf(cvGetReal2D(&curry, 0, j))) {
				cout << "!!!!!Warning! RBF LUT Value " << i << ", " << j << " is " 
				<< cvGetReal2D(&curry, 0, j) << " (norm was " << norm.val[0] << ")" << "; -1/tau^2 was " << negt2inv<<  endl;
				if (j > 0) {
					cvSetReal2D(&curry, 0, j, cvGetReal2D(&curry, 0, j-1));
				} else {
					exit(0);
				}
			}
			if (cvGetReal2D(&curry, 0, j)<-1) {
				cvSetReal2D(&curry,0,j,-1); 
			}
			if (cvGetReal2D(&curry, 0, j)>1) {
				cvSetReal2D(&curry,0,j,1); 
			}
		}
	}
	
	cvReleaseMat(&xdiff); 
	cvReleaseMat(&wtrow); 
	return predictions; 
}
*/

ostream& operator<< (ostream& ofs, const FeatureRegressor* reg) {
	if (_REGRESSOR_DEBUG) cout << "Writing regressor to stream" << endl; 
	ofs.precision(20);
	ofs << reg->lookUpTableMax << " " << reg->lookUpTableMin << endl;
	if (reg->lookUpTableMax - reg->lookUpTableMin <= 0) {
		ofs << 0 << " " << 0 << endl ;
		return ofs; 
	}
	ofs << reg->lookUpTable.cols << " " << reg->lookUpTable.rows << endl; 
	if (_REGRESSOR_DEBUG) cout << "Writing Look-up table" << endl; 
	for (int i = 0; i < reg->lookUpTable.rows; i++) {
		for (int j = 0; j < reg->lookUpTable.cols; j++) {
			if (_REGRESSOR_DEBUG) cout << "Writing table entry (" << i << "," << j << ")" << endl; 
			ofs << reg->lookUpTable.at<double>(i, j) << " "; 
		}
		ofs << endl; 
	}
	if (_REGRESSOR_DEBUG) cout << "Finished writing feature regressor" << endl; 
	//ofs.unsetf(ios_base::floatfield); 
	ofs << reg->patchFeature;
	return ofs; 
}

ostream& operator<< (ostream& ofs, const FeatureRegressor &reg) {
	if (_REGRESSOR_DEBUG) cout << "Writing regressor to stream" << endl; 
	ofs.precision(20);
	ofs << reg.lookUpTableMax << " " << reg.lookUpTableMin << endl;
	if (reg.lookUpTableMax - reg.lookUpTableMin <= 0) {
		ofs << 0 << " " << 0 << endl ;
		return ofs; 
	}
	ofs << reg.lookUpTable.cols << " " << reg.lookUpTable.rows << endl; 
	if (_REGRESSOR_DEBUG) cout << "Writing Look-up table" << endl; 
	for (int i = 0; i < reg.lookUpTable.rows; i++) {
		for (int j = 0; j < reg.lookUpTable.cols; j++) {
			if (_REGRESSOR_DEBUG) cout << "Writing table entry (" << i << "," << j << ")" << endl; 
			ofs << reg.lookUpTable.at<double>(i, j) << " "; 
		}
		ofs << endl; 
	}
	if (_REGRESSOR_DEBUG) cout << "Finished writing feature regressor" << endl; 
	//ofs.unsetf(ios_base::floatfield); 
	ofs << reg.patchFeature;
	return ofs; 
}

istream& operator>> (istream& ifs, FeatureRegressor *&model) {
	double lookUpTableMax; 
	double lookUpTableMin; 
	ifs >> lookUpTableMax; 
	ifs >> lookUpTableMin; 
	if (_REGRESSOR_DEBUG) cout << "LUTMax: " << lookUpTableMax << " ; LUTMin: " << lookUpTableMin << endl; 
	int paramheight = 0; 
	int paramwidth = 0; 
	ifs >> paramwidth >> paramheight; 
	Mat lut; 
	if (paramwidth  >0 && paramheight > 0) {
		if (_REGRESSOR_DEBUG) cout << "Making LUT With size " << paramheight << "x" << paramwidth << endl; 
		lut.create(paramheight, paramwidth, CV_64F); 
		
		if (_REGRESSOR_DEBUG) cout << "Created param vec and ready to fill it with parameters." << endl ;
		double val = 0; 
		for (int i = 0; i < lut.rows; i++) {
			for (int j = 0; j < lut.cols; j++) {
				ifs >> val; 
				if (_REGRESSOR_DEBUG) cout << "Read in parameter " << j << " as value " << val << endl; 
				lut.at<double>( i, j) =  val ; 
			}
		}
		if (_REGRESSOR_DEBUG) cout << "Filled LUT" << endl; 
	}
	//if (_REGRESSOR_DEBUG) cout << "Setting feature to a new feature of type " << name << endl; 
	Feature* feat; 
	if (_REGRESSOR_DEBUG) cout << "Reading Feature" << endl; 
	ifs >> feat; 
	if (_REGRESSOR_DEBUG) cout << "Read Feature, creating regressor." << endl; 
	model = new FeatureRegressor(feat); 
	if (_REGRESSOR_DEBUG) cout << "Created regressor, filling values. " << endl; 
	model->lookUpTableMax = lookUpTableMax; 
	model->lookUpTableMin = lookUpTableMin; 
	model->lookUpTable = lut.clone(); 
	delete(feat); 
	return ifs; 
}


istream& operator>> (istream& ifs, FeatureRegressor &model) {
	double lookUpTableMax; 
	double lookUpTableMin; 
	ifs >> lookUpTableMax; 
	ifs >> lookUpTableMin; 
	if (_REGRESSOR_DEBUG) cout << "LUTMax: " << lookUpTableMax << " ; LUTMin: " << lookUpTableMin << endl; 
	int paramheight = 0; 
	int paramwidth = 0; 
	ifs >> paramwidth >> paramheight; 
	Mat lut; 
	if (paramwidth  >0 && paramheight > 0) {
		if (_REGRESSOR_DEBUG) cout << "Making LUT With size " << paramheight << "x" << paramwidth << endl; 
		lut.create(paramheight, paramwidth, CV_64F); 
		
		if (_REGRESSOR_DEBUG) cout << "Created param vec and ready to fill it with parameters." << endl ;
		double val = 0; 
		for (int i = 0; i < lut.rows; i++) {
			for (int j = 0; j < lut.cols; j++) {
				ifs >> val; 
				if (_REGRESSOR_DEBUG) cout << "Read in parameter " << j << " as value " << val << endl; 
				lut.at<double>( i, j) =  val ; 
			}
		}
		if (_REGRESSOR_DEBUG) cout << "Filled LUT" << endl; 
	}
	//if (_REGRESSOR_DEBUG) cout << "Setting feature to a new feature of type " << name << endl; 
	Feature* feat; 
	if (_REGRESSOR_DEBUG) cout << "Reading Feature" << endl; 
	ifs >> feat; 
	if (_REGRESSOR_DEBUG) cout << "Read Feature, creating regressor." << endl; 
	model = FeatureRegressor(feat); 
	if (_REGRESSOR_DEBUG) cout << "Created regressor, filling values. " << endl; 
	model.lookUpTableMax = lookUpTableMax; 
	model.lookUpTableMin = lookUpTableMin; 
	model.lookUpTable = lut.clone(); 
	delete(feat); 
	return ifs; 
}

