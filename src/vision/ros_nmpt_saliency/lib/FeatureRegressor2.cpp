/*
 *  FeatureRegressor2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/23/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "Feature2.h"
#include "FeatureRegressor2.h"
#include "DebugGlobals.h"
#include "NMPTUtils.h"
#include "ImagePatch.h"
#include <opencv2/core/core.hpp>

using namespace cv;  
using namespace std; 

double FeatureRegressor2::TAU = .05; 
double FeatureRegressor2::EPS = .001; 

FeatureRegressor2::FeatureRegressor2() {
	patchFeature = NULL; 
}

FeatureRegressor2::FeatureRegressor2(const Feature2* feature) {
	patchFeature = NULL; 
	setFeature(feature); 
}

FeatureRegressor2 & FeatureRegressor2::operator=(const FeatureRegressor2 &rhs) {
	if (this != &rhs) {
		setFeature(rhs.patchFeature); 
		lookUpTableMin = rhs.lookUpTableMin; 
		lookUpTableMax = rhs.lookUpTableMax; 
		lookUpTable = rhs.lookUpTable.clone(); 
	}
	return *this; 
}

FeatureRegressor2::FeatureRegressor2(const FeatureRegressor2 &rhs) {
	if (this != &rhs) {
		patchFeature = NULL; 
		setFeature(rhs.patchFeature); 
		lookUpTableMin = rhs.lookUpTableMin; 
		lookUpTableMax = rhs.lookUpTableMax; 
		lookUpTable = rhs.lookUpTable.clone(); 
	} 
}

void FeatureRegressor2::setFeature(const Feature2* feature) {
	if (patchFeature != NULL) delete (patchFeature); 
	patchFeature = feature->copy(); 
	if (_REGRESSOR_DEBUG) {
		cout << feature->debugInfo(); 
	}
	lookUpTableMin = 0; 
	lookUpTableMax = 0; 
	lookUpTable = Mat(); 
}

FeatureRegressor2::~FeatureRegressor2() {	
	if (patchFeature != NULL) 
		delete(patchFeature); 
}

Feature2* FeatureRegressor2::getFeature() const {
	return patchFeature; 
}

cv::Size FeatureRegressor2::getFeaturePatchSize() const {
	return patchFeature->getPatchSize(); 
}

double FeatureRegressor2::getLUTRange() const {
	return lookUpTableMax-lookUpTableMin; 
}

void FeatureRegressor2::train(int numTableElements, const vector<ImagePatch2> &data, const cv::Mat &labels, const cv::Mat &dataWeights) {
	
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

void FeatureRegressor2::combineLUTs(const FeatureRegressor2 &other) {
	lookUpTable += other.lookUpTable; 
}

void FeatureRegressor2::predict(const vector<ImagePatch2> &patches, Mat &scalarVals) const {
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

void FeatureRegressor2::predictPatchList( PatchList2* patches) const {
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

void FeatureRegressor2::applyLUTToImage(cv::Mat &image) const {
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

ostream& operator<< (ostream& ofs, const FeatureRegressor2 &reg) {
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

istream& operator>> (istream& ifs, FeatureRegressor2 &model) {
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
	Feature2* feat; 
	if (_REGRESSOR_DEBUG) cout << "Reading Feature" << endl; 
	ifs >> feat; 
	if (_REGRESSOR_DEBUG) cout << "Read Feature, creating regressor." << endl; 
	model = FeatureRegressor2(feat); 
	if (_REGRESSOR_DEBUG) cout << "Created regressor, filling values. " << endl; 
	model.lookUpTableMax = lookUpTableMax; 
	model.lookUpTableMin = lookUpTableMin; 
	model.lookUpTable = lut.clone(); 
	delete(feat); 
	return ifs; 
}


cv::FileStorage& operator << (cv::FileStorage &fs, const FeatureRegressor2 &rhs) {
	if (_REGRESSOR_DEBUG) cout << "Writing regressor to storage" << endl; 
	fs << "{" << "min" << rhs.lookUpTableMin << "max" << rhs.lookUpTableMax 
	<< "lut" << rhs.lookUpTable; 
	rhs.patchFeature->writeToFile(fs, "feature"); 	
	fs << "}"; 
	if (_REGRESSOR_DEBUG) cout << "Finished writing feature regressor" << endl; 
	return fs; 
}
void operator >> (const cv::FileNode &fs, FeatureRegressor2 &rhs) {
	if (_REGRESSOR_DEBUG) cout << "Reading regressor" << endl; 
	fs["min"] >> rhs.lookUpTableMin; 
	fs["max"] >> rhs.lookUpTableMax; 
	fs["lut"] >> rhs.lookUpTable; 
	rhs.patchFeature = Feature2::readFromFile(	fs["feature"] ) ; 
	if (_REGRESSOR_DEBUG) cout << "Finished reading feature regressor" << endl; 
}
