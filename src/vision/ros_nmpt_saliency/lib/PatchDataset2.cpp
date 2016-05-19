/*
 *  PatchDataset2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/25/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "PatchDataset2.h"
#include <math.h>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "DebugGlobals.h"
#include "DetectionEvaluator.h"
#include "OpenLoopPolicies.h"
#include "FastPatchList.h"
#include "NMPTUtils.h"

using namespace std; 
using namespace cv;

int PatchDataset2::minPerNegIm = 100; 

PatchDataset2::PatchDataset2() {
	init(); 
}
PatchDataset2 & PatchDataset2::operator=(const PatchDataset2 &rhs) {
	if (this != &rhs) {
		copy(rhs); 
	}
	return *this; 
}

PatchDataset2::PatchDataset2(const PatchDataset2 &rhs) {
	if (this != &rhs) {
		copy(rhs); 
	} 
}

PatchDataset2::PatchDataset2(Size patchsize,
							 const std::string &positiveImageListFilename, 
							 const std::string &positiveImageLabelsFilename,
							 const std::string &negativeImageListFilename ,
							 int posExamplePatchRadius,
							 int posExampleScaleRadius,
							 int numPosPatches, 
							 int numNegPatches,
							 int useFastPatchList, 
							 double scale) {
	init(patchsize, positiveImageListFilename, positiveImageLabelsFilename, 
		 negativeImageListFilename, posExamplePatchRadius, posExampleScaleRadius, 
		 numPosPatches, numNegPatches, useFastPatchList, scale); 
}

void PatchDataset2::setUseFastPatchList(int yesorno) {
	useFast = yesorno; 
	list = useFast? &fpl : &pl; 
}

int PatchDataset2::getUseFastPatchList() const {
	return useFast ; 
}

void PatchDataset2::init(Size patchsize,
						 const std::string &positiveImageListFilename, 
						 const std::string &positiveImageLabelsFilename,
						 const std::string &negativeImageListFilename ,
						 int posExamplePatchRadius,
						 int posExampleScaleRadius,
						 int numPosPatches, 
						 int numNegPatches,
						 int useFastPatchList, 
						 double scale) {
	
	setUseFastPatchList(useFastPatchList); 
	
	this->scale = scale; 
	
	posImagesFileList = positiveImageListFilename; 
	posImagesLabelsList = positiveImageLabelsFilename; 
	negImagesFileList = negativeImageListFilename; 
	
	int numPosImages = 0; 
	int numNegImages = 0; 
	int posImagesHaveLabels = 0; 
	
	patches.clear(); 
	labels.clear(); 
	posPatches.clear(); 
	negPatches.clear(); 
	
	
	patchSize = patchsize; 
	
	if (posImagesFileList.empty() && negImagesFileList.empty() ) return; 
	
	if (!posImagesFileList.empty()) {
		posImagesHaveLabels = !posImagesLabelsList.empty(); 
		
		posImageDataset = ImageDataSet2(posImagesFileList, 
										posImagesLabelsList);
		if (posImagesHaveLabels) {
			evaluator.setDataSet(posImageDataset);
			numPosImages = evaluator.getFileNames().size(); 
		} else {		
			numPosImages = posImageDataset.getNumEntries(); 
		}
	}
	
	
	if (!negImagesFileList.empty()) {
		negImageDataset = ImageDataSet2(negImagesFileList); 
		numNegImages = negImageDataset.getNumEntries(); 
	} else {
		negImageDataset = posImageDataset; 
	}
	
	Mat image; 
	
	if (posImagesHaveLabels) { // Extract labeled regions from images. 
		vector<string> names = evaluator.getFileNames(); 
		vector<vector<Rect> > locs = evaluator.getTargetLocations(); 
		for (size_t imNum = 0; imNum < names.size(); imNum++) {
			if (_DATASET_DEBUG) cout << "Getting patches from positive example " << imNum << endl; 
			
			image = imread(names[imNum], 0);
			list->setImage(image);  
			for (size_t patchNum = 0; patchNum < locs[imNum].size(); patchNum++) {
				Rect ROI = locs[imNum][patchNum]; 
				if (ROI.x < 0) ROI.x = 0; 
				if (ROI.y < 0) ROI.y = 0; 
				if (ROI.x+ROI.width > image.cols) ROI.width=image.cols-ROI.x; 
				if (ROI.y+ROI.height > image.rows) ROI.height=image.rows-ROI.y; 
				vector<ImagePatch2> objPatches = list->getNearbyPatches(ROI,
																	   posExamplePatchRadius,
																	   posExampleScaleRadius); 
				
				for (size_t z = 0; z < objPatches.size(); z++) {
					patches.push_back(objPatches[z]); 
					labels.push_back(1); 
					posPatches.push_back(patches.back()); 
				}
			}
		}
	} else if (numPosImages > 0) { //Treat the whole image as a positive patch.
		for (int i = 0; i < posImageDataset.getNumEntries(); i++) {
			Mat im = imread(posImageDataset.getFileName(i), 0); 
			Mat patch; 
			resize(im, patch, patchSize, 0, 0, INTER_NEAREST); 
			ImagePatch2 impatch; 
			patches.push_back(impatch); 
			patches.back().setImage(patch, 0, 1, 0, 0); 
			posPatches.push_back(patches.back());
			labels.push_back(1); 
		}								 
	}
	
	if (numPosPatches > -1 && numPosPatches < (int)patches.size()) {
		patches.resize(numPosPatches); 
		posPatches.resize(numPosPatches); 
	}
	
	numPosPatches = patches.size(); 
	
	if (numNegImages < 1) { //Take negative patches from positive images
		
		
		vector<string> names = evaluator.getFileNames(); 
		vector<vector<Rect> > locs = evaluator.getTargetLocations(); 
		numNegImages = names.size(); 
		
		if (numNegPatches < 0 && numNegImages > 0) numNegPatches = numPosPatches; 
		if (_DATASET_DEBUG) cout << "Found " << numPosPatches << " positive patches." << endl ;
		if (_DATASET_DEBUG) cout << "Looking for " << numNegPatches << " negative patches." << endl ;
		
		int numTotal = numPosPatches+numNegPatches; 
		
		int ind = numPosPatches; 
		int patchesPerNegIm = ceil( (numTotal-numPosPatches)*1.0 / numNegImages); 
		
		patchesPerNegIm = patchesPerNegIm > minPerNegIm ? patchesPerNegIm : minPerNegIm; 
		
		if (_DATASET_DEBUG) cout << "Getting " << numTotal- numPosPatches 
		<< " negative patches from " << numNegImages << " images with " 
		<< patchesPerNegIm << " patches per image." << endl; 
		
		for (size_t i = 0; i < names.size(); i++) {
			if (_DATASET_DEBUG) cout << "Getting patches from negative example " << i << endl; 
			if (_DATASET_DEBUG) cout << "Loading neg image " << names[i] << endl; 
			if (ind >= numTotal) break; 
			
			image = imread(names[i], 0); //0 means CV_LOAD_IMAGE_GRAYSCALE
			
			vector<ImagePatch2> impatches = getRandomPatches(patchesPerNegIm,image,patchSize, locs[i]);
			
			for (size_t j = 0; j < impatches.size(); j++) {
				if (ind >= numTotal) break; 
				patches.push_back(impatches[j]); 
				negPatches.push_back(impatches[j]); 
				labels.push_back(-1); 
				ind++; 
			}
		}	
	} else { // Take negative patches from background images
		if (numNegPatches < 0 && numNegImages > 0) numNegPatches = numPosPatches; 
		
		if (_DATASET_DEBUG) cout << "Found " << numPosPatches << " positive patches." << endl ;
		if (_DATASET_DEBUG) cout << "Looking for " << numNegPatches << " negative patches." << endl ;
		
		int numTotal = numPosPatches+numNegPatches; 
		
		int ind = numPosPatches; 
		int patchesPerNegIm = ceil( (numTotal-numPosPatches)*1.0 / numNegImages); 
		patchesPerNegIm = patchesPerNegIm > minPerNegIm ? patchesPerNegIm : minPerNegIm; 
		
		if (_DATASET_DEBUG) cout << "Getting " << numTotal- numPosPatches 
		<< " negative patches from " << numNegImages << " images with " 
		<< patchesPerNegIm << " patches per image." << endl; 
		
		for (int i = 0; i < negImageDataset.getNumEntries(); i++) {
			if (_DATASET_DEBUG) cout << "Getting patches from negative example " << i << endl; 
			image = imread( negImageDataset.getFileName(i), 0); //CV_LOAD_IMAGE_GRAYSCALE);

			for (int j = 0; j < patchesPerNegIm; j++) {
				vector<Rect> r; 
				patches.push_back(getRandomPatch(image,patchSize,r)); 
				negPatches.push_back(patches.back()); 
				labels.push_back(-1); 
				ind++; 
			}
		}	
	}
	
}


PatchDataset2::~PatchDataset2() {
}


Size PatchDataset2::getPatchSize() const{
	return patchSize; 
}


vector<ImagePatch2> PatchDataset2::getPatches() const{
	return patches; 
}


vector<ImagePatch2> PatchDataset2::getPosPatches() const {
	return posPatches; 
}


vector<ImagePatch2> PatchDataset2::getNegPatches() const {
	return negPatches; 
}

void PatchDataset2::getLabels(Mat &dest) const {
	int numTotal = labels.size(); 
	dest.create(numTotal, 1, CV_64F); 
	for (int i = 0; i < numTotal; i++) 
		dest.at<double>(i,0) = labels[i]; 
}

ImageDataSet2 PatchDataset2::getNegImagesDataset() const {
	return negImageDataset; 
}

ImageDataSet2 PatchDataset2::getPosImagesDataset() const {
	return posImageDataset; 
}

int PatchDataset2::getNumUniquePositiveImages() const {
	if (!evaluator.empty())	return evaluator.dataSetSize(); 
	return posImageDataset.getNumEntries(); 
}

string PatchDataset2::getUniquePosImageName(int num) const {
	if (num < 0 || num > getNumUniquePositiveImages()) return ""; 
	if (!evaluator.empty()) return evaluator.getFileNames()[num]; 
	return posImageDataset.getFileName(num); 
}


vector<Rect> PatchDataset2::getObjectLocationsInPosImage(int num) const {
	vector<Rect> r; 
	if (num < 0 || evaluator.empty() || num > getNumUniquePositiveImages()) return r;
	return evaluator.getTargetLocations()[num]; 	
}


string PatchDataset2::getNegImagesFileName() const {
	return negImagesFileList; 
}


string PatchDataset2::getPosImagesFileName() const{
	return posImagesFileList; 
} 


string PatchDataset2::getPosLabelsFileName() const {
	return posImagesLabelsList; 
} 


ImagePatch2 PatchDataset2::getPatch(const Mat &image, Size patchSize, Point objCenter, Size objSize) const {
	
	list->setImage(image); 
	
	Mat patchImage(patchSize, CV_8U); 
	int x = objCenter.x-objSize.width/2; 
	int y = objCenter.y-objSize.height/2; 
	Rect ROI(x,y,objSize.width, objSize.height); 
	if (ROI.x < 0) ROI.x = 0; 
	if (ROI.y < 0) ROI.y = 0; 
	if (ROI.x+ROI.width > image.cols) ROI.width=image.cols-ROI.x; 
	if (ROI.y+ROI.height > image.rows) ROI.height=image.rows-ROI.y; 
	
	vector<ImagePatch2> patches = list->getNearbyPatches(ROI,0,0); 
	if (!patches.empty())		return patches[0]; 
	
	resize(image(ROI), patchImage, patchImage.size(), 0, 0, INTER_NEAREST); 
	
	ImagePatch2 retval; 
	
	retval.setImage(patchImage, 0, 1, 0, 0); 
	return retval; 
}


ImagePatch2 PatchDataset2::getRandomPatch(const Mat &image, Size patchSize, 
										  const vector<Rect> &blackoutList) const{	
	return getRandomPatches(1, image, patchSize, blackoutList)[0]; 
}

vector<ImagePatch2> PatchDataset2::getRandomPatches(int numPatches, const Mat & image, 
													Size patchSize, const vector<Rect> &blackoutList) const{
	
	Mat patch_image; 
	
	list->setImage(image);  
	vector<SearchResult> scaleResults; 
	vector<SearchResult> allResults; 
	allResults.clear(); 
	for (int s = 0; s < list->getNumScales(); s++) {
		list->resetListToScale(s);
		scaleResults.clear(); 
		list->getRemainingPatches(scaleResults, blackoutList, patchSize.width*.5, 2);
		allResults.insert(allResults.end(), scaleResults.begin(), scaleResults.end()); 
	}
	
	vector<ImagePatch2> retvec; 
	
	for (int i = 0; i < numPatches; i++) {
		
		int locnum = (int)(NMPTUtils::randomFloat()*allResults.size());
		 
		list->fillImageWithPixelsOfSearchPatch(patch_image, allResults[locnum]); 
		
		ImagePatch2 retval; 
		retval.setImage(patch_image, 0, 1, 0, 0); 
		retvec.push_back(retval); 
	}
	
	return retvec; 
}

void PatchDataset2::recreatePosNegPatches() {
	posPatches.clear(); 
	negPatches.clear(); 
	for (size_t i = 0; i < patches.size(); i++) {
		if (labels[i] > 0)
			posPatches.push_back(patches[i]); 
		else
			negPatches.push_back(patches[i]); 
	}
}

void PatchDataset2::copy(const PatchDataset2 &rhs) {
	setUseFastPatchList(rhs.useFast); 
	
	posImagesFileList= rhs.posImagesFileList; 
	posImagesLabelsList = rhs.posImagesLabelsList; 
	negImagesFileList = rhs.negImagesFileList; 
	
	patchSize = rhs.patchSize; 
	scale = rhs.scale; 
	
	patches=rhs.patches; 	
	labels=rhs.labels; 
	
	recreatePosNegPatches(); 
	
	posImageDataset = rhs.posImageDataset; 
	negImageDataset = rhs.negImageDataset; 
	
	evaluator = rhs.evaluator;
}

int PatchDataset2::empty() const {
	return posImageDataset.empty() && negImageDataset.empty() && evaluator.empty(); 
}


FileStorage& operator << (cv::FileStorage &fs, const PatchDataset2 &rhs) {
	fs << "{" << "useFast" << rhs.useFast << "scale" << rhs.scale 
	<< "posImagesFileList" << rhs.posImagesFileList
	<< "posImagesLabelsList" << rhs.posImagesLabelsList <<  "negImagesFileList" 
	<< rhs.negImagesFileList << "patch_w" << rhs.patchSize.width << "patch_h" 
	<< rhs.patchSize.height << "posImageDataset" << rhs.posImageDataset 
	<< "negImageDataset" << rhs.negImageDataset << "evaluator" << rhs.evaluator
	<< "numPatches" << (int)rhs.patches.size();
	if (!rhs.patches.empty()) {
		fs << "labels" << "[" << rhs.labels << "]"; 
		fs << "patches" << "[" ; 
		for (size_t i = 0; i < rhs.patches.size(); i++) {
			fs << rhs.patches[i] ; 
		}
		fs << "]"; 
	}
	fs << "}" ; 
	return fs; 
}

void operator >> ( const cv::FileNode &fs, PatchDataset2 &rhs)  {
	fs["useFast"] >> rhs.useFast; 
	rhs.setUseFastPatchList(rhs.useFast); 
	fs["scale"] >> rhs.scale; 
	fs["posImagesFileList"] >> rhs.posImagesFileList;
	fs["posImagesLabelsList"] >> rhs.posImagesLabelsList; 
	fs["negImagesFileList"] >> rhs.negImagesFileList;
	fs["patch_w"] >> rhs.patchSize.width; 
	fs["patch_h"] >> rhs.patchSize.height; 
	fs["posImageDataset"] >> rhs.posImageDataset; 
	fs["negImageDataset"] >> rhs.negImageDataset; 
	fs["evaluator"] >> rhs.evaluator; 
	int numPatches; 
	fs["numPatches"] >> numPatches;
	if (numPatches > 0) {
		FileNode n = fs["labels"]; 
		rhs.labels.clear(); 
		for (size_t i = 0; i < n.size(); i++) {
			int l; 
			n[i] >> l ;
			rhs.labels.push_back(l); 
		}
		n = fs["patches"]; 
		for (size_t i = 0; i < n.size(); i++) {
			ImagePatch2 p; 
			n[i] >> p; 
			rhs.patches.push_back(p); 
		}
	}
	rhs.recreatePosNegPatches(); 
}