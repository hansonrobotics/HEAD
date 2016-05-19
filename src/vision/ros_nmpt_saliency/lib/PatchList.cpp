/*
 *  PatchList.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/25/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "PatchList.h"
#include "DebugGlobals.h"
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std; 
using namespace cv; 

PatchList::PatchList(Size minSize, Size maxSize, double scaleInc, double stepWidth, 
					  Size baseObjectSize) : minsize(minSize), maxsize(maxSize), 
stepwidth(stepWidth), scaleinc(scaleInc), default_width(baseObjectSize.width), default_height(baseObjectSize.height){
	setNullPointers(); 
	if (minsize.width == 0 || minsize.height == 0) {
		if (_PATCHLIST_DEBUG) cout << "Creating list with size " << default_width << ", " << default_height << endl; 
	}
	numScales = 0; 
	totalPatches = 0; 
	currentListLength = 0; 
	currentScale = 0 ; 
	shouldResetAccumulator = 1; 
	origImageSize = Size(0,0); 
	
	copyImageData = 1; 
	scaleDownPatches = 1; 
}

PatchList::~PatchList() {
	if (_PATCHLIST_DEBUG) cout<< "Patch List Destuctor" << endl; 
	if (_PATCHLIST_DEBUG) cout<< "Calling clearPointers()" << endl; 
	clearPointers(); 
	
	if (_PATCHLIST_DEBUG) cout << "Deleting images 1--" << images.size() << endl ;
	for (unsigned int i = 0; i < images.size(); i++) 
		if (i < 1 || images[i]!=images[0])
			delete(images[i]) ;
	 
}

void PatchList::resetListToScale(int scale){
	
	if (_PATCHLIST_DEBUG) cout<< "Reset List To Scale" << endl; 
	if (accInds == NULL || srcInds == NULL || destInds == NULL || imLoc==NULL) {
		cout<< "Warning: Must call setImage before setting a PatchList Scale." << endl; 
		return; 
	}
	
	checkAndWarn(scale); 
	
	if (shouldResetAccumulator) {
		accumulatorImage = 0.0; 
	}
	shouldResetAccumulator = 0; 
	if (scale == currentScale && currentListLength == numAtScale[scale])
		return; 
	memcpy(srcInds, srcAtScales[scale] , numAtScale[scale]*sizeof(integral_type*)); 
	memcpy(destInds, destsAtScales[scale] , numAtScale[scale]*sizeof(double*)); 
	memcpy(accInds, accAtScales[scale] , numAtScale[scale]*sizeof(double*)); 
	memcpy(imLoc, imLocAtScales[scale] , numAtScale[scale]*sizeof(int)); 
	currentListLength = numAtScale[scale]; 
	currentScale = scale;
}

Size PatchList::getBasePatchSize(){
	return Size (default_width, default_height); 
}

void PatchList::setBasePatchSize(Size baseSize) {
	default_width = baseSize.width; 
	default_height = baseSize.height; 	
} 

void PatchList::getRemainingPatches(vector<SearchResult>& searchResults) {
	searchResults.clear(); 
	searchResults.resize(currentListLength); 
	ImagePatch* currImage = images[currentScale]; 
	int width = currImage->getImageSize().width; 
	
	for (int i = 0; i < currentListLength; i++) {
		searchResults[i].imageLocation.x = imLoc[i]%origImageSize.width; 		
		searchResults[i].imageLocation.y = imLoc[i]/origImageSize.width; 
		searchResults[i].imageLocation.width=scalePatchSizes[currentScale].width;
		searchResults[i].imageLocation.height=scalePatchSizes[currentScale].height;
		searchResults[i].value = *accInds[i]; 
		int offset = (int)(srcInds[i]-currImage->startOfIntegralData()); 
		searchResults[i]._x = offset%(width+1);
		searchResults[i]._y = offset/(width+1); 
		searchResults[i]._scale = currentScale; 
	}
}
 
void PatchList::accumulateAndRemovePatchesBelowThreshold(double threshold){
	if (_PATCHLIST_DEBUG) cout<< "Accumulate And Remove Patches Below Threshold from scale " << currentScale << endl; 
	int currLast = 0; 
	for (int i = 0; i < currentListLength; i++) {
		*accInds[i] += *destInds[i]; 
		if (*accInds[i] >= threshold) {
			accInds[currLast] = accInds[i]; 
			destInds[currLast] = destInds[i]; 
			srcInds[currLast] = srcInds[i]; 
			imLoc[currLast] = imLoc[i]; 
			currLast++; 
		}
	}
	currentListLength = currLast; 
	shouldResetAccumulator = 1; 
}

int PatchList::getCurrentListLength() {
	return currentListLength; 
}

int PatchList::getTotalPatches() {
	return totalPatches; 
}

int PatchList::getNumScales() {
	return numScales; 
}

void PatchList::checkAndWarn(int scale) {
	if (scale >= numScales || scale < 0) {
		cout<< "Warning: Can't get scale size except for a non-negative value less than numScales (currently " << numScales << ")" << endl; 
		cout << "Desired Scale is .......... " << scale << endl; 
		cout << "Original Image Size is .... " << origImageSize.width << "x" << origImageSize.height << endl; 
		cout << "Current Scale is .......... " << currentScale << endl; 
		cout << "Total Patches is .......... " << totalPatches << endl; 
		cout << "Current List Length is .... " << currentListLength << endl; 
		cout << "Number of images is ....... " << images.size() << endl; 
		cout << "Max size is ............... " << maxsize.width << "x" << maxsize.height << endl; 
		cout << "Min size is ............... " << minsize.width << "x" << minsize.height << endl; 
		cout << "Should reset accumulator? . " << shouldResetAccumulator << endl; 
		exit(0); 
	}
}

int PatchList::getIntegralWidthStepAtScale(int scale){
	if (scale == -1) 
		scale = currentScale; 
	if (_PATCHLIST_DEBUG) cout<< "Get Integral Width Step of Scale " << scale << endl; 
	checkAndWarn(scale); 
	return images[scale]->integralDataRowWidth();// getImageSize().width+1; 
}

Size PatchList::getPatchSizeAtScale(int scale){
	if (scale == -1) 
		scale = currentScale; 
	if (_PATCHLIST_DEBUG) cout<< "Get Patch Size of Scale " << scale << endl; 	
	checkAndWarn(scale); 
	return scalePatchSizes[scale]; 
}

Size PatchList::getFilterSizeAtScale(int scale){
	if (scale == -1) 
		scale = currentScale; 
	if (_PATCHLIST_DEBUG) cout<< "Get Filter Size of Scale " << scale << endl; 
	checkAndWarn(scale); 
	return scaleFilterSizes[scale]; 
}

Size PatchList::getImageSizeAtScale(int scale){
	if (scale == -1) 
		scale = currentScale; 
	if (_PATCHLIST_DEBUG) cout<< "Get Image Size of Scale " << scale << endl; 
	checkAndWarn(scale); 
	return scaleImageSizes[scale]; 
}

void PatchList::setImage(IplImage* newImage) {
	Mat image = newImage; 
	setImage(image); 
}

void PatchList::setImage(const Mat &image) {
	IplImage im = image; 
	IplImage *newImage = &im; 
	
	if (setImageAllScalesNeedsPointerReset(newImage)) {
		if (_PATCHLIST_DEBUG) cout << "Pointers need reset." << endl; 
		resetPointers(); 
	}
	
	if (currentScale >= numScales)
		currentScale = 0; 
	
	resetListToScale(currentScale); 
}

void PatchList::getRemainingPatches(vector<SearchResult>& searchResults, vector<cv::Rect>blackoutRegions, int spatialRadius, int scaleRadius) {
	if (_PATCHLIST_DEBUG) cout << "Get remaining patches." << endl; 
	getRemainingPatches(searchResults); 
	if (_PATCHLIST_DEBUG) cout << "With no blackouts, there were " << searchResults.size() << " remaining." << endl; 
	
	if (blackoutRegions.size() == 0) return; 
	if (_PATCHLIST_DEBUG) cout << "There are " << blackoutRegions.size() << " requested blacked out patches." << endl ;
	
	if (_PATCHLIST_DEBUG) cout << "Getting patches near to blackout area." << endl ;
	vector<SearchResult> blackoutResults; 
	blackoutResults.clear(); 
	for (int i = 0; i < (int)blackoutRegions.size(); i++) {
		if (_PATCHLIST_DEBUG) cout << "Getting search results near blackout region " << i << endl; 
		vector<SearchResult> theseBlackoutResults = getNearbySearchResults(blackoutRegions[i], spatialRadius, scaleRadius); 
		if (_PATCHLIST_DEBUG) cout << "Found " << theseBlackoutResults.size() << " patches near requested region " << i << endl; 
		for (int j = 0; j < (int) theseBlackoutResults.size(); j++) {
			if (theseBlackoutResults[j]._scale == currentScale) // we only need to check the current scale. 
				blackoutResults.push_back(theseBlackoutResults[j]); 
		}
	}
	
	if (_PATCHLIST_DEBUG) cout << "There are " << blackoutResults.size() << " total blacked out patches." << endl ;
	int startSize = searchResults.size(); 
	for (int i = 0; i < (int)searchResults.size(); i++) {
		for (int j = 0; j < (int)blackoutResults.size(); j++) {
			if (searchResults[i]._x == blackoutResults[j]._x && searchResults[i]._y == blackoutResults[j]._y 
				&& searchResults[i]._scale == blackoutResults[j]._scale) {
				searchResults[i] = searchResults.back(); //elem i is getting nuked. Copy the last elem to this elem.
				searchResults.erase(searchResults.end()-1); //then nuke the last elem.
				i= i-1; //have to check swapped element
				break; 
			}
		}
	}
	if (startSize != (int)searchResults.size()) {
		cout << "!!! Blackout removed " << startSize - (int)searchResults.size() << " elements!" << endl; 
	}
}

vector<SearchResult> PatchList::getNearbySearchResults(cv::Rect roi, int spatialRadius, int scaleRadius) {
	
	double center_x = roi.x+roi.width*1.0/2; 
	double center_y = roi.y+roi.height*1.0/2; 
	
	int bestScale = getScaleNearROI(roi, scaleRadius); 
	
	if (_PATCHLIST_DEBUG) cout << "Searching for serach results near (" << roi.x << ", " <<  roi.y << ", " << roi.width << ", " << roi.height << ")" << endl; 
	
	vector<SearchResult> retval; 
	for (int i = bestScale-scaleRadius; i <=bestScale+scaleRadius; i++) {
		if (i < 0 || i >= numScales) continue; 
		
		if (_PATCHLIST_DEBUG) cout << "Searching in scale " << i << " of " << numScales << endl; 
		ImagePatch* currImage = images[i]; 
		SearchResult s; 
		s._scale = i; 
		int x = (int)(center_x-1.0*getPatchSizeAtScale(i).width/2); //
		int y = (int)(center_y-1.0*getPatchSizeAtScale(i).height/2); //
		s.imageLocation = Rect(x, y, getPatchSizeAtScale(i).width, getPatchSizeAtScale(i).height); 
		s.value = 0; 
		if (_PATCHLIST_DEBUG) cout << "patchSize At Scale " << i << " is " << getPatchSizeAtScale(i).width << endl; 
		if (_PATCHLIST_DEBUG) cout << "filterSize At Scale " << i << " is " << getFilterSizeAtScale(i).width << endl; 
		if (_PATCHLIST_DEBUG) cout << "widthRatio is " << pow(scaleinc, i) << endl; 
		double widthRatio = getPatchSizeAtScale(i).width*1.0/ getFilterSizeAtScale(i).width;
		double heightRatio = getPatchSizeAtScale(i).height*1.0/ getFilterSizeAtScale(i).height;
		/*
		double widthRatio = pow(scaleinc,i); 
		double heightRatio = pow(scaleinc,i); 
		if (getPatchSizeAtScale(i).width == getFilterSizeAtScale(i).width || getPatchSizeAtScale(i).height== getFilterSizeAtScale(i).height) {
			widthRatio = 1; 
			heightRatio = 1; 
		} else {
			cout << "Difference is " << (round(x*1.0/widthRatio) - round(x*1.0/(getPatchSizeAtScale(i).width/ getFilterSizeAtScale(i).width))) << endl;
		}*/
		x = round(x*1.0/widthRatio); 
		y = round(y*1.0/heightRatio); 
		Size size = currImage->getImageSize(); 
		
		for (int j = y - spatialRadius; j <= y+spatialRadius; j++) {
			if (j < 0 || j > (size.height-getFilterSizeAtScale(i).height))
				continue; 
			s._y = j; 
			for (int k = x - spatialRadius; k <= x+spatialRadius; k++) {
				if (k < 0 || k > (size.width-getFilterSizeAtScale(i).width))
					continue; 
				s._x = k; 
				retval.push_back(s); 
			}
		}
		
	}
	
	return retval; 
}

int PatchList::getScaleNearROI(cv::Rect roi, int scaleRadius) {
	double roiArea = roi.width*roi.height; 
	double mindiff = INFINITY; 
	Size base = getPatchSizeAtScale(0); 
	double baseArea = base.width*base.height;
	int bestScale=-scaleRadius-1; 
	for (int i = -scaleRadius-1; i <numScales+scaleRadius+1; i++) {
		double scaleArea = baseArea*pow(scaleinc,2*i); 
		double areaDiff = roiArea-scaleArea; 
		areaDiff=(areaDiff<0?-areaDiff:areaDiff); //absolute value
		if (areaDiff<mindiff) { //if best so far, make a record that it's best
			mindiff = areaDiff; 
			bestScale = i; 
		} else {
			//should be monotonic
			break;
		}
	}
	return bestScale; 
}

vector<ImagePatch*> PatchList::getNearbyPatches(cv::Rect roi, int spatialRadius, int scaleRadius) {
	
	vector<ImagePatch*> retval; 
	if (!copyImageData) {
		cout << "WARNING: Trying to getNearbyPatches, but image data was not copied." << endl; 
		return retval; 
	}
	vector<SearchResult> s = getNearbySearchResults(roi, spatialRadius, scaleRadius); 
	if (_PATCHLIST_DEBUG && !s.empty()) {
		cout << "Found search result: " << endl ;
		cout << "-- imageLocation: " << s[0].imageLocation.x << ", " << s[0].imageLocation.y << "; (" << s[0].imageLocation.width << "x" <<  s[0].imageLocation.height << ")" << endl;
		cout << "-- _x           : " << s[0]._x << endl; 
		cout << "-- _y           : " << s[0]._y << endl; 
		cout << "-- _scale       : " << s[0]._scale << endl; 
	}
	for (int i = 0; i < (int)s.size(); i++) {		
		//!!!!! TODO : Check this out. -- Decide if we want this to be size of base patch or size of patch in image
		// As it is, we are grabbing every available pixel.
		Mat patchIm; 
		if (scaleDownPatches) {
			patchIm.create(getBasePatchSize(), CV_8U); 
		}
		else {
			patchIm.create(getFilterSizeAtScale(s[i]._scale), CV_8U); 
		}
		//IplImage patchim = patchIm; 
		fillImageWithPixelsOfSearchPatch(patchIm, s[i]); 
		ImagePatch* patch = new ImagePatch(); 
		patch->setImage(patchIm); 
		retval.push_back(patch); 
	}
	
	return retval; 
}

void PatchList::fillImageWithPixelsOfSearchPatch(IplImage* im, SearchResult r) {
	Mat newIm = im; 
	fillImageWithPixelsOfSearchPatch(newIm, r); 
}

void PatchList::fillImageWithPixelsOfSearchPatch(cv::Mat &image, SearchResult r) {
	
	Size dstsize = (image.rows <= 0)? getBasePatchSize() : image.size(); 
	
	
	if (!copyImageData) {
		cout << "WARNING: Trying to get patch pixels, but image data was not copied." << endl; 
		return ; 
	}
	
	Size currScaleSize = getFilterSizeAtScale(r._scale); //patches->getPatchSizeAtScale(patches->currentScale);
	Mat currImage ; 
	images[r._scale]->getImageHeader(currImage); 
	if (_PATCHLIST_DEBUG) cout << "Getting Patch at " << r._x << ", " << r._y << "; " << currScaleSize.width << "x" << currScaleSize.height << " (scale " << r._scale << "); imsize: " << currImage.cols << "x" << currImage.rows << endl; 
	Mat subImage = currImage(Rect(r._x, r._y, currScaleSize.width, currScaleSize.height)); 
	resize(subImage, image, dstsize, 0, 0, INTER_AREA); 
}

Size PatchList::getMaxEffectivePatchSize() {
	int maxw = maxsize.width; 
	if (maxsize.width == 0 || maxsize.width > origImageSize.width)
		maxw = origImageSize.width; 
	int maxh = maxsize.height; 
	if (maxsize.height == 0 || maxsize.height > origImageSize.height)
		maxh = origImageSize.height; 
	if (_PATCHLIST_DEBUG) cout << "PatchList getMaxEffectivePatchSize: maxw = " << maxw << "; maxh = " << maxh << endl;
	return Size(maxw, maxh); 
}

Size PatchList::getMinEffectivePatchSize() {
	int minw = (minsize.width>0)?minsize.width:default_width; 
	int minh = (minsize.height>0)?minsize.height:default_height; 
	if (_PATCHLIST_DEBUG) cout << "PatchList getMinEffectivePatchSize: minw = " << minw << "; minh = " << minh << endl;
	return Size(minw, minh); 
}

int PatchList::getEffectiveNumScales() {
	Size max_s = getMaxEffectivePatchSize(); 
	Size min_s = getMinEffectivePatchSize(); 
	
	int nscales_w = (log(max_s.width) - log(min_s.width))/log(scaleinc) + 1; 
	int nscales_h = (log(max_s.height) - log(min_s.height))/log(scaleinc)+1; 
	return (nscales_w<nscales_h)?nscales_w:nscales_h; 
}

double PatchList::getBaseWidthScale() {
	Size min_s = getMinEffectivePatchSize(); 
	return 1.0*default_width/min_s.width; 
}

double PatchList::getBaseHeightScale() {
	Size min_s = getMinEffectivePatchSize(); 
	return 1.0*default_height/min_s.height; 
}

double PatchList::getEffectiveBaseWidthRatio(int scale) {
	
	return getBaseWidthScale()/pow(scaleinc, scale) ; 
}

double PatchList::getEffectiveBaseHeightRatio(int scale) {
	return getBaseHeightScale()/pow(scaleinc, scale) ; 
}

Size PatchList::getEffectivePatchSizeAtScale(int scale) {	
	Size min_s = getMinEffectivePatchSize(); 
	double scalemul = pow(scaleinc, scale); 
	return Size(min_s.width*scalemul,min_s.height*scalemul); 
}

Size PatchList::getEffectiveResampledImageSizeAtScale(int scale) {
	
	double baseWidthScale = getEffectiveBaseWidthRatio(scale); // 1.0*default_width/min_s.width; 
	double baseHeightScale = getEffectiveBaseHeightRatio(scale); // 1.0*default_height/min_s.height; 
	int width = origImageSize.width*baseWidthScale/stepwidth;
	int height = origImageSize.height*baseHeightScale/stepwidth;
	return Size(width,height); 
}

int PatchList::setImageAllScalesNeedsPointerReset(IplImage* newImage) {
	
	origImageSize = Size(newImage->width, newImage->height); 	
	oldNumScales = numScales; 
	numScales = getEffectiveNumScales(); 
	
	if (_PATCHLIST_DEBUG) cout<< "Resetting patch list with " << numScales << " scales." << endl; 
		
	if (numScales < 1) {
		cout<< "Warning: There are no scales. Possibly the minimum size is larger than the image?" << endl; 
	}
	double currInc = stepwidth; 
	int needsReset = 0; 
	
	Size scaleSize = getEffectiveResampledImageSizeAtScale(0); 
	
	
	if (scaleCanvas.cols < scaleSize.width ||scaleCanvas.rows < scaleSize.height) {
		scaleCanvas.create(scaleSize, CV_8U); 
		if (_PATCHLIST_DEBUG) cout << "!!! PatchList reset IplImage for resizing image data." << endl; 
	}
	
	for (int i = 0; i < numScales; i++) {
		if (_PATCHLIST_DEBUG) cout << "For scale " << i << " needsReset is so far " << needsReset << endl; 
		
		scaleSize = getEffectiveResampledImageSizeAtScale(i); 
		
        Mat resizeScratchPad = scaleCanvas(Rect(0, 0, scaleSize.width, scaleSize.height));
        Mat newImageMat = newImage;
        cv::resize(newImageMat, resizeScratchPad, resizeScratchPad.size(), 0,0, INTER_NEAREST);
		
		if (_PATCHLIST_DEBUG) cout << "Resized image." << endl; 
		if ((unsigned int) i < images.size()) {	
			if (_PATCHLIST_DEBUG) cout << "Replacing old patch." << endl; 
			const integral_type* previousIntegral = images[i]->startOfIntegralData(); 
			Size size = images[i]->getImageSize(); 
			int previousWidth = size.width; 
			int previousHeight = size.height; 
			images[i]->setImage(resizeScratchPad, copyImageData, 1); 
			
			Size newSize = images[i]->getImageSize(); 
			if (_PATCHLIST_DEBUG) cout << "ImagePatch size is " << newSize.width << "x" << newSize.height << endl; 
			if (previousWidth != newSize.width || previousHeight != newSize.height ||
				previousIntegral != images[i]->startOfIntegralData() ){
				if (_PATCHLIST_DEBUG) {
					if (previousWidth != newSize.width) 
						cout << "Needs reset because image width at scale " << i 
						<< " changed from " << previousWidth << " to " 
						<< newSize.width << endl; 
					if (previousHeight != newSize.height) 
						cout << "Needs reset because image height at scale " << i 
						<< " changed from " << previousHeight << " to " 
						<< newSize.height << endl; 
					if (previousIntegral != images[i]->startOfIntegralData()) 
						cout << "Needs reset because integral pointer at scale " << i 
						<< " changed from " << (void*)previousIntegral << " to " 
						<< (void*)images[i]->startOfIntegralData()<< endl; 
				}
				needsReset = 1; 
			}
		} else {
			if (_PATCHLIST_DEBUG) cout << "Adding new patch to collection." << endl; 
			ImagePatch* patch = new ImagePatch(); 
			patch->setImage(resizeScratchPad, copyImageData, 1); 
			images.push_back(patch); 
			Size newSize = patch->getImageSize(); 
			if (_PATCHLIST_DEBUG) cout << "ImagePatch size is " << newSize.width << "x" << newSize.height << endl; 
			if (_PATCHLIST_DEBUG) cout << "Needs reset because number of scales increased above max seen so far." << endl; 
			needsReset = 1; 
		}
		
		currInc = currInc*scaleinc;
	}
	return needsReset; 
	
}

void PatchList::setNullPointers() {
	if (_PATCHLIST_DEBUG)  cout<< "Set NULL Pointers" << endl; 
	srcInds = NULL; 
	destInds = NULL; 
	accInds = NULL; 
	imLoc = NULL; 
}

void PatchList::clearPointers() {
	if (_PATCHLIST_DEBUG)  cout<< "Clear Pointers" << endl; 
	if (srcInds != NULL) free(srcInds);   //srcInds points to elements in another array, so only the top pointer should be freed
	if (destInds != NULL) free(destInds); //destInds points to elements in another array, so only the top ointer should be freed
	if (accInds != NULL) free(accInds); 
	if (imLoc != NULL) free(imLoc); 
	if (!destsAtScales.empty() ) {
		for (int i = 0; i < oldNumScales; i++) {
			free(destsAtScales[i]); 
			destsAtScales[i] = NULL; 
		}
		destsAtScales.clear();
	}		
	if (!srcAtScales.empty() ) {
		for (int i = 0; i < oldNumScales; i++) {
			free(srcAtScales[i]); 
			srcAtScales[i] = NULL; 
		}		
		srcAtScales.clear();
	}	
	if (!accAtScales.empty()) {
		for (int i = 0; i < oldNumScales; i++) {
			free(accAtScales[i]); 
			accAtScales[i] = NULL; 
		}
		accAtScales.clear();
	}
	if (!imLocAtScales.empty()) {
		for (int i = 0; i < oldNumScales; i++) {
			free(imLocAtScales[i]); 
			imLocAtScales[i] = NULL; 
		}
		imLocAtScales.clear(); 
	}
	setNullPointers(); 
}

void PatchList::getFilterImage(cv::Mat &image) {
	image = filterImage; 
}

void PatchList::getAccumImage(cv::Mat &image) {
	image = accumulatorImage; 
}

void PatchList::resetContainerSizes() {
	if (_PATCHLIST_DEBUG) cout<< "Reset Container Sizes" << endl; 
	clearPointers(); 
	
	if (numScales < 1) {
		cout<< "Warning: There are no scales. Possibly the minimum size is larger than the image?" << endl; 
	}
	
	numAtScale.resize(numScales); 
	scalePatchSizes.resize(numScales); 
	scaleImageSizes.resize(numScales);
	scaleFilterSizes.resize(numScales); 
	srcAtScales.resize(numScales); 
	destsAtScales.resize(numScales); 
	accAtScales.resize(numScales); 
	imLocAtScales.resize(numScales); 
	
	filterImage.create(origImageSize, CV_64F); 
	accumulatorImage.create(origImageSize, CV_64F); 
		
	indImage.create(origImageSize, CV_32S); 	
	int* indim = (int*)indImage.data; 
	int rowStep = origImageSize.width; //indImage->widthStep/sizeof(int); 
	for (int i = 0; i < indImage.rows; i++) {
		int ind = i*rowStep; 
		for (int j = 0; j < indImage.cols; j++) {
			indim[ind+j] = ind+j; 
		}
	}
}

void PatchList::resetPointers() {
	if (_PATCHLIST_DEBUG)  cout<< "PatchList Reset Pointers" << endl; 
	
	resetContainerSizes(); 
		
	double* dest1 = (double*)filterImage.data; 
	double* dest2 = (double*)accumulatorImage.data; 
	int dstWidthStep1 = filterImage.step/sizeof(double); 
	int dstWidthStep2 = accumulatorImage.step/sizeof(double); 
		
	double currWidth = default_width; //minsize.width; 
	double currHeight = default_height; //minsize.height; 
	totalPatches = 0; 
	
	for (int i = 0; i < numScales; i++) {
		Mat scaleInds(images[i]->getImageSize(), CV_32S); 
		resize(indImage, scaleInds, scaleInds.size(), 0,0, INTER_NEAREST); //This is a cool trick
		
		const integral_type* integralData = images[i]->startOfIntegralData(); 
		int* iminds = (int*)scaleInds.data; 
		int indWidthStep = scaleInds.step / sizeof(int); 
		
		Size s = images[i]->getImageSize(); 
		int scalewidth = s.width - default_width + 1; //minsize.width+ 1; 
		int scaleheight = s.height - default_width + 1; // minsize.height + 1; 
		int integralDataRowWidth = images[i]->integralDataRowWidth(); 
		
		numAtScale[i] = scalewidth*scaleheight; 
		scaleImageSizes[i] = Size(scalewidth,scaleheight); 
		scalePatchSizes[i]= getEffectivePatchSizeAtScale(i);//Size((int)currWidth, (int)currHeight);  //PatchSize is size of patch in orig image
		scaleFilterSizes[i]= Size((int)default_width,(int)default_height);//Size((int)minsize.width, (int)minsize.height);  //filterSize is size of patch in downsampled image
		totalPatches+= numAtScale[i]; 
		
		//cout<< "Scale " << i << " expects " << numAtScale[i] << " patches." << endl; 
		
		srcAtScales[i] = (const integral_type**)malloc(numAtScale[i]*sizeof(integral_type*)); 
		destsAtScales[i] = (double**)malloc(numAtScale[i]*sizeof(double*));   
		accAtScales[i] = (double**)malloc(numAtScale[i]*sizeof(double*));  
		imLocAtScales[i] = (int*)malloc(numAtScale[i]*sizeof(int));  
		
		int ind = 0; 
		for (int j = 0; j < scaleheight; j++) {
			int rowInd1 = j*(integralDataRowWidth); 
			int rowInd2 = j*dstWidthStep1; 
			int rowInd3 = j*dstWidthStep2; 
			int rowInd4 = j*indWidthStep; 
			for (int k = 0; k < scalewidth; k++) {
				srcAtScales[i][ind] = &integralData[rowInd1+k];
				destsAtScales[i][ind] = &dest1[rowInd2+k]; 
				accAtScales[i][ind] = &dest2[rowInd3+k];
				imLocAtScales[i][ind] = iminds[rowInd4+k]; 
				ind++; 
			}
		}
		
		//cout<< "Scale " << i << " found " << ind << " patches." << endl; 
		
		currWidth = currWidth*scaleinc; 
		currHeight = currHeight*scaleinc; 
		
	}
	
	//cvReleaseImage(&indImage); 
	
	if (_PATCHLIST_DEBUG) cout<< "In all, " << totalPatches << " patches were listed." << endl; 	
	
	srcInds = (integral_type**)malloc(numAtScale[0]*sizeof(integral_type*)); 
	destInds = (double**)malloc(numAtScale[0]*sizeof(double*)); 
	accInds = (double**)malloc(numAtScale[0]*sizeof(double*)); 
	imLoc = (int*)malloc(numAtScale[0]*sizeof(int)); 
	
	shouldResetAccumulator = 1; 
}

void PatchList::keepOnlyLocalMaxima(int radius) {
	if (_PATCHLIST_DEBUG) cout << "Removing local maxima" << endl ;
	int newrad = (int)(1.0*radius/pow(scaleinc, currentScale)); 
	if (radius == 0) return; 
	int currLast = 0; 
	for (int i = 0; i < currentListLength; i++) {
		if (isLocalMaximum(accInds[i], newrad)) {
			accInds[currLast] = accInds[i]; 
			destInds[currLast] = destInds[i]; 
			srcInds[currLast] = srcInds[i]; 
			imLoc[currLast] = imLoc[i]; 
			currLast++; 
		}
	}
	currentListLength = currLast; 
	shouldResetAccumulator = 1; 
}

int PatchList::isLocalMaximum(double* accIndsPtr, int radius) {
	Point ind = matLocOfImLoc(accIndsPtr); 
	Size imSize = getImageSizeAtScale(); 
	for (int y = ind.y-radius; y <= ind.y+radius; y++) {
		if (y < 0 || y >= imSize.height) continue; 
		for (int x = ind.x-radius; x<=ind.x+radius; x++) {
			if (x < 0 || x >= imSize.width) continue; 
			if (accumulatorImage.at<double>( y, x) > *accIndsPtr)
				return 0; 
		}
	}
	return 1; 		
}

CvPoint PatchList::matLocOfImLoc(double* accIndsPtr) {
	int offset = accIndsPtr-(double*)accumulatorImage.data; 
	int dstWidthStep2 = accumulatorImage.step/sizeof(double); 
	int j = offset/dstWidthStep2; 
	int k = offset%dstWidthStep2; 
	return Point(k,j); 
}

IplImage* PatchList::getProbImage(double faceAreaPrior, double faceInvisibleProb ) {
	Mat dest; 
	getProbImage(dest,faceAreaPrior, faceInvisibleProb); 
	IplImage* retval = cvCreateImage(dest.size(), IPL_DEPTH_64F, 1); 
	Mat destMat = retval; 
	dest.copyTo(destMat); 
	return retval; 
}
	
void PatchList::getProbImage(Mat &dest, double faceAreaPrior, double faceInvisibleProb ) {
	
	if (_PATCHLIST_DEBUG) cout << "Allocating images for getProbImage" << endl; 
	
	Size accsize = getImageSizeAtScale(); 
	Size probsize = accsize; 
	probsize.width = probsize.width+default_width-1; 
	probsize.height = probsize.height+default_height-1; 
	Size ratsize = accsize; 
	ratsize.width = ratsize.width+2*(default_width-1); 
	ratsize.height = ratsize.height+2*(default_height-1); 
	Size intsize = ratsize; 
	intsize.width = intsize.width+1; 
	intsize.height = intsize.height+1; 
	IplImage* ratios = 	cvCreateImage(ratsize, IPL_DEPTH_64F, 1);
	IplImage* priors = cvCreateImage(ratsize, IPL_DEPTH_64F, 1); 
	IplImage* intImage = cvCreateImage(intsize, IPL_DEPTH_64F, 1);
	IplImage* intPriors = cvCreateImage(intsize, IPL_DEPTH_64F, 1); 
	IplImage* sum = cvCreateImage(probsize, IPL_DEPTH_64F, 1); 
	IplImage* priorsum = cvCreateImage(probsize, IPL_DEPTH_64F, 1); 
	IplImage* prob = cvCreateImage(probsize, IPL_DEPTH_64F, 1); 
	
	double min=0, max=0; 
	
	if (_PATCHLIST_DEBUG) cout << "Computing Likelihood Ratios" << endl; 
	cvSetZero(ratios); 
	cvSetZero(priors);
	Rect accROI(0, 0, accsize.width, accsize.height);
	Rect ratROI(default_width-1,default_height-1,accsize.width,accsize.height); 
	
	Mat accRoiIm1 = accumulatorImage(accROI); 
	IplImage accRoiIm = accRoiIm1; 
	
	cvSetImageROI(ratios, ratROI);
	cvSetImageROI(priors, ratROI);
	cvSet(priors,cvRealScalar(1.0));
	cvCopy(&accRoiIm, ratios, NULL);
	cvAdd(&accRoiIm,ratios,ratios);
	
	if (_PATCHLIST_DEBUG) cvMinMaxLoc(ratios, &min, &max); 
	if (_PATCHLIST_DEBUG) cout << "Min: " << min << " ; Max: " << max << endl; 
	
	cvExp(ratios,ratios); 
	
	if (_PATCHLIST_DEBUG) cvMinMaxLoc(ratios, &min, &max); 
	if (_PATCHLIST_DEBUG) cout << "Min: " << min << " ; Max: " << max << endl; 
	cvResetImageROI(ratios); 
	cvResetImageROI(priors);
	
	if (_PATCHLIST_DEBUG) cout << "Computing Integral" << endl; 
	
	cvIntegral(ratios, intImage, NULL, NULL); 
	cvIntegral(priors, intPriors, NULL, NULL); 
	
	
	if (_PATCHLIST_DEBUG) cout << "Computing Box Filter - 1st" << endl; 
	
	cvSetImageROI(intImage, cvRect(0,0,  probsize.width,probsize.height));
	cvSetImageROI(intPriors, cvRect(0,0,  probsize.width,probsize.height));
	cvCopy(intImage,sum); 
	cvCopy(intPriors,priorsum); 
	
	if (_PATCHLIST_DEBUG) cout << "Computing Box Filter - 2nd" << endl; 
	if (_PATCHLIST_DEBUG) cout << "sum has size (" << sum->width << "x" << sum->height << ")" << endl; 
	
	if (_PATCHLIST_DEBUG) cout << "intImage has size (" << intImage->width << "x" << intImage->height << ")" << endl;
	if (_PATCHLIST_DEBUG) cout << "Using ROI " << default_width <<", " << 0 <<", " << probsize.width <<", " << probsize.height << ")" << endl; 
	
	cvSetImageROI(intImage, cvRect(default_width, 0,
								   probsize.width,probsize.height));
	cvSetImageROI(intPriors, cvRect(default_width, 0,
								   probsize.width,probsize.height));
	if (_PATCHLIST_DEBUG) cout << "Computing Box Filter - 2nd" << endl; 
	cvSub(sum,intImage, sum, NULL); 	
	cvSub(priorsum,intPriors, priorsum, NULL); 
	
	if (_PATCHLIST_DEBUG) cout << "Computing Box Filter - 3rd" << endl; 
	
	cvSetImageROI(intImage, cvRect(0, default_height,
								   probsize.width,probsize.height));
	cvSetImageROI(intPriors, cvRect(0, default_height,
								   probsize.width,probsize.height));
	cvSub(sum,intImage, sum, NULL); 
	cvSub(priorsum,intPriors, priorsum, NULL); 
	
	
	if (_PATCHLIST_DEBUG) cout << "Computing Box Filter - 4th" << endl; 
	
	cvSetImageROI(intImage, cvRect(default_width, default_height,
								   probsize.width,probsize.height));
	cvSetImageROI(intPriors, cvRect(default_width, default_height,
								   probsize.width,probsize.height));
	cvAdd(sum,intImage, sum, NULL); 
	cvAdd(priorsum,intPriors, priorsum, NULL); 
	
	if (_PATCHLIST_DEBUG) cvMinMaxLoc(sum, &min, &max); 
	if (_PATCHLIST_DEBUG) cout << "Min: " << min << " ; Max: " << max << endl; 
	
	
	if (_PATCHLIST_DEBUG) cout << "Computing Probability" << endl; 
	
	cvDiv(sum,priorsum,sum,1.0); 
	
	cvAddS(sum, cvRealScalar((1.0-faceAreaPrior)/faceAreaPrior), 
							 prob, NULL); 
	cvDiv(NULL, prob, prob, 1.0); 
	cvMul(sum, prob, prob, 1.0); 
	
	cvSubRS(prob, cvRealScalar(1.0), sum, NULL); 
	cvScaleAdd(sum, cvRealScalar(faceInvisibleProb), prob, prob); 
	
	
	if (_PATCHLIST_DEBUG) cvMinMaxLoc(prob, &min, &max); 
	if (_PATCHLIST_DEBUG) cout << "Min: " << min << " ; Max: " << max << endl; 
	
	cvReleaseImage(&ratios); 
	cvReleaseImage(&intImage); 
	cvReleaseImage(&sum); 
	cvReleaseImage(&priors); 
	cvReleaseImage(&intPriors);
	cvReleaseImage(&priorsum); 
	Mat probMat = prob; 
	probMat.copyTo(dest); 
	cvReleaseImage(&prob) ;
	//return prob; 

}

void PatchList::setDontCopyImageData(int flag) {
	copyImageData = !flag; 
}

void PatchList::setDontScaleDownPatches(int flag) {
	scaleDownPatches = !flag; 
}

