/*
 *  PatchList2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/24/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "PatchList2.h"
#include "DebugGlobals.h"
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std; 
using namespace cv; 

PatchList2::PatchList2()  {
	init(); 
}

PatchList2::PatchList2(Size baseObjectSize, Size minSize, Size maxSize, 
					   double scaleInc, double stepWidth, int dontCopyImage) {
	init(baseObjectSize, minSize, maxSize, scaleInc, stepWidth, dontCopyImage); 
}

PatchList2 & PatchList2::operator=(const PatchList2 &rhs) {
	if (this != &rhs) {
		copy(rhs); 
	}
	return *this; 
}

PatchList2::PatchList2(const PatchList2 &rhs) {
	if (this != &rhs) {
		copy(rhs); 
	} 
}

void PatchList2::copy(const PatchList2 &rhs) {
	init(rhs.defaultSize, rhs.minsize, rhs.maxsize, rhs.scaleinc, rhs. stepwidth, rhs.copyImageData); 
}

void PatchList2::init(Size baseObjectSize, Size minSize, Size maxSize, 
					  double scaleInc, double stepWidth, int dontCopyImage) {
	minsize = minSize; 
	maxsize = maxSize; 
	defaultSize = baseObjectSize; 
	scaleinc = scaleInc; 
	stepwidth = stepWidth;
	copyImageData = !dontCopyImage; 
	
	setNullPointers(); 
	if (minsize.width == 0 || minsize.height == 0) {
		if (_PATCHLIST_DEBUG) cout << "Creating list with size " << defaultSize.width << ", " << defaultSize.height << endl; 
	}
	
	numScales = 0; 
	totalPatches = 0; 
	currentListLength = 0; 
	currentScale = 0 ; 
	shouldResetAccumulator = 1; 
	origImageSize = Size(0,0); 
	scaleDownPatches = 1; 
}

PatchList2::~PatchList2() {
	if (_PATCHLIST_DEBUG) cout<< "Patch List Destuctor" << endl; 
	if (_PATCHLIST_DEBUG) cout<< "Calling clearPointers()" << endl; 
	clearPointers(); 
	
}

void PatchList2::resetListToScale(int scale){
	
	
	if (_PATCHLIST_DEBUG) cout<< "Reset List To Scale " << scale << endl; 
	//if (accInds.empty() || srcInds.empty() || destInds.empty() || imLoc.empty()) {
	//	cout<< "Warning: Must call setImage before setting a PatchList Scale." << endl; 
	//	return; 
	//}
	
	checkAndWarn(scale); 
	
	if (shouldResetAccumulator) {
		accumulatorImage = 0.0; 
	}
	shouldResetAccumulator = 0; 
	if (scale == currentScale && currentListLength == numAtScale[scale])
		return; 
	
	
	if (_PATCHLIST_DEBUG) cout<< "Doing Reset List To Scale " << scale << endl; 
	srcInds = srcAtScales[scale]; 
	destInds = destsAtScales[scale]; 
	accInds = accAtScales[scale]; 
	imLoc = imLocAtScales[scale]; 
	currentListLength = numAtScale[scale]; 
	currentScale = scale;
	
	if (_PATCHLIST_DEBUG) {
		cout << "currentListLength: " << currentListLength << endl; 
		cout << "srcInds size: " << srcInds.size() << " ; destInds.size(): " << destInds.size() << endl;
		cout << "srcInds[0]: " << (void*)srcInds[0] << endl; 
		cout << "images[0].data: " << (void*)images[0].getIntegralHeader().data << endl; 
		cout << "images[1].data: " << (void*)images[1].getIntegralHeader().data << endl; 
	}
}

Size PatchList2::getBasePatchSize() const {
	return defaultSize; 
}

void PatchList2::setBasePatchSize(Size baseSize) {
	defaultSize = baseSize; 	
} 

void PatchList2::getRemainingPatches(vector<SearchResult>& searchResults) const {
	searchResults.clear(); 
	searchResults.resize(currentListLength); 
	const Mat intim = images[currentScale].getIntegralHeader(); 
	int width = intim.cols; 
	
	for (int i = 0; i < currentListLength; i++) {
		searchResults[i].imageLocation.x = imLoc[i]%origImageSize.width; 		
		searchResults[i].imageLocation.y = imLoc[i]/origImageSize.width; 
		searchResults[i].imageLocation.width=scalePatchSizes[currentScale].width;
		searchResults[i].imageLocation.height=scalePatchSizes[currentScale].height;
		searchResults[i].value = *accInds[i]; 
		int offset = (int)(srcInds[i]-(int*)(intim.data)); 
		searchResults[i]._x = offset%(width);
		searchResults[i]._y = offset/(width); 
		searchResults[i]._scale = currentScale; 
		if (_PATCHLIST_DEBUG && searchResults[i]._x < 0) {
			cout << "!!!!!!!! Inconsistency in getRemainingPatches " << i << " of " << currentListLength << endl; 
			cout << "imlocl: " << searchResults[i].imageLocation.x << ", " << searchResults[i].imageLocation.x << ", "
			<< searchResults[i].imageLocation.width << ", " << searchResults[i].imageLocation.height << ";" << endl; 
			cout << "value: " << searchResults[i].value << "; scale " << searchResults[i]._scale << "; "
			<< "; _x: " << searchResults[i]._x << "; _y: " << searchResults[i]._y << endl; 
			cout << "width: " << width << "; curr length: " << currentListLength << endl; 
			cout << "srcInds: " << (void*)srcInds[i] << "; intim.data: " << (int*)intim.data 
			<< "; offset: " << offset << endl; 
		}
	}
}

void PatchList2::accumulateAndRemovePatchesBelowThreshold(double threshold){
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

int PatchList2::getCurrentListLength() const {
	return currentListLength; 
}

int PatchList2::getTotalPatches() const {
	return totalPatches; 
}

int PatchList2::getNumScales() const {
	return numScales; 
}

void PatchList2::checkAndWarn(int scale) const {
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

int PatchList2::getIntegralWidthStepAtScale(int scale) const {
	if (scale == -1)  scale = currentScale; 
	if (_PATCHLIST_DEBUG) cout<< "Get Integral Width Step of Scale " << scale << endl; 
	checkAndWarn(scale); 
	return images[scale].getIntegralHeader().step / sizeof(int) ;// getImageSize().width+1; 
}

Size PatchList2::getPatchSizeAtScale(int scale) const {
	if (scale == -1) 
		scale = currentScale; 
	if (_PATCHLIST_DEBUG) cout<< "Get Patch Size of Scale " << scale << endl; 	
	checkAndWarn(scale); 
	return scalePatchSizes[scale]; 
}

Size PatchList2::getFilterSizeAtScale(int scale) const {
	if (scale == -1) 
		scale = currentScale; 
	if (_PATCHLIST_DEBUG) cout<< "Get Filter Size of Scale " << scale << endl; 
	checkAndWarn(scale); 
	return scaleFilterSizes[scale]; 
}

Size PatchList2::getImageSizeAtScale(int scale) const {
	if (scale == -1) 
		scale = currentScale; 
	if (_PATCHLIST_DEBUG) cout<< "Get Image Size of Scale " << scale << endl; 
	checkAndWarn(scale); 
	return scaleImageSizes[scale]; 
}

void PatchList2::setImage(const Mat &image) {
	if (setImageAllScalesNeedsPointerReset(image)) {
		if (_PATCHLIST_DEBUG) cout << "Pointers need reset." << endl; 
		resetPointers(); 
	}
	
	if (currentScale >= numScales)
		currentScale = 0; 
	
	resetListToScale(currentScale); 
	if (_PATCHLIST_DEBUG) cout << "Set image finished." << endl; 
}

void PatchList2::getRemainingPatches(vector<SearchResult>& searchResults, 
									 const vector<cv::Rect> &blackoutRegions, 
									 int spatialRadius, 
									 int scaleRadius) const {
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
		if (_BLACKOUT_DEBUG) cout << "!!! Blackout removed " 
			<< (startSize - (int)searchResults.size()) << " elements!" << endl; 
	}
}

vector<SearchResult> PatchList2::getNearbySearchResults(cv::Rect roi, 
														int spatialRadius, 
														int scaleRadius) const {
	
	double center_x = roi.x+roi.width*1.0/2; 
	double center_y = roi.y+roi.height*1.0/2; 
	
	int bestScale = getScaleNearROI(roi, scaleRadius); 
	
	if (_PATCHLIST_DEBUG) cout << "Searching for serach results near (" << roi.x << ", " <<  roi.y << ", " << roi.width << ", " << roi.height << ")" << endl; 
	
	vector<SearchResult> retval; 
	for (int i = bestScale-scaleRadius; i <=bestScale+scaleRadius; i++) {
		if (i < 0 || i >= numScales) continue; 
		
		if (_PATCHLIST_DEBUG) cout << "Searching in scale " << i << " of " << numScales << endl; 
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
		
		x = round(x*1.0/widthRatio); 
		y = round(y*1.0/heightRatio); 
		Size size = images[i].getImageSize(); 
		Size filterSize = getFilterSizeAtScale(i); 
		for (int j = y - spatialRadius; j <= y+spatialRadius; j++) {
			if (j < 0 || j > (size.height-filterSize.height))
				continue; 
			s._y = j; 
			for (int k = x - spatialRadius; k <= x+spatialRadius; k++) {
				if (k < 0 || k > (size.width-filterSize.width))
					continue; 
				s._x = k; 
				retval.push_back(s); 
			}
		}
		
	}
	
	return retval; 
}

int PatchList2::getScaleNearROI(cv::Rect roi, int scaleRadius) const {
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

vector<ImagePatch2> PatchList2::getNearbyPatches(cv::Rect roi, 
												 int spatialRadius, 
												 int scaleRadius) const {
	
	vector<ImagePatch2> retval; 
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
		
		fillImageWithPixelsOfSearchPatch(patchIm, s[i]); 
		ImagePatch2 patch;  
		patch.setImage(patchIm); 
		retval.push_back(patch); 
	}
	
	return retval; 
}

void PatchList2::fillImageWithPixelsOfSearchPatch(cv::Mat &image, 
												  const SearchResult &r) const {
	
	Size dstsize = (image.rows <= 0)? getBasePatchSize() : image.size(); 
	
	if (!copyImageData) {
		cout << "WARNING: Trying to get patch pixels, but image data was not copied." << endl; 
		return ; 
	}
	
	Size currScaleSize = getFilterSizeAtScale(r._scale); //patches->getPatchSizeAtScale(patches->currentScale);
	const Mat currImage = images[r._scale].getImageHeader(); 
	if (_PATCHLIST_DEBUG) cout << "Getting Patch at " << r._x << ", " << r._y << "; " << currScaleSize.width << "x" << currScaleSize.height << " (scale " << r._scale << "); imsize: " << currImage.cols << "x" << currImage.rows << endl; 
	Mat subImage = currImage(Rect(r._x, r._y, currScaleSize.width, currScaleSize.height)); 
	resize(subImage, image, dstsize, 0, 0, INTER_AREA); 
}

Size PatchList2::getMaxEffectivePatchSize() const {
	int maxw = maxsize.width; 
	if (maxsize.width == 0 || maxsize.width > origImageSize.width)
		maxw = origImageSize.width; 
	int maxh = maxsize.height; 
	if (maxsize.height == 0 || maxsize.height > origImageSize.height)
		maxh = origImageSize.height; 
	if (_PATCHLIST_DEBUG) cout << "PatchList getMaxEffectivePatchSize: maxw = " << maxw << "; maxh = " << maxh << endl;
	return Size(maxw, maxh); 
}

Size PatchList2::getMinEffectivePatchSize() const {
	int minw = (minsize.width>0)?minsize.width:defaultSize.width; 
	int minh = (minsize.height>0)?minsize.height:defaultSize.height; 
	if (_PATCHLIST_DEBUG) cout << "PatchList getMinEffectivePatchSize: minw = " << minw << "; minh = " << minh << endl;
	return Size(minw, minh); 
}

int PatchList2::getEffectiveNumScales() const {
	Size max_s = getMaxEffectivePatchSize(); 
	Size min_s = getMinEffectivePatchSize(); 
	
	int nscales_w = (log(max_s.width) - log(min_s.width))/log(scaleinc) + 1; 
	int nscales_h = (log(max_s.height) - log(min_s.height))/log(scaleinc)+1; 
	return (nscales_w<nscales_h)?nscales_w:nscales_h; 
}

double PatchList2::getBaseWidthScale() const {
	Size min_s = getMinEffectivePatchSize(); 
	return 1.0*defaultSize.width/min_s.width; 
}

double PatchList2::getBaseHeightScale() const {
	Size min_s = getMinEffectivePatchSize(); 
	return 1.0*defaultSize.height/min_s.height; 
}

double PatchList2::getEffectiveBaseWidthRatio(int scale) const {
	
	return getBaseWidthScale()/pow(scaleinc, scale) ; 
}

double PatchList2::getEffectiveBaseHeightRatio(int scale) const {
	return getBaseHeightScale()/pow(scaleinc, scale) ; 
}

Size PatchList2::getEffectivePatchSizeAtScale(int scale) const {	
	Size min_s = getMinEffectivePatchSize(); 
	double scalemul = pow(scaleinc, scale); 
	return Size(min_s.width*scalemul,min_s.height*scalemul); 
}

Size PatchList2::getEffectiveResampledImageSizeAtScale(int scale) const {
	
	double baseWidthScale = getEffectiveBaseWidthRatio(scale); // 1.0*default_width/min_s.width; 
	double baseHeightScale = getEffectiveBaseHeightRatio(scale); // 1.0*default_height/min_s.height; 
	int width = origImageSize.width*baseWidthScale/stepwidth;
	int height = origImageSize.height*baseHeightScale/stepwidth;
	return Size(width,height); 
}

int PatchList2::setImageAllScalesNeedsPointerReset(const Mat &newImage) {
	
	origImageSize = newImage.size(); 	
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
		resize(newImage, resizeScratchPad, resizeScratchPad.size(), 0,0, INTER_NEAREST); 
		
		if (_PATCHLIST_DEBUG) cout << "Resized image." << endl; 
		if ((unsigned int) i < images.size()) {	
			if (_PATCHLIST_DEBUG) cout << "Replacing old patch." << endl; 
			const int* previousIntegral = (int*) images[i].getIntegralHeader().data; 
			Size size = images[i].getImageSize(); 
			int previousWidth = size.width; 
			int previousHeight = size.height; 
			images[i].setImage(resizeScratchPad, copyImageData, 1);  
			
			Size newSize = images[i].getImageSize(); 
			if (_PATCHLIST_DEBUG) cout << "ImagePatch size is " << newSize.width << "x" << newSize.height << endl; 
			if (previousWidth != newSize.width || previousHeight != newSize.height ||
				previousIntegral != (int*) (images[i].getIntegralHeader().data) ){
				if (_PATCHLIST_DEBUG) {
					if (previousWidth != newSize.width) 
						cout << "Needs reset because image width at scale " << i 
						<< " changed from " << previousWidth << " to " 
						<< newSize.width << endl; 
					if (previousHeight != newSize.height) 
						cout << "Needs reset because image height at scale " << i 
						<< " changed from " << previousHeight << " to " 
						<< newSize.height << endl; 
					if (previousIntegral != (int*)images[i].getIntegralHeader().data) 
						cout << "Needs reset because integral pointer at scale " << i 
						<< " changed from " << (void*)previousIntegral << " to " 
						<< (void*)images[i].getIntegralHeader().data<< endl; 
				}
				needsReset = 1; 
			}
		} else {
			if (_PATCHLIST_DEBUG) cout << "Adding new patch to collection." << endl; 
			ImagePatch2 patch ; 
			patch.setImage(resizeScratchPad, copyImageData, 1); 
			images.push_back(patch); 
			Size newSize = patch.getImageSize(); 
			if (_PATCHLIST_DEBUG) cout << "ImagePatch size is " << newSize.width << "x" << newSize.height << endl; 
			if (_PATCHLIST_DEBUG) cout << "Needs reset because number of scales increased above max seen so far." << endl; 
			needsReset = 1; 
		}
		
		currInc = currInc*scaleinc;
	}
	return needsReset; 
	
}

void PatchList2::setNullPointers() {
	if (_PATCHLIST_DEBUG)  cout<< "Set NULL Pointers" << endl; 
	/*
	srcInds = NULL; 
	destInds = NULL; 
	accInds = NULL; 
	imLoc = NULL; 
	 */
}

void PatchList2::clearPointers() {
	/*
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
	 */
	setNullPointers(); 
}

void PatchList2::getPatchImage(cv::Mat &image) const {
	image = images[currentScale].getImageHeader().clone(); 
}

void PatchList2::getFilterImage(cv::Mat &image) const {
	image = filterImage; 
}

void PatchList2::getAccumImage(cv::Mat &image) const {
	image = accumulatorImage; 
}

void PatchList2::resetContainerSizes() {
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
	//currentListLength = 0; 
}

void PatchList2::resetPointers() {
	if (_PATCHLIST_DEBUG)  cout<< "PatchList Reset Pointers" << endl; 
	
	resetContainerSizes(); 
	
	double* dest1 = (double*)filterImage.data; 
	double* dest2 = (double*)accumulatorImage.data; 
	int dstWidthStep1 = filterImage.step/sizeof(double); 
	int dstWidthStep2 = accumulatorImage.step/sizeof(double); 
	
	double currWidth = defaultSize.width; //minsize.width; 
	double currHeight = defaultSize.height; //minsize.height; 
	totalPatches = 0; 
	
	for (int i = 0; i < numScales; i++) {
		Mat scaleInds(images[i].getImageSize(), CV_32S); 
		resize(indImage, scaleInds, scaleInds.size(), 0,0, INTER_NEAREST); //This is a cool trick
		
		const int* integralData = (int*)(images[i].getIntegralHeader().data); 
		int* iminds = (int*)scaleInds.data; 
		int indWidthStep = scaleInds.step / sizeof(int); 
		
		Size s = images[i].getImageSize(); 
		int scalewidth = s.width - defaultSize.width + 1; //minsize.width+ 1; 
		int scaleheight = s.height - defaultSize.height + 1; // minsize.height + 1; 
		int integralDataRowWidth = images[i].getIntegralHeader().step / sizeof(int); 
		
		numAtScale[i] = scalewidth*scaleheight; 
		scaleImageSizes[i] = Size(scalewidth,scaleheight); 
		scalePatchSizes[i]= getEffectivePatchSizeAtScale(i);//Size((int)currWidth, (int)currHeight);  //PatchSize is size of patch in orig image
		scaleFilterSizes[i]= defaultSize; //Size((int)minsize.width, (int)minsize.height);  //filterSize is size of patch in downsampled image
		totalPatches+= numAtScale[i];  
		
		if (_PATCHLIST_DEBUG) cout<< "Scale " << i << " expects " << numAtScale[i] << " patches." << endl; 
		
		srcAtScales[i].resize(numAtScale[i]); 
		destsAtScales[i].resize(numAtScale[i]);    
		accAtScales[i].resize(numAtScale[i]);   
		imLocAtScales[i].resize(numAtScale[i]);  
		
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
		
		if (_PATCHLIST_DEBUG) cout<< "Scale " << i << " found " << ind << " patches." << endl; 
		
		currWidth = currWidth*scaleinc; 
		currHeight = currHeight*scaleinc; 
		
	}
	
	//cvReleaseImage(&indImage); 
	
	if (_PATCHLIST_DEBUG) cout<< "In all, " << totalPatches << " patches were listed." << endl; 	

	
	shouldResetAccumulator = 1; 
}

void PatchList2::keepOnlyLocalMaxima(int radius) {
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

int PatchList2::isLocalMaximum(const double* accIndsPtr, int radius) const {
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

Point PatchList2::matLocOfImLoc(const double* accIndsPtr) const {
	int offset = accIndsPtr-(double*)accumulatorImage.data; 
	int dstWidthStep2 = accumulatorImage.step/sizeof(double); 
	int j = offset/dstWidthStep2; 
	int k = offset%dstWidthStep2; 
	return Point(k,j); 
}

void PatchList2::getProbImage(Mat &dest, double faceAreaPrior, 
							  double faceInvisibleProb ) const {
	
	if (_PATCHLIST_DEBUG) cout << "Allocating images for getProbImage" << endl; 
	
	Size accsize = getImageSizeAtScale(); 
	Size probsize(accsize.width+defaultSize.width-1, 
					 accsize.height+defaultSize.height-1); 
	Size ratsize(accsize.width+2*(defaultSize.width-1), 
				 accsize.height+2*(defaultSize.height-1)); 
	
	Mat ratios, priors, intImage, intPriors, sum, priorsum, prob; 
	
	double min=0, max=0; 
	
	if (_PATCHLIST_DEBUG) cout << "Computing Likelihood Ratios" << endl; 
	
	ratios = Mat::zeros(ratsize, CV_64F); 
	priors = Mat::zeros(ratsize, CV_64F); 
		
	Rect accROI(0, 0, accsize.width, accsize.height);
	Rect ratROI(defaultSize.width-1,defaultSize.height-1,accsize.width,accsize.height); 
	
	Mat accRoiIm = accumulatorImage(accROI);
	Mat ratRoiIm = ratios(ratROI);
	Mat priRoiIm = priors(ratROI); 
	
	priRoiIm = 1.; 	
	ratRoiIm = 2.*accRoiIm;
	
	if (_PATCHLIST_DEBUG) minMaxLoc(ratRoiIm, &min, &max); 
	if (_PATCHLIST_DEBUG) cout << "Min: " << min << " ; Max: " << max << endl; 
	
	exp(ratRoiIm,ratRoiIm); 
	
	if (_PATCHLIST_DEBUG) minMaxLoc(ratRoiIm, &min, &max); 
	if (_PATCHLIST_DEBUG) cout << "Min: " << min << " ; Max: " << max << endl; 

	if (_PATCHLIST_DEBUG) cout << "Computing Integral" << endl; 
	
	integral(ratios, intImage); 
	integral(priors, intPriors);
	
	if (_PATCHLIST_DEBUG) cout << "Computing Box Filter" << endl; 
		
	Rect r1 = Rect(0,0,  probsize.width,probsize.height);
	Rect r2 = Rect(defaultSize.width, 0, probsize.width,probsize.height); 
	Rect r3 = Rect(0, defaultSize.height, probsize.width,probsize.height); 
	Rect r4 = Rect(defaultSize.width, defaultSize.height, probsize.width,probsize.height);
	
	sum = intImage(r1) - intImage(r2) - intImage(r3) + intImage(r4);
	priorsum = intPriors(r1) - intPriors(r2) - intPriors(r3) + intPriors(r4);
		
	if (_PATCHLIST_DEBUG) minMaxLoc(sum, &min, &max); 
	if (_PATCHLIST_DEBUG) cout << "Min: " << min << " ; Max: " << max << endl; 
	
	
	if (_PATCHLIST_DEBUG) cout << "Computing Probability" << endl; 
	
	divide(sum, priorsum, sum); 
	prob = (1.0-faceAreaPrior)/faceAreaPrior + sum; 
	divide(sum, prob, prob); 
	subtract(1., prob, sum); 
	scaleAdd(sum, faceInvisibleProb, prob, prob); 
	
	
	if (_PATCHLIST_DEBUG) minMaxLoc(prob, &min, &max); 
	if (_PATCHLIST_DEBUG) cout << "Min: " << min << " ; Max: " << max << endl; 
	 
	prob.copyTo(dest); 
	//return prob; 
	
}

void PatchList2::setDontCopyImageData(int flag) {
	copyImageData = !flag; 
}

void PatchList2::setDontScaleDownPatches(int flag) {
	scaleDownPatches = !flag; 
}

