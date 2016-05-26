/*
 *  PatchDataset.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/11/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */



#include "PatchDataset.h"

#include <math.h>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include "DetectionEvaluator.h"
#include "OpenLoopPolicies.h"
#include "FastPatchList.h"
#include "NMPTUtils.h"

using namespace std; 
using namespace cv;

int PatchDataset::useFast = 1; 
int PatchDataset::minPerNegIm = 100; 


PatchDataset::PatchDataset(Size patchsize,
						   const std::string &positiveImageListFilename, 
						   const std::string &positiveImageLabelsFilename,
						   const std::string &negativeImageListFilename ,
						   int posExamplePatchRadius,
						   int posExampleScaleRadius,
						   int numPosPatches, 
						   int numNegPatches,
						   double scale) {
	
	
	if (useFast)
		list = new FastPatchList(); 
	else
		list = new PatchList(); 
	
	rng = cvRNG(BlockTimer::getCurrentTimeInMicroseconds());
	
	posImagesFileList = positiveImageListFilename; 
	posImagesLabelsList = positiveImageLabelsFilename; 
	negImagesFileList = negativeImageListFilename; 
	
	posImageDataset = NULL; 
	negImageDataset = NULL; 
	evaluator = NULL; 
	
	numPosImages = 0; 
	numNegImages = 0; 
	posImagesHaveLabels = 0; 
	
	patches.clear(); 
	labels.clear(); 
	posPatches.clear(); 
	negPatches.clear(); 

	
	patchSize = patchsize; 
	
	if (posImagesFileList.empty() && negImagesFileList.empty() ) return; 
	
	if (!posImagesFileList.empty()) {
		getPosImagesDataset(); 
		//posImageDataset = ImageDataSet::loadFromFile(positiveImageListFilename,
		//											 positiveImageLabelsFilename); 
		numPosImages = posImageDataset->getNumEntries(); 
		posImagesHaveLabels = !posImagesLabelsList.empty(); 
	}
	
	if (!negImagesFileList.empty()) {
		negImageDataset = ImageDataSet::loadFromFile(negImagesFileList.c_str()); 
		numNegImages = negImageDataset->getNumEntries(); 
	}
	
	
	//IplImage* image = NULL;
	Mat image; 
	
	if (posImagesHaveLabels) {
		for (int patchNum = 0; patchNum < numPosImages; patchNum++) {
			int i = patchNum%numPosImages; 
			cout << "Getting patch from positive example " << i << endl; 
			if (i == 0 || strncmp(posImageDataset->getFileName(i).c_str(), posImageDataset->getFileName(i-1).c_str(), 5000)) {
				//cvReleaseImage(&image); 
				image = imread(posImageDataset->getFileName(i), 0);//cvLoadImage( posImageDataset->getFileName(i).c_str());
				list->setImage(image); 
				//cvReleaseImage(&grayImage); 
				
			}
			vector<double> loc = posImageDataset->getFileLabels(i); 
			
			if (loc[2] <= 0) continue; 
			
			double d = loc[2]*scale; 
			
			
			int x = loc[0]-d/2; 
			int y = loc[1]-d/2; 
			Rect ROI = cvRect(x,y,d, d); 
			if (ROI.x < 0) ROI.x = 0; 
			if (ROI.y < 0) ROI.y = 0; 
			if (ROI.x+ROI.width > image.cols) ROI.width=image.cols-ROI.x; 
			if (ROI.y+ROI.height > image.rows) ROI.height=image.rows-ROI.y; 
			vector<ImagePatch*> objPatches = list->getNearbyPatches(ROI,
																	posExamplePatchRadius,
																	posExampleScaleRadius); 
			if (!objPatches.empty()) {
				for (unsigned int z = 0; z < objPatches.size(); z++) {
					patches.push_back(objPatches[z]); 
					labels.push_back(1); 
					posPatches.push_back(patches.back()); 
				}
			}
		}
	} else {
	}
	
	if (numPosPatches > -1 && numPosPatches < (int)patches.size())
		patches.resize(numPosPatches); 
	
	numPosPatches = patches.size(); 
	
	if (numNegImages < 1) {
		//loadEvaluator(); 
		
		numNegImages = getNumUniquePositiveImages(); 
		
		//if (negImageDataset != NULL) delete(negImageDataset); 
		//negImageDataset = posImageDataset; 
		if (numNegPatches < 0 && numNegImages > 0) numNegPatches = numPosPatches; 
		cout << "Found " << numPosPatches << " positive patches." << endl ;
		cout << "Looking for " << numNegPatches << " negative patches." << endl ;
				
		int numTotal = numPosPatches+numNegPatches; 
		
		int ind = numPosPatches; 
		int patchesPerNegIm = ceil( (numTotal-numPosPatches)*1.0 / numNegImages); 
		
		patchesPerNegIm = patchesPerNegIm > minPerNegIm ? patchesPerNegIm : minPerNegIm; 
		
		cout << "Getting " << numTotal- numPosPatches 
		<< " negative patches from " << numNegImages << " images with " 
		<< patchesPerNegIm << " patches per image." << endl; 
		
		for (int i = 0; i < numNegImages; i++) {
			cout << "Getting patches from negative example " << i << endl; 
			cout << "Loading neg image " << getUniquePosImageName(i) << endl; 
			if (ind >= numTotal) break; 
			if (i == 0 || strncmp(getUniquePosImageName(i).c_str(), getUniquePosImageName(i-1).c_str(), 5000)) {
				cout << "Releasing old image" << endl; 
				//cvReleaseImage(&image); 
				cout << "Creating new image" << endl; 
				image = imread(getUniquePosImageName(i), 0); //0 means CV_LOAD_IMAGE_GRAYSCALE
				//image = cvLoadImage( getUniquePosImageName(i).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
			}
			
			//cout << "Getting Random Patches" << endl; 
			IplImage oldIm = image; 
			vector<ImagePatch*> impatches = getRandomPatches(patchesPerNegIm,&oldIm,patchSize, getObjectLocationsInPosImage(i)); 
			//cout << "Got " << impatches.size() << " random patches." << endl; 
			
			for (int j = 0; j < patchesPerNegIm; j++) {
				if (ind >= numTotal) break; 
				//cout << "Getting Random Patch Number " << ind << endl; 
				/*
				 patches[ind] = getRandomPatch(image,patchSize); 
				 negPatches[ind-numPosImages] = patches[ind]; 
				 labels[ind] = -1; 
				 */
				//patches.push_back(getRandomPatch(image,patchSize, getObjectLocationsInPosImage(i))); 
				patches.push_back(impatches[j]); 
				//negPatches.push_back(patches.back()); 
				negPatches.push_back(impatches[j]); 
				labels.push_back(-1); 
				ind++; 
			}
		}	
		//cvReleaseImage(&image); 
	} 	 else {
		if (numNegPatches < 0 && numNegImages > 0) numNegPatches = numPosPatches; 
		
		
		cout << "Found " << numPosPatches << " positive patches." << endl ;
		cout << "Looking for " << numNegPatches << " negative patches." << endl ;
		
		
		int numTotal = numPosPatches+numNegPatches; 
		
		int ind = numPosPatches; 
		int patchesPerNegIm = ceil( (numTotal-numPosPatches)*1.0 / numNegImages); 
		patchesPerNegIm = patchesPerNegIm > minPerNegIm ? patchesPerNegIm : minPerNegIm; 
		
		cout << "Getting " << numTotal- numPosPatches 
		<< " negative patches from " << numNegImages << " images with " 
		<< patchesPerNegIm << " patches per image." << endl; 
		
		for (int i = 0; i < numNegImages; i++) {
			cout << "Getting patches from negative example " << i << endl; 
			//cout << "Loading neg image " << negImageDataset->getFileName(i) << endl; 
			if (ind >= numTotal) break; 
			if (i == 0 || strncmp(negImageDataset->getFileName(i).c_str(), negImageDataset->getFileName(i-1).c_str(), 5000)) {
				//cvReleaseImage(&image); 
				image = imread( negImageDataset->getFileName(i), 0); //CV_LOAD_IMAGE_GRAYSCALE);
			}
			for (int j = 0; j < patchesPerNegIm; j++) {
				if (ind >= numTotal) break; 
				//cout << "Getting Random Patch Number " << ind << endl; 
				/*
				 patches[ind] = getRandomPatch(image,patchSize); 
				 negPatches[ind-numPosImages] = patches[ind]; 
				 labels[ind] = -1; 
				 */
				vector<Rect> r; 
				r.clear(); 
				IplImage oldIm = image; 
				patches.push_back(getRandomPatch(&oldIm,patchSize,r)); 
				negPatches.push_back(patches.back()); 
				labels.push_back(-1); 
				ind++; 
			}
		}	
	}
	
}


PatchDataset::~PatchDataset() {
	
	if (negImageDataset != NULL && negImageDataset != posImageDataset) {
		//cout << "Freeing negImageDataset" << endl; 
		delete(negImageDataset); 
	}
	if (posImageDataset != NULL) {
		//cout << "Freeing posImageDataset" << endl; 
		delete(posImageDataset); 
	}
	if (evaluator != NULL) {
		//cout << "Freeing evaluator" << endl; 
		delete(evaluator) ; 
	}
	for (unsigned int i = 0; i < patches.size(); i++) {
		if (patches[i] != NULL) {
			//cout << "Freeing patches[" << i << "] / " << patches.size()<< endl; 
			delete(patches[i]); 
		}
	}
	if (list != NULL) {
		//cout << "Freeing list." << endl; 
		delete(list); 
	}
}


/*This is a stopgap method to get off the ground. It gets the positive
 examples from the image data set, and as many negative examples as 
 positive examples, culled randomly from the negative data set*/
Size PatchDataset::getPatchSize(){
	 return patchSize; 
}


vector<ImagePatch*> PatchDataset::getPatches(){
	return patches; 
}


vector<ImagePatch*> PatchDataset::getPosPatches(){
	return posPatches; 
}


vector<ImagePatch*> PatchDataset::getNegPatches(){
	return negPatches; 
}

void PatchDataset::getLabels(Mat &dest) {
	int numTotal = labels.size(); 
	dest.create(numTotal, 1, CV_64F); 
	for (int i = 0; i < numTotal; i++) 
		dest.at<double>(i,0) = labels[i]; 
}

CvMat* PatchDataset::getLabels() {
	int numTotal = labels.size(); 
	CvMat* retval = cvCreateMat(numTotal, 1, CV_64FC1); 
	for (int i = 0; i < numTotal; i++) {
		cvSetReal2D(retval, i, 0, labels[i]); 
	}
	return retval; 
}


ImageDataSet* PatchDataset::getNegImagesDataset() {
	if (negImageDataset !=NULL && negImageDataset != posImageDataset) 
		return negImageDataset; 
	else if (negImagesFileList.empty())
		return NULL; 
	else {
		negImageDataset = ImageDataSet::loadFromFile(negImagesFileList.c_str()); 
		return negImageDataset; 
	}
}


ImageDataSet* PatchDataset::getPosImagesDataset() {
	if (posImageDataset !=NULL) 
		return posImageDataset; 
	else if (posImagesFileList.empty())
		return NULL; 
	else {
		if (posImagesLabelsList.empty()) {
			posImageDataset = ImageDataSet::loadFromFile(posImagesFileList.c_str());
		} else {
			posImageDataset = ImageDataSet::loadFromFile(posImagesFileList.c_str(), 
														 posImagesLabelsList.c_str());
			if (evaluator == NULL) 
				evaluator = new DetectionEvaluator(posImageDataset);
		}
		return posImageDataset; 
	}
}


int PatchDataset::loadEvaluator() {
	getPosImagesDataset(); 
	return (evaluator!=NULL) ;
}


int PatchDataset::getNumUniquePositiveImages() {
	if (!loadEvaluator()) return -1; 
	return evaluator->getFileNames().size(); 
}


string PatchDataset::getUniquePosImageName(int num) {
	int numIms = getNumUniquePositiveImages(); 
	if (numIms<=0 || num < 0 || num >= numIms) return "";
	return evaluator->getFileNames()[num]; 
}


vector<Rect> PatchDataset::getObjectLocationsInPosImage(int num) {
	vector<Rect> r; 
	r.clear();
	int numIms = getNumUniquePositiveImages(); 
	if (numIms<=0 || num < 0 || num >= numIms) return r;
	return evaluator->getTargetLocations()[num]; 
	
}


string PatchDataset::getNegImagesFileName() {
	return negImagesFileList; 
}


string PatchDataset::getPosImagesFileName(){
	return posImagesFileList; 
} 


string PatchDataset::getPosLabelsFileName(){
	return posImagesLabelsList; 
} 


ImagePatch* PatchDataset::getPatch(IplImage* image, Size patchSize, CvPoint objCenter, Size objSize) {
	
	Mat newim = image; 
	Mat grayImage; 
	//IplImage* grayImage = cvCreateImage(cvSize(image->width, image->height), 
	//									IPL_DEPTH_8U, 1); 
	cvtColor(newim,grayImage,CV_RGB2GRAY); 
	list->setImage(grayImage); 
	//cvReleaseImage(&grayImage); 
	
	Mat patchImage(patchSize, CV_8U); 
	int x = objCenter.x-objSize.width/2; 
	int y = objCenter.y-objSize.height/2; 
	Rect ROI(x,y,objSize.width, objSize.height); 
	if (ROI.x < 0) ROI.x = 0; 
	if (ROI.y < 0) ROI.y = 0; 
	if (ROI.x+ROI.width > newim.cols) ROI.width=newim.cols-ROI.x; 
	if (ROI.y+ROI.height > newim.rows) ROI.height=newim.rows-ROI.y; 
	//Mat grayROI(Size(ROI.width, ROI.height), CV_8U);
	
	vector<ImagePatch*> patches = list->getNearbyPatches(ROI,0,0); 
	if (!patches.empty())
		return patches[0]; 
	
	resize(grayImage(ROI), patchImage, patchImage.size(), 0, 0, INTER_NEAREST); 
	
	//cvSetImageROI(image, ROI) ; 
	//cvCvtColor(image, grayROI, CV_RGB2GRAY); 
	//cvResize(grayROI, patchImage, CV_INTER_NN); 
	ImagePatch* retval = new ImagePatch(); 
	//Mat newPatchImage = patchImage; 
	retval->setImage(patchImage, 1, 1); 
	//cvReleaseImage(&grayROI); 
	//cvReleaseImage(&patchImage); 
	//cvResetImageROI(image); 
	return retval; 
}


ImagePatch* PatchDataset::getRandomPatch(IplImage* image, Size patchSize, vector<Rect> blackoutList){
	//cout << "Getting random patch from image " << (void*) image << " of size " << patchSize.width << "x" << patchSize.height << " with " << blackoutList.size() << " blackouts."  << endl; 
	ImagePatch* retval = new ImagePatch(); 
	//IplImage* patch_image = cvCreateImage(patchSize, IPL_DEPTH_8U, 1); 
	Mat patch_image(patchSize, CV_8U); 
	
	Mat newIm = image; 
	//PatchList list; 
	//cout << "Created patch list; setting image." << endl; 
	list->setImage(newIm);  
	vector<SearchResult> scaleResults; 
	vector<SearchResult> allResults; 
	allResults.clear(); 
	for (int s = 0; s < list->getNumScales(); s++) {
		list->resetListToScale(s);
		//cout << "---------------------------" << endl; 
		//cout << "Filtering at scale " << s <<  endl; 
		scaleResults.clear(); 
		list->getRemainingPatches(scaleResults, blackoutList, patchSize.width*.5, 2);
		//cout << "Got " << scaleResults.size()  << " patches back. " << endl; 
		allResults.insert(allResults.end(), scaleResults.begin(), scaleResults.end()); 
		
		//cout << "Now there are " << allResults.size()  << " patches total. " << endl; 
		
	}
	
	int locnum = (int)(OpenLoopPolicies::randomFloat()*allResults.size());
	
	//cout << "Picking patch number " << locnum << "/" << allResults.size() << endl; 
	list->fillImageWithPixelsOfSearchPatch(patch_image, allResults[locnum]); 
	
	//cout << "Got patch number " << locnum << endl; 
	//Mat newPatchImage = patch_image; 
	retval->setImage(patch_image, 1, 1); 
	//cvReleaseImage(&patch_image); 
	return retval; 
}

vector<ImagePatch*> PatchDataset::getRandomPatches(int numPatches, IplImage* image, Size patchSize, vector<Rect> blackoutList){
	//cout << "Getting random patch from image " << (void*) image << " of size " << patchSize.width << "x" << patchSize.height << " with " << blackoutList.size() << " blackouts."  << endl; 
	
	Mat patch_image(patchSize, CV_8U); 
	//IplImage* patch_image = cvCreateImage(patchSize, IPL_DEPTH_8U, 1); 
	//PatchList list; 
	//cout << "Created patch list; setting image." << endl; 
	//Mat newImage = image; 
	Mat newIm = image; 
	list->setImage(newIm);  
	vector<SearchResult> scaleResults; 
	vector<SearchResult> allResults; 
	allResults.clear(); 
	for (int s = 0; s < list->getNumScales(); s++) {
		list->resetListToScale(s);
		//cout << "---------------------------" << endl; 
		//cout << "Filtering at scale " << s <<  endl; 
		scaleResults.clear(); 
		list->getRemainingPatches(scaleResults, blackoutList, patchSize.width*.5, 2);
		//cout << "Got " << scaleResults.size()  << " patches back. " << endl; 
		allResults.insert(allResults.end(), scaleResults.begin(), scaleResults.end()); 
		
		//cout << "Now there are " << allResults.size()  << " patches total. " << endl; 
		
	}
	
	vector<ImagePatch*> retvec; 
	retvec.clear(); 
	
	for (int i = 0; i < numPatches; i++) {
		
		int locnum = (int)(NMPTUtils::randomFloat()*allResults.size());
		
		//cout << "Picking patch number " << locnum << "/" << allResults.size() << endl; 
		list->fillImageWithPixelsOfSearchPatch(patch_image, allResults[locnum]); 
		
		//cout << "Got patch number " << i << endl; 
		ImagePatch* retval = new ImagePatch(); 
		//Mat patchImage = patch_image; 
		retval->setImage(patch_image, 1, 1); 
		retvec.push_back(retval); 
	}
	
	//cvReleaseImage(&patch_image); 

	return retvec; 
	/*
	 patch->setImage(patch_image); 
	 
	 
	 for (int f = 0; f < booster->getNumFeaturesUsed(); f++) {
	 Feature* feat = booster->getFeatureAndTuningCurveNumber(f)->getFeature(); 
	 feat->filterPatchList(list); 
	 double patchval = feat->evaluateImagePatch(patch); 
	 double listval = *(list->destInds[locnum]); 
	 cout << "Feature " << f << " - List Method: " << listval 
	 << " ; \tPatch Method: " << patchval << " ; \tDiff: "
	 << listval-patchval << endl; 
	 if (listval-patchval != 0) cout << " **** " << feat->debugInfo() << endl; 
	 }			
	 cvShowImage(WINDOW_NAME, patch_image);
	 //boxes.clear(); 
	 
	 
	 if(cvWaitKey(500) == 'q') return 0;
	 }
	 */
	
	/*
	 int x = cvRandReal(&rng)*(image->width-patchSize.width); 
	 int y = cvRandReal(&rng)*(image->height-patchSize.height); 
	 int maxw = image->width-x-patchSize.width; 
	 int maxh = image->height - y-patchSize.height; 
	 
	 if (maxw > maxh*1.0*patchSize.width/patchSize.height) {
	 maxw = maxh*1.0*patchSize.width/patchSize.height; 
	 } else {
	 maxh = maxw*1.0*patchSize.height/patchSize.width; 
	 }
	 double areascale = cvRandReal(&rng); 
	 int w = areascale*maxw+patchSize.width; 
	 int h = areascale*maxh+patchSize.height; 
	 
	 //cout << "Getting patch @ (" << x << ", " << y <<") of size " << w << "x" << h <<" in " <<image->width<<"x"<<image->height << " image." << endl; 
	 
	 IplImage* grayROI = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1); 
	 IplImage* patchImage = cvCreateImage(patchSize, IPL_DEPTH_8U, 1); 
	 ImagePatch* retval = new ImagePatch(); 
	 //cout << "Converting to Gray" << endl; 
	 cvSetImageROI(image, cvRect(x,y,w,h)) ; 
	 cvCvtColor(image, grayROI, CV_RGB2GRAY); 
	 //cout << "Resizeing" << endl; 
	 cvResize(grayROI, patchImage, CV_INTER_NN); 
	 //cout << "Creating Image Patch" << endl; 
	 retval->setImage(patchImage, 1, 1); 
	 
	 //cout <<"Releasing images." << endl; 
	 cvReleaseImage(&grayROI); 
	 cvReleaseImage(&patchImage); 
	 cvResetImageROI(image); 
	 
	 //cout << "Done." << endl; 
	 return retval; 
	 */
}


void PatchDataset::addToStream(ostream& out) {
	out << patchSize.width << " " << patchSize.height << endl; 
	out << patches.size() << endl; 
	for (unsigned int i = 0; i < patches.size(); i++) {
		out << patches[i]; 
	}
	out << labels.size() << endl; 
	for (unsigned int i = 0; i < labels.size(); i++) {
		out << labels[i] << endl; 
	}
	out << posImagesFileList.size() << endl; 
	for (unsigned int i = 0; i < posImagesFileList.size(); i++) {
		out << posImagesFileList[i]; 
	}
	out << endl; 
	out << posImagesLabelsList.size() << endl; 
	for (unsigned int i = 0; i < posImagesLabelsList.size(); i++) {
		out << posImagesLabelsList[i]; 
	}
	out << endl; 
	out << negImagesFileList.size() << endl; 
	for (unsigned int i = 0; i < negImagesFileList.size(); i++) {
		out << negImagesFileList[i]; 
	}
	out << endl; 	
}


void PatchDataset::readFromStream(istream& in) {
	int size; 
	in >> patchSize.width >> patchSize.height; 
	in >> size; 
	patches.resize(size, (ImagePatch*)NULL); 
	for (int i = 0; i < size; i++) {
		patches[i] = new ImagePatch(); 
		in >> patches[i]; 
	}
	in >> size; 
	int nPos = 0; 
	int nNeg = 0; 
	labels.resize(size, 0); 
	for (int i = 0; i < size; i++) {
		in >> labels[i]; 
		if (labels[i] == 1) nPos++; 
		else if (labels[i] == -1) nNeg++; 
	}
	posPatches.resize(nPos, (ImagePatch*)NULL); 
	negPatches.resize(nNeg, (ImagePatch*)NULL); 
	int pInd = 0; 
	int nInd = 0; 
	for (int i = 0; i < size; i++) {
		if (labels[i] == 1) posPatches[pInd++] = patches[i]; 
		else if (labels[i] == -1) negPatches[nInd++] = patches[i]; 
	}
	in >> size; 
	posImagesFileList.resize(size); 
	for (int i = 0; i < size; i++) {
		in >> posImagesFileList[i]; 
	}
	in >> size; 
	posImagesLabelsList.resize(size); 
	for (int i = 0; i < size; i++) {
		in >> posImagesLabelsList[i]; 
	}
	in >> size; 
	negImagesFileList.resize(size); 
	for (int i = 0; i < size; i++) {
		in >> negImagesFileList[i]; 
	}
	
	posImageDataset = NULL; 
	negImageDataset = NULL; 
	//getPosImagesDataset(); 
}


void PatchDataset::addToStreamBinary(ostream& out)  {
	out.write((char*)&(patchSize.width), sizeof(int)); 
	out.write((char*)&(patchSize.height), sizeof(int)); 
	int size = patches.size(); 
	out.write((char*)&size, sizeof(int));
	for (unsigned int i = 0; i < patches.size(); i++) {
		patches[i]->addToStreamBinary(out); 
	}	
	size = labels.size(); 
	out.write((char*)&size, sizeof(int));
	for (unsigned int i = 0; i < labels.size(); i++) {
		out.write((char*)&(labels[i]), sizeof(int)); 
	}
	size = posImagesFileList.size(); 
	out.write((char*)&size, sizeof(int));
	for (unsigned int i = 0; i < posImagesFileList.size(); i++) {
		out.write(&(posImagesFileList[i]), sizeof(char)); 
	}
	size = posImagesLabelsList.size(); 
	out.write((char*)&size, sizeof(int));
	for (unsigned int i = 0; i < posImagesLabelsList.size(); i++) {
		out.write(&(posImagesLabelsList[i]), sizeof(char)); 
	}
	size = negImagesFileList.size(); 
	out.write((char*)&size, sizeof(int));
	for (unsigned int i = 0; i < negImagesFileList.size(); i++) {
		out.write(&(negImagesFileList[i]), sizeof(char)); 
	}
}


void PatchDataset::readFromStreamBinary(istream& in) {
	int size; 
	in.read((char*)&(patchSize.width),sizeof(int)); 
	in.read((char*)&(patchSize.height),sizeof(int)); 
	in.read((char*)&size,sizeof(int)); 
	patches.resize(size, (ImagePatch*)NULL); 
	for (int i = 0; i < size; i++) {
		patches[i] = new ImagePatch(); 
		patches[i]->readFromStreamBinary(in); 
	}	
	int nPos = 0; 
	int nNeg = 0;	
	in.read((char*)&size,sizeof(int)); 
	labels.resize(size, 0); 
	for (int i = 0; i < size; i++) {
		in.read((char*)&(labels[i]),sizeof(int)); 
		if (labels[i] == 1) nPos++; 
		else if (labels[i] == -1) nNeg++; 
	}
	posPatches.resize(nPos, (ImagePatch*)NULL); 
	negPatches.resize(nNeg, (ImagePatch*)NULL); 
	int pInd = 0; 
	int nInd = 0; 
	for (int i = 0; i < size; i++) {
		if (labels[i] == 1) posPatches[pInd++] = patches[i]; 
		else if (labels[i] == -1) negPatches[nInd++] = patches[i]; 
	}
	in.read((char*)&size,sizeof(int)); 
	posImagesFileList.resize(size); 
	for (int i = 0; i < size; i++) {
		in.read(&(posImagesFileList[i]), sizeof(char));  
	}
	in.read((char*)&size,sizeof(int)); 
	posImagesLabelsList.resize(size); 
	for (int i = 0; i < size; i++) {
		in.read(&(posImagesLabelsList[i]), sizeof(char));  
	}
	in.read((char*)&size,sizeof(int)); 
	negImagesFileList.resize(size); 
	for (int i = 0; i < size; i++) {
		in.read(&(negImagesFileList[i]), sizeof(char));  
	}
	posImageDataset = NULL; 
	negImageDataset = NULL; 
}


void PatchDataset::writeToFile(const string &filename) {
	ofstream out;
    out.open(filename.c_str(), ios::binary);
	addToStreamBinary(out); 
	out.close(); 
}


PatchDataset* PatchDataset::readFromFile(const string &filename) {
	ifstream in;
    in.open (filename.c_str(), ios::binary);
	PatchDataset* dataset = new PatchDataset(cvSize(0,0)); 
	dataset->readFromStreamBinary(in); 
	
	Size patchsize = dataset->getPatchSize(); 
	if (patchsize.width > 0 && patchsize.height > 0)  {
		//PatchList::default_width = patchsize.width; 
		//PatchList::default_height= patchsize.height; 
	}
	
	in.close(); 
	return dataset; 
}


ostream& operator<< (ostream& ofs, PatchDataset* a) {
	a->addToStream(ofs);
	return ofs; 
}


istream& operator>> (istream& ifs, PatchDataset* a) {
	a->readFromStream(ifs);
	return ifs; 
}
