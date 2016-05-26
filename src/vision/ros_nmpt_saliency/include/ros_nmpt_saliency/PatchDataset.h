/*
 *  PatchDataset.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/11/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef PATCHDATASET_H
#define PATCHDATASET_H


#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>

#include <iostream>
#include <vector>
#include <string>

#include "BlockTimer.h"
#include "ImageDataSet.h"
#include "ImagePatch.h"
#include "PatchList.h" 

#include "DebugGlobals.h"

class DetectionEvaluator; 

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A tool for managing the data used in 
 * training GentleBoost classifiers. 
 *
 * The primary purpose of this class is to
 * quickly save and load collections of pathches for training and testing
 * classifiers. However, it is also useful for remembering the disk location
 * of the dataset files used to construct a dataset.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class PatchDataset {
public:
	/**
	 * \brief Constructor. 
	 *
	 * @param patchsize Basic size of patches to use for training. When 
	 * constructing a PatchDataset, this must be specified. When reading
	 * a PatchDataset from an istream, any size may be initially specified,
	 * which will be subsequently overwritten.
	 * @param positiveImageListFilename Path to a file containing a list of
	 * files, one per line, that contain the object you're trying to learn
	 * to detect. If the same file contains more than one of the object, it
	 * can appear on more than one line. 
	 * @param positiveImageLabelsFilename Path to a file containing the same 
	 * number of lines as imageListFile, and on each 
	 * line should be one label for that image. Each label
	 * consists of three doubles describing the location of an object in 
	 * the respective image from positiveImageListFilename: [center-x] [center-y]
	 * [object-width]. 
	 * @param negativeImageListFilename Path to a file containing a list of
	 * files, one per line, of images that are known to not contain the object
	 * of interest.
	 * @param posExamplePatchRadius Take positive examples that are slightly
	 * left/right/up/down from the labeled example. This can help get a 
	 * wider variety of positive examples, at the expense of memory and making 
	 * the classifier harder to train. Note that a radius of 1 gives 9x the 
	 * training examples, and a radius of 2 gives 25x. 
	 * @param posExampleScaleRadius Take positive examples that are slightly 
	 * larger or smaller than the labeled object. This size is radius is in
	 * terms of search scale, not pixels. This can help get a 
	 * wider variety of positive examples, at the expense of memory and making 
	 * the classifier harder to train. Note that a radius of 1 gives 3x the 
	 * training examples, and a radius of 2 gives 5x. 
	 * @param numPosPatches Number of positive patches to include in the dataset.
	 * If this is less than 0 or greater than the total number of available
	 * patches, the total number of available patches is used. 
	 * @param numNegPatches Number of negative patches to include in the dataset.
	 * If this is less than 0, then a balanced dataset is created, where there 
	 * are as many negative patches as positive patches. 
	 * @param scale	Tunes the size of the object window, making it narrower or
	 * wider than indicated in the dataset by a factor of [scale]. A scale 
	 * less than 1 will crop the patches smaller around the center of the object,
	 * will greater than 1 will include more of a border around the object. 
	 */
	PatchDataset(cv::Size patchsize,
				 const std::string &positiveImageListFilename = "", 
				 const std::string &positiveImageLabelsFilename = "",
				 const std::string &negativeImageListFilename = "",
				 int posExamplePatchRadius=0,
				 int posExampleScaleRadius=0,
				 int numPosPatches = -1, 
				 int numNegPatches = -1,
				 double scale = 1); 
	
	/**
	 * \brief Destructor.
	 */
	~PatchDataset();
	
	/**
	 * \brief Get all patches in the dataset.
	 * 
	 * @return All patches in the dataset.
	 */
	std::vector<ImagePatch*> getPatches(); 
	
	/**
	 * \brief Get the positive patches in the dataset.
	 * 
	 * @return The positive in the dataset.
	 */
	std::vector<ImagePatch*> getPosPatches(); 
	
	/**
	 * \brief Get the negative patches in the dataset.
	 * 
	 * @return The negative patches in the dataset.
	 */
	std::vector<ImagePatch*> getNegPatches(); 
		
	/**
	 * \brief Get the labels for all patches.
	 * 
	 * @return A vector of labels, of size Nx1, where N is the total number
	 * of patches returned by getPatches(), and the order of labels is respective
	 * to the patches returned by getPatches().
	 */
	void getLabels(cv::Mat &dest); 
	DEPRECATED(CvMat* getLabels()); 
	
	
	/**
	 * \brief Get the size of the patches in the dataset.
	 * 
	 * @return The size of the image patches in the dataset.
	 */
	cv::Size getPatchSize(); 
	
	/**
	 * \brief Save the dataset in a compact, binary format for later use.
	 * 
	 * @param filename Path to the file being saved.
	 */
	void writeToFile(const std::string &filename); 
	
	/**
	 * \brief Load the dataset from a compact, binary format saved earlier.
	 * 
	 * @param filename Path to the file being loaded.
	 *
	 * @return A new dataset object.
	 */
	static PatchDataset* readFromFile(const std::string &filename); 
	
	/**
	 * \brief Load from a file in a verbose, text based format.
	 */
	friend std::istream& operator>> (std::istream& ifs, PatchDataset* dataset); 
	
	
	/**
	 * \brief Write to a file in a verbose, text based format.
	 */
	friend std::ostream& operator<< (std::ostream& ifs, PatchDataset* dataset); 
	
	
	/**
	 * \brief Reconstruct the ImageDataSet object (used by other Machine
	 * Perception Primitives in NMPT) used to create the negative examples in 
	 * this PatchDataset.
	 */
	ImageDataSet* getNegImagesDataset(); 
	
	/**
	 * \brief Reconstruct the ImageDataSet object (used by other Machine
	 * Perception Primitives in NMPT) used to create the positive examples in
	 * this PatchDataset.
	 */
	ImageDataSet* getPosImagesDataset(); 
	
	/**
	 * \brief Query the name of the source file that was used to locate files
	 * containing the negative patches in this dataset. 
	 */
	std::string getNegImagesFileName(); 
	
	
	/**
	 * \brief Query the name of the source file that was used to locate files
	 * containing the positive examples in this dataset. 
	 */
	std::string getPosImagesFileName(); 
	
	
	/**
	 * \brief Query the name of the source file that was used to locate
	 * the positive examples in their source images for this dataset. 
	 */
	std::string getPosLabelsFileName(); 
	
	int getNumUniquePositiveImages(); 
	std::string getUniquePosImageName(int num);
	std::vector<cv::Rect> getObjectLocationsInPosImage(int num);
	
protected:
	
	static int useFast; 
	static int minPerNegIm; 
	
	int numPosImages; 
	int numNegImages; 
	int posImagesHaveLabels; 
	
	ImagePatch* getPatch(IplImage* image, cv::Size patchSize, CvPoint objCenter, cv::Size objSize); 
	ImagePatch* getRandomPatch(IplImage* image, cv::Size patchSize, std::vector<cv::Rect> blackoutList); 
	std::vector<ImagePatch*> getRandomPatches(int numPatches, IplImage* image, cv::Size patchSize, std::vector<cv::Rect> blackoutList);
	
	cv::Size patchSize; 
	
	CvRNG rng; 
	
	std::vector<ImagePatch*> patches; 
	std::vector<ImagePatch*> posPatches; 
	std::vector<ImagePatch*> negPatches; 
	
	std::vector<int> labels; 
	std::string posImagesFileList; 
	std::string posImagesLabelsList; 
	std::string negImagesFileList; 
	
	ImageDataSet* posImageDataset; 
	ImageDataSet* negImageDataset; 
	
	void addToStream(std::ostream& out) ; 
	void addToStreamBinary(std::ostream& out) ; 
	void readFromStream(std::istream& in) ; 
	void readFromStreamBinary(std::istream& in) ; 	
	
	int loadEvaluator();
	
	PatchList* list; 
	DetectionEvaluator* evaluator; 
	
};

std::ostream& operator<< (std::ostream& ofs, PatchDataset* a);
std::istream& operator>> (std::istream& ifs, PatchDataset* a);

#endif
