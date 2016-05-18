/*
 *  PatchDataset2.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/25/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#ifndef PATCHDATASET2_H
#define PATCHDATASET2_H


#include <opencv2/core/core.hpp>

#include <iostream>
#include <vector>
#include <string>

#include "BlockTimer.h"
#include "ImageDataSet2.h"
#include "ImagePatch2.h"
#include "PatchList2.h" 
#include "FastPatchList2.h" 
#include "DetectionEvaluator2.h"
#include "DebugGlobals.h"


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
class PatchDataset2 {
public:
	
	/**
	 * \brief Default Constructor.
	 **/
	PatchDataset2();
	
	/**
	 * \brief Copy Constructor.
	 **/
	PatchDataset2(const PatchDataset2 &copy) ; 
	
	/**
	 * \brief Assignment operator
	 **/
	PatchDataset2 & operator=(const PatchDataset2 &rhs); 	
	
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
	 * @param useFastPatchList Sets the patch extraction method. 
	 * @param scale Tunes the size of the object window, making it narrower or
	 * wider than indicated in the dataset by a factor of [scale]. A scale 
	 * less than 1 will crop the patches smaller around the center of the object,
	 * will greater than 1 will include more of a border around the object. 
	 */
	PatchDataset2(cv::Size patchsize,
				 const std::string &positiveImageListFilename = "", 
				 const std::string &positiveImageLabelsFilename = "",
				 const std::string &negativeImageListFilename = "",
				 int posExamplePatchRadius=0,
				 int posExampleScaleRadius=0,
				 int numPosPatches = -1, 
				 int numNegPatches = -1,
				 int useFastPatchList = 1, 
				  double scale = 1. ); 
	
	/**
	 * \brief Destructor.
	 */
	~PatchDataset2();
	
	/**
	 * \brief Get all patches in the dataset.
	 * 
	 * @return All patches in the dataset.
	 */
	std::vector<ImagePatch2> getPatches() const; 
	
	/**
	 * \brief Get the positive patches in the dataset.
	 * 
	 * @return The positive in the dataset.
	 */
	std::vector<ImagePatch2> getPosPatches() const; 
	
	/**
	 * \brief Get the negative patches in the dataset.
	 * 
	 * @return The negative patches in the dataset.
	 */
	std::vector<ImagePatch2> getNegPatches() const; 
	
	/**
	 * \brief Get the labels for all patches.
	 * 
	 * @return A vector of labels, of size Nx1, where N is the total number
	 * of patches returned by getPatches(), and the order of labels is respective
	 * to the patches returned by getPatches().
	 */
	void getLabels(cv::Mat &dest) const; 
		
	/**
	 * \brief Get the size of the patches in the dataset.
	 * 
	 * @return The size of the image patches in the dataset.
	 */
	cv::Size getPatchSize() const; 
	
	/**
	 * \brief Reconstruct the ImageDataSet object (used by other Machine
	 * Perception Primitives in NMPT) used to create the negative examples in 
	 * this PatchDataset.
	 */
	ImageDataSet2 getNegImagesDataset() const; 
	
	/**
	 * \brief Reconstruct the ImageDataSet object (used by other Machine
	 * Perception Primitives in NMPT) used to create the positive examples in
	 * this PatchDataset.
	 */
	ImageDataSet2 getPosImagesDataset() const; 
	
	/**
	 * \brief Query the name of the source file that was used to locate files
	 * containing the negative patches in this dataset. 
	 */
	std::string getNegImagesFileName() const; 
	
	/**
	 * \brief Query the name of the source file that was used to locate files
	 * containing the positive examples in this dataset. 
	 */
	std::string getPosImagesFileName() const; 
	
	/**
	 * \brief Query the name of the source file that was used to locate
	 * the positive examples in their source images for this dataset. 
	 */
	std::string getPosLabelsFileName() const; 
	
	/**
	 * \brief Get the number of unique positive images (unique by name) in 
	 * the dataset.
	 **/
	int getNumUniquePositiveImages() const; 
	
	/**
	 * \brief Get the file name of an image in the dataset. 
	 **/
	std::string getUniquePosImageName(int num) const;
	
	/** 
	 * \brief Get the location of all marked objects in the image (if any).
	 **/
	std::vector<cv::Rect> getObjectLocationsInPosImage(int num) const;
	
	/**
	 * \brief Sets the method of patch extraction for creating the dataset. 
	 **/
	void setUseFastPatchList(int yesorno); 
	
	/**
	 * \brief Get the method of patch extraction used for creating the dataset. 
	 **/ 
	int getUseFastPatchList() const; 
	
	/**
	 * \brief Test whether this dataset contains any information
	 **/
	int empty() const; 
	
	
	friend cv::FileStorage& operator << (cv::FileStorage &fs, const PatchDataset2 &data); 
	friend void operator >> ( const cv::FileNode &fs, PatchDataset2 &data) ; 
protected:
	void init(cv::Size patchsize=cv::Size(30,30), 
			  const std::string &positiveImageListFilename="", 
			  const std::string &positiveImageLabelsFilename="",
			  const std::string &negativeImageListFilename="" ,
			  int posExamplePatchRadius=0, int posExampleScaleRadius=0,
			  int numPosPatches=-1, int numNegPatches=-1, int useFastPatchList=1,
			  double scale = 1.) ; 
	void copy(const PatchDataset2 &rhs); 
	
	ImagePatch2 getPatch(const cv::Mat &image, cv::Size patchSize, cv::Point objCenter, cv::Size objSize) const; 
	ImagePatch2 getRandomPatch(const cv::Mat &image, cv::Size patchSize, const std::vector<cv::Rect> &blackoutList) const ; 
	std::vector<ImagePatch2> getRandomPatches(int numPatches, const cv::Mat &image, cv::Size patchSize, const std::vector<cv::Rect> &blackoutList) const;
	void recreatePosNegPatches() ; 
	
	static int minPerNegIm; 
	
	int useFast; 
	double scale; 
	
	std::string posImagesFileList; 
	std::string posImagesLabelsList; 
	std::string negImagesFileList; 
	
	cv::Size patchSize; 
	
	std::vector<ImagePatch2> patches; 
	std::vector<ImagePatch2> posPatches; 
	std::vector<ImagePatch2> negPatches; 
	
	std::vector<int> labels; 
	
	ImageDataSet2 posImageDataset; 
	ImageDataSet2 negImageDataset; 
	
	DetectionEvaluator2 evaluator; 
	PatchList2  pl; 
	FastPatchList2 fpl; 
	
	PatchList2* list; 
	
};

cv::FileStorage& operator << (cv::FileStorage &fs, const PatchDataset2 &data); 
void operator >> ( const cv::FileNode &fs, PatchDataset2 &data) ; 


#endif
