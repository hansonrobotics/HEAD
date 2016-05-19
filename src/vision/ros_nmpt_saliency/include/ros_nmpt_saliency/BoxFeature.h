/*
 *  BoxFeature.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef BOXFEATURE_H
#define BOXFEATURE_H


#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include "Feature.h"
#include "PatchList.h"

class ImagePatch; 

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A linear feature that uses the "Integral
 * Image" trick to efficiently evaluate convolution kernels on an image, where
 * the kernel can be expressed as a sum of boxes. 
 *
 * The derived HaarFeature class is a subset of these box features that are
 * like wavelets composed of blocks.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class BoxFeature : public Feature {
public: 
	
	/**
	 * \brief Constructor. 
	 *
	 * @param expectedPatchSize Specifies the size of the filter, which specifies
	 * the possible ranges for the box positions.
	 * @param nBoxes  The number of boxes.
	 * The filtering time decreases linearly with the number of boxes.
	 */
	BoxFeature(cv::Size expectedPatchSize=cv::Size(0,0), int nBoxes =2); 
	
	/**
	 * \brief Destructor. 
	 */
	virtual ~BoxFeature(); 
	
	
	/**
	 * \brief Set the parameters that describe the position of the boxes. See
	 * detailed description of the parameters below.
	 * 
	 * The BoxFeature parameter vector has 5N+4 elements, where N is the
	 * number of boxes. The first 5N elements describe the sign of the box
	 * (addition or subtraction), x,y position of
	 * the box's top-left corner, and its width and height respectively. The
	 * parameters for each box are together, so the first 5 elements describe 
	 * the first box, etc. The sign parameter can be -1 or +1, and the position 
	 * parameters are bounded below by 0 and above
	 * by patchSize.[width or height]-1, as appropriate.
	 *
	 * The last 4 parameters describe global behavior of the filtering process.
	 * They are: horizontal symmetry, vertical symmetry, mean subtraction,
	 * brightness normalization respectively. 
	 * 
	 * The first two each effectively
	 * double the number of boxes, by reflecting them around the horizontal and 
	 * vertical axes respectively. Values for these can be -1,0,+1. 0 means 
	 * no symmetry, -1 means the symmetric box has the opposite sign, and +1
	 * means the symmetric box has the same sign. 
	 * 
	 * Mean Subtraction means the mean pixel value of the image patch is 
	 * subtracted before filtering. This normalizes for contrast but not 
	 * brightness. Brightness Normalization divides the image patch by its L1 norm
	 * before filtering. If both options are set, brightness is normalized 
	 * and then the mean is subtracted. Both of these operations incur a time
	 * penalty, which is relatively small.
	 */
	virtual void setFeatureParameters(const cv::Mat &paramVec);
	
	/**
	 *\brief Turn an ImagePatch into a scalar value by applying the box filter.
	 * 
	 * @param patch The image patch to evaluate.
	 * @return The value of the Image Patch according to the feature.
	 **/
	virtual double evaluateImagePatch(const ImagePatch* patch); 
	
	/**
	 * \brief Apply the feature to a PatchList data structure. 
	 * 
	 * The PatchList is constructed in such a way that this filtering can be 
	 * done as efficiently as possible.
	 * 
	 * @param patches Remaining patches in a large image to be filtered.
	 **/
	virtual void filterPatchList( PatchList* patches);
	
	
protected:
	void expandSymmetries(); 
	void setKernelVisualization() ; 
	void keepBoxesInPatch(); 
	void createWithNBoxes(int n); 
	
	double filterEnergy; 
	
	int numBoxes; 
	CvMat* weights; 
	CvMat* boxX; 
	CvMat* boxY; 
	CvMat* width; 
	CvMat* height; 
	
	CvMat* boxX1; 
	CvMat* boxX2; 
	CvMat* boxY1; 
	CvMat* boxY2; 
	
	int hasVSym; 
	int hasHSym; 
	int meanSub; 
	int normBrightness; 
	
	const static int numFeaturesPerBox = 5; 
	const static int numExtraFeatures = 4; 
};

#endif
