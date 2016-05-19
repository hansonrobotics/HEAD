/*
 *  HaarFeature2.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/22/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

/*
 *  HaarFeature.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef HAARFEATURE2_H
#define HAARFEATURE2_H

#include <opencv2/core/core.hpp>
#include "Feature2.h"
#include "BoxFeature2.h"

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A linear feature that uses Haar Wavelets,
 * which are a certain kind of box filter.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class HaarFeature2 : public BoxFeature2 {
public: 
	
	/**
	 * \brief Constructor. Unlike BoxFeature, HaarFeature doesn't need to know
	 * the number of boxes in the constructor, because it is specified in the
	 * parameters.
	 *
	 * @param expectedPatchSize Specifies the size of the filter, which specifies
	 * the possible ranges for the box positions.
	 */
	HaarFeature2(cv::Size expectedPatchSize); 	
	
	/**
	 * \brief Default Constructor. 
	 */
	HaarFeature2(); 	
	
	/**
	 * \brief Copy Constructor. 
	 */
	HaarFeature2(const HaarFeature2 &copy); 	
	
	/**
	 * \brief Assignment operator
	 **/
	HaarFeature2 & operator=(const HaarFeature2 &rhs); 
	
	/**
	 * \brief Destructor. 
	 */
	virtual ~HaarFeature2(); 
	
	/**
	 * \brief Set the parameters that describe the Haar Wavelet used for 
	 * filtering. See detailed description of the parameters below.
	 *
	 * A haar wavelet is localized to a sub rectangle of the total filter
	 * area. There are 6 supported kinds of Haar wavelets: left-right, up-down, 
	 * center-surround, left-center-right, diagonal, up-center-down
	 *
	 * The HaarFeature parameter vector has 10 elements, which are: sub-region
	 * width, sub-region height, subregion x,y position of the center,
	 * kind (numbered 1-6 respective of the above), sign (describes which boxes
	 * are added and which are subtracted), horizontal symmetry, vertical 
	 * symmetry, mean subtraction, and brightness normalization.
	 *
	 * The center of an even-sized wavelet is to the right/bottom of the center. The
	 * center of an odd-sized wavelet is on the center.
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
	 *
	 * Parameter value ranges: 
	 * \li 1) width: [1:patchWidth] -- [1:2]=2, (2:3]=3, (3:4]=4, ... 
	 * \li 2) height: [1:patchHeight] -- [1:2]=2, (2:3]=3, (3:4]=4, ... 
	 * \li 3) center-x: [0:(patchWidth-1)] -- [0:1]=0,  (1:2]=1, ..., (patchWith-2:patchWidth-1]=patchWidth-2
	 * \li 4) center-y: [0:(patchWidth-1)] -- [0:1]=0,  (1:2]=1, ..., (patchWith-2:patchWidth-1]=patchWidth-2
	 * \li 5) wavelet: [0:6] -- [0:1]=1, (1:2]=2, ...
	 * \li 6) sign: [0:2] -- [0:1]=-1, (1:2]=1
	 * \li 7) hSym: [0:3] -- [0:1]=-1, (1:2]=0, (2:3]=1
	 * \li 8) vSym: [0:3] -- [0:1]=-1, (1:2]=0, (2:3]=1
	 * \li 9) meanSub: [0:2] -- [0:1]=0, (1:2]=1
	 * \li 10) normBrigthness: [0:2] -- [0:1]=0, (1:2]=1
	 */
	virtual void setFeatureParameters(const cv::Mat &paramVec);
	
private:
	virtual void init(cv::Size expectedPatchSize = cv::Size(0,0), int nBoxes=2); 
	int fullWidth; 
	int fullHeight; 
	int fullX; 
	int fullY; 
	int waveletType; 
	int waveletSign; 
};


#endif
