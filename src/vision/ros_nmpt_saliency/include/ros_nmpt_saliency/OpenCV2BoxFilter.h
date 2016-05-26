/*
 *  OpenCV2BoxFilter.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/22/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __OPENCV2_BOX_FILTER
#define __OPENCV2_BOX_FILTER

#include <opencv2/core/core.hpp>

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A class for efficiently applying multiple box convolutions to the same image.
 *
 * By computing the integral image once, an image can be filtered multiple times with boxes of different sizes. 
 * Requires OpenCV libraries. 
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */
class OpenCV2BoxFilter {
public: 
	/**
	 * \brief Constructor. 
	 *
	 * NOTE: The source image is assumed to be single channel image, of a type supported by
	 * cv::Integral. The destination image will be a 64-bit floating point image.
	 *
	 * @param maxPaddingRequired Extra memory required for performing convolutions. In practice, this must
	 *                           be the distance from the central pixel of the most extreme point of any 
	 *                           box we expect to use, i.e. at least the maximum intended value of: (1) 
	 *                           |boxPosition.x|, (2) |boxPosition.x+boxPosition.width-1|, 
	 *                           (3) |boxPosition.y|, (4) |boxPosition.y+boxPosition.height-1|. Erring on the
	 *                           side of too much padding has no adverse consequences other than increased
	 *                           memory usage.
	 * 
	 * @param filterType OpenCV data type for output of filtering. Can be CV_32S, CV_32F, CV_64F.
	 */
	OpenCV2BoxFilter(int maxPaddingRequired, int filteType = CV_32F);
	
	
	/**
	 * \brief Constructor. 
	 *
	 * NOTE: The source image is assumed to be single channel image, of a type supported by
	 * cv::Integral. The destination image will be a 64-bit floating point image.
	 *
	 * NOTE: call setMaxPaddingRequired later.
	 */
	OpenCV2BoxFilter();
	
	/**
	 * \brief Copy Constructor. 
	 *
	 */
	OpenCV2BoxFilter(const OpenCV2BoxFilter &rhs);
	
	/**
	 * \brief Assignment Operator. 
	 *
	 */
	OpenCV2BoxFilter & operator=(const OpenCV2BoxFilter &rhs) ; 
	
	
	/**
	 * \brief Default Destructor. 
	 * 
	 * Deallocates all memory associated with the box filter. 
	 */
	~OpenCV2BoxFilter(); 
	
	/**
	 * \brief Change the image that will be filtered. The source image is assumed to be single 
	 * channel image, of a type supported by cv::integral.
	 *
	 * 
	 * This method computes a new integral image that will be used repeatedly for future filtering, until
	 * this method is invoked again with a new image. 
	 * @param imageToFilter A new OpenCV image to filter repeatedly. 
	 */
	void setNewImage(const cv::Mat &imageToFilter);
	
	/**
	 * \brief Compute a box filter and over-write the result to an image. The result of filtering is the sum
	 * of the pixel values in the specified rectangular region.  It will be a 64-bit single channel floating point
	 * image.
	 *
	 * @param destinationImage Image target for the results of the filtering. After invoking this method,
	 * the contents of this variable will be overwritten with the results of the operation. 
	 * @param boxPosition Location of the box (relative to the central pixel) to average pixel values over. For example,
	 * cvRect(0,0,1,1) has zero-offset from the current pixel and a width/height of one pixel. Filtering the image
	 * in this way will return the original image.  To filter with a width-5 averaging filter with no offset, use
	 * cvRect(-2,-2,5,5). 
	 * @param scaleResultFactor Scale the result by some factor. For example, if the scale is 
	 * 1/boxPosition.width*boxPosition.height, the average of the pixels in the box is returned instead of the sum.
	 */ 
	void setBoxFilter(cv::Mat &destinationImage, cv::Rect boxPosition=cv::Rect(0,0,1,1), double scaleResultFactor=1.0); 
	
	/**
	 * \brief Compute a box filter and add the result to an image. The result of filtering is the average
	 * of the pixel values in the specified rectangular region. This is useful for designing filters composed of 
	 * multiple boxes. 
	 *
	 * @param destinationImage Image target for the results of the filtering. After invoking this method,
	 * the contents of this variable will be overwritten with the results of the operation. 
	 * @param boxPosition Location of the box (relative to the central pixel) to average pixel values over. For example,
	 * cvRect(0,0,1,1) has zero-offset from the current pixel and a width/height of one pixel. Filtering the image
	 * in this way will return the original image.  To filter with a width-5 averaging filter with no offset, use
	 * cvRect(-2,-2,5,5). 
	 * @param scaleResultFactor Scale the result by some factor. For example, if the scale is 
	 * boxPosition.width*boxPosition.height, the sum of the pixels in the box is returned instead of the average.
	 */  
	void accumulateBoxFilter(cv::Mat &destinationImage, cv::Rect boxPosition=cv::Rect(0,0,1,1), double scaleResultFactor=1.0);
private:
	int maxPaddingRequired; 
	int filterType; 
	
	cv::Mat integralImage; 
	cv::Mat zeroPaddedScratchImage; 
}; 

#endif
