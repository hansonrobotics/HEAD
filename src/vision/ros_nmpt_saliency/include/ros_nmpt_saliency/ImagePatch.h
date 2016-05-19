/*
 *  ImagePatch.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/11/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef IMAGEPATCH_H
#define IMAGEPATCH_H

#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>

#include "DebugGlobals.h"

/**
 * \brief Type used for storing integral data. Depending on your architecture
 * it may be more efficient for this to be unsigned long, or unsigned long long. 
 * The max size of image patches that can be represented is approximately 8MP
 * if unsigned long is used, and much larger if unsigned long long is used.
 */
typedef int integral_type; 

class FeatureRegressor; 
class BoxFeature; 

/*
class ImagePatchROI {
public:
	int x, y, width, height; 
	ImagePatchROI(); 
	ImagePatchROI(int X, int Y, int Width, int Height); 
};
*/


/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A data structure that represents  
 * an image in an efficient memory layout, and manipulates that image in ways
 * useful for efficient feature processing.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class ImagePatch {
public: 
	
	/**
	 * \brief Constructor.
	 * 
	 * Creates an empty image patch, which can be set later using setImage.
	 */
	ImagePatch(); 
	
	/**
	 * \brief Destructor.
	 */
	~ImagePatch(); 
	
	/**
	 * \brief Copy Constructor
	 **/
	ImagePatch(const ImagePatch &copy); 
	
	/**
	 * \brief Assignment operator
	 **/
	ImagePatch & operator=(const ImagePatch &rhs); 
	
	/**
	 * \brief Sets the ImagePatch to contain this image's data.
	 * 
	 * Copies the whole image, so any cropping, converting, and resizing must
	 * be done prior to calling setImage. The image must be of type
	 * IPL_DEPTH_8U (8-bit, unsigned integer), with a single channel.
	 *
	 * @param image The image we want to represent.
	 * @param setData Copy the image data. This takes some memory, and is often
	 * not needed if we are only using integral based features. However, it is
	 * necessary for visualization of ImagePatch data, and may be useful for 
	 * other feature types.
	 * @param setIntegral Compute the integral of the ImagePatch. This is 
	 * required to apply BoxFeature to the ImagePatch. 
	 */
	void setImage(const cv::Mat &image, int setData=1, int setIntegral=1);
	DEPRECATED(void setImage(const IplImage* image, int setData=1, int setIntegral=1)); 
	
	
	/**
	 * \brief Sets the ImagePatch to contain this image's data contained in ROI.
	 * 
	 * Any converting, and resizing must
	 * be done prior to calling setImage. The image must be of type
	 * IPL_DEPTH_8U (8-bit, unsigned integer), with a single channel.
	 *
	 * @param image The image we want to represent.
	 * @param ROI The region of the image we want to represent.
	 * @param setData Copy the image data. This takes some memory, and is often
	 * not needed if we are only using integral based features. However, it is
	 * necessary for visualization of ImagePatch data, and may be useful for 
	 * other feature types.
	 * @param setIntegral Compute the integral of the ImagePatch. This is 
	 * required to apply BoxFeature to the ImagePatch. 
	 */
	void setImage(const cv::Mat &image, cv::Rect ROI, int setData=1, int setIntegral=1);
	DEPRECATED(void setImage(const IplImage* image, CvRect ROI, int setData=1, int setIntegral=1));  
	
	/**
	 * \brief Get the size of the ImagePatch. 
	 */
	cv::Size getImageSize() const; 
	
	
	
	/**
	 * \brief Get an OpenCV image of this image patch's actual data, without
	 * copying. You are responsible for releasing this Image Header. Don't 
	 * release the image!
	 */
	void getImageHeader(cv::Mat &header) const; 
	DEPRECATED(IplImage* getImageHeader() const); 
	
	/**
	 * \brief Get a pointer to the integral image data. This data is laid
	 * out row-wise. Each row has length integralDataRowWidth(). Elements 
	 * after the getImageSize().width+1st have unspecified value. 
	 */
	const integral_type* startOfIntegralData() const; 
	
	/**
	 * \brief Get a pointer to the raw image data. This data is laid
	 * out row-wise. Each row has length integralDataRowWidth(). Elements 
	 * after the getImageSize().width th have unspecified value. 
	 */
	const unsigned char* startOfImageData() const; 
	
	
	/**
	 * \brief Number of unsigned chars from the start of one row of the raw image
	 * data to the start of the next. At least getImageSize().width.
	 */
	int imageDataRowWidth() const;
	
	/**
	 * \brief Number of bytes from the start of one row of the raw image
	 * data to the start of the next.
	 */
	int imageDataRowBytes() const; 
	
	
	/**
	 * \brief Number of integral_type values from the start of one row of raw image
	 * data to the start of the next. At least getImageSize().width+1.
	 */
	int integralDataRowWidth() const; 
	
	
	/**
	 * \brief Number of bytes from the start of one row of the integral image
	 * data to the start of the next. 
	 */
	int integralDataRowBytes() const; 
	
	
	/**
	 * \brief Load a single image patch from a file in a verbose text-based format.  
	 */
	friend std::istream& operator>> (std::istream& ifs, ImagePatch* model); 
	
	/**
	 * \brief Load a collection of image patches from a file in a verbose text-based format. 
	 */	
	friend std::istream& operator>> (std::istream& ifs, std::vector<ImagePatch*>& model); 
	
	/**
	 * \brief Save a single image patch to a file in a verbose text-based format. 
	 */
	friend std::ostream& operator<< (std::ostream& ofs, ImagePatch* model); 
	
	/**
	 * \brief Save a collection of image patches to a file in a verbose text-based format. 
	 */
	friend std::ostream& operator<< (std::ostream& ofs, std::vector<ImagePatch*>& model); 
	
	
	/**
	 * \brief Save a collection of image patches to a file in a compact binary format. 
	 */
	static void writePatchesToFile(const char* filename, const std::vector<ImagePatch*> &patches); 
	
	/**
	 * \brief Load a collection of image patches to a file in a compact binary format. 
	 */
	static std::vector<ImagePatch*> readPatchesFromFile(const char* filename);
	
	
	
	/**
	 * \brief Save a collection of image patches to a file in a verbose text-based format. 
	 */
	void addToStreamBinary(std::ostream& out) ;
	
	/**
	 * \brief Load a collection of image patches to a file in a compact binary format. 
	 */
	void readFromStreamBinary(std::istream& in) ; 
	
	/**
	 * \brief Get size of integral image
	 **/
	cv::Size getIntegralSize() const; 
	
	void testImagePatch() ; 
private:
	
	//unsigned char* data; 
	//integral_type* integralData; 
	int width; 
	int height;
	//int dataRowWidth; 
	//int integralRowWidth;
	/*
	size_t dataRowBytes; 
	size_t integralRowBytes; 
	
	
	
	size_t dataBufferSize; 
	size_t integralBufferSize; 
	*/
	void addToStream(std::ostream& out) ; 
	void readFromStream(std::istream& in) ; 
	
	cv::Mat imgData; 
	cv::Mat intData; 
	
	//void setImage(IplImage* image, CvRect ROI, int setData=1, int setIntegral=1); 
	
	
	/*
	void filterPatchWithBox(BoxFeature* feat, float* dest, int dstWidthStep); 
	double filterPatchWithBox(BoxFeature* feat); 
	
	
	//double filterPatchWithBoxAtROI(BoxFeature* feat, ImagePatchROI roi); 
	void filterPatchWithBoxROIMethod1(BoxFeature* feat, float* dest, int dstWidthStep);
	void filterPatchWithBox(BoxFeature* feat, int atWidth, int atHeight, vector<integral_type*>& srcInds, vector<float*>& destInds); 
	void filterPatchWithBox(FeatureRegressor* reg, int atWidth, int atHeight, vector<integral_type*>& srcInds, vector<float*>& destInds); 
	void filterPatchWithBox(BoxFeature* feat, int atWidth, int atHeight, float* dest, vector<int>& srcInds, vector<int>& destInds); 
	 
	*/
	
	
};

std::ostream& operator<< (std::ostream& ofs, ImagePatch* a);
std::istream& operator>> (std::istream& ifs, ImagePatch* a);
std::ostream& operator<< (std::ostream& ofs, std::vector<ImagePatch*>& a);
std::istream& operator>> (std::istream& ifs, std::vector<ImagePatch*>& a);

#endif
