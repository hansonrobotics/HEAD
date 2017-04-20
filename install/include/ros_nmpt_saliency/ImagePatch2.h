/*
 *  ImagePatch2.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/24/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#ifndef IMAGEPATCH2_H
#define IMAGEPATCH2_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>


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
class ImagePatch2 {
public: 
	
	/**
	 * \brief Constructor.
	 * 
	 * Creates an empty image patch, which can be set later using setImage.
	 */
	ImagePatch2(); 
	
	/**
	 * \brief Destructor.
	 */
	~ImagePatch2(); 
	
	/**
	 * \brief Copy Constructor
	 **/
	ImagePatch2(const ImagePatch2 &copy); 
	
	/**
	 * \brief Assignment operator
	 **/
	ImagePatch2 & operator=(const ImagePatch2 &rhs); 
	
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
	 * @param setSqInt Compute the square integral of the ImagePatch. This 
	 * is required to apply features that rely on variance.
	 * @param setTInt Compute the tilted integral of the image patch.
	 */
	void setImage(const cv::Mat &image, int setData=1, int setIntegral=1, int setSqInt=0, int setTInt=0);
	
	
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
	void setImage(const cv::Mat &image, cv::Rect ROI, int setData=1, int setIntegral=1, int setSqInt=0, int setTInt=0);
	
	/**
	 * \brief Get the size of the ImagePatch. 
	 */
	cv::Size getImageSize() const; 
	
	/**
	 * \brief Get this image patch's data.
	 */
	const cv::Mat getImageHeader() const; 

	/**
	 * \brief Get this image patch's integral data.
	 */
	const cv::Mat getIntegralHeader() const; 
	
	/**
	 * \brief Get this image patch's sqared integral data.
	 */
	const cv::Mat getSqIntegralHeader() const; 
	
	/**
	 * \brief Get this image patch's tilted integral data.
	 */
	const cv::Mat getTIntegralHeader() const; 
	
	/**
	 * \brief Test whether this image patch has image data.
	 */
	int hasImageRep() const;
	
	/**
	 * \brief Test whether this image patch has integral data.
	 */
	int hasIntegralRep() const ;	
	
	/**
	 * \brief Test whether this image patch has sqare integral data.
	 */
	int hasSqIntegralRep() const ;	
	
	/**
	 * \brief Test whether this image patch has tilted integral data.
	 */
	int hasTIntegralRep() const;	
	
	/**
	 * \brief If a representation has not previously been set, set it now.
	 **/ 
	void createRepIfNeeded(int setData=1, int setIntegral=1, int setSqInt=0, int setTInt=0); 
	
	
	friend cv::FileStorage& operator << (cv::FileStorage &fs, const ImagePatch2 &patch); 
	friend void operator >> ( const cv::FileNode &fs, ImagePatch2 &patch) ; 
	
private:
	
	void copy(const ImagePatch2 &rhs, int clone=0); 
	void getImageRep(cv::Mat &dest) const; 
	
	cv::Mat imgData; 
	cv::Mat intData; 
	cv::Mat sqIntData;
	cv::Mat tIntData; 
	
};

cv::FileStorage& operator << (cv::FileStorage &fs, const ImagePatch2 &patch); 
void operator >> ( const cv::FileNode &fs, ImagePatch2 &patch) ; 
/*
std::ostream& operator<< (std::ostream& ofs, ImagePatch2* a);
std::istream& operator>> (std::istream& ifs, ImagePatch2* a);
std::ostream& operator<< (std::ostream& ofs, std::vector<ImagePatch2*>& a);
std::istream& operator>> (std::istream& ifs, std::vector<ImagePatch2*>& a);
 */

#endif
