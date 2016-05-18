/*
 *  NMPTUtils.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/16/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _NMPT_UTILS_H
#define _NMPT_UTILS_H


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <stdio.h>

/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A set of functions that are commonly 
 * used by many NMPT classes and executables. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
namespace NMPTUtils {
	/**
	 * \brief Display a matrix to stdout in a multi line format easy to read. 
	 */ 
	void printMat(const cv::Mat &mat);  
	/**
	 * \brief Display a matrix to stdout in a multi line format easy to read. 
	 */ 
	void printMat(const CvMat &mat);  
	
	/**
	 * \brief Get a single normally distributed random number.  
	 */ 
	double randomNormal() ; 
	
	/**
	 * \brief Get a single random number distributed uniformly from 0-1.  
	 */ 
	double randomFloat(); 
	
	/**
	 * \brief Format a matrix in a single line, comma separated format suitable for output to a CSV file. 
	 */ 
	std::string commaSeparatedFlattenedMat(const cv::Mat &mat); 
	
	
	/**
	 * \brief Allows the source for an NMPT program to be specified from the command line. 
	 * 
	 * \li If the program is invoked with no arguments, the default camera is used.
	 * \li If the program is invoked with any options (e.g. --help), a usage message is printed.
	 * \li If the program is invoked with an integer argument between 0 and 9, the camera 
	 * with the corresponding number is used. 
	 * \li If the program is invoked with a single string argument, it is interpererted as
	 * a path to a video file, which will be used for the source. 
	 *
	 * Returns 0 if there was an error, 1 if a video file was the source, or 2 if a camera
	 * was the source, 	 
	 */ 
	int getVideoCaptureFromCommandLineArgs(cv::VideoCapture& capture, const int argc, const char** argv); 
	
	
	/**
	 * \brief Computes the ratio of the area of the intersection of two rectangles to the area 
	 * of the union.
	 */
	double rectAreaOverlapRatio(cv::Rect r1, cv::Rect r2);
	
	
	/**
	 * \brief Computes the area of the intersection of two rectangles.
	 */
	double rectAreaIntersect(cv::Rect r1, cv::Rect r2); 
	
	/**
	 * \brief Returns true if a is NaN or infinty
	 */
	double notfinite(double a) ; 
	
	
	/**
	 * \brief Divides unnormDist by its L1-norm, effectively turning it into a probability 
	 * distribution's histogram. The values of unnormDist should be all positive. If any 
	 * values are NaN or inf, the probability mass is split uniformly between those values
	 * and all other values become 0. 
	 */
	void distNorm(cv::Mat &normDist, const cv::Mat &unnormDist); 
	
	/**
	 * \brief Generate a matrix of samples from a probability histogram. 
	 */
	void sample(cv::Mat &samples, const cv::Mat& histogram, cv::Size sampleSize=cv::Size(1,1)); 
	
	
	/**
	 * \brief Generate a single sample from a probability histogram.
	 */
	int sample(const cv::Mat& histogram); 
	
	/**
	 * \brief Apply a function (in place) to every element of a matrix. The function must take
	 * an argument of the matrix's data type, and return a value of the matrix's 
	 * data type.
	 */
	template<class mattype>	void map( cv::Mat &mat, mattype (*function)(mattype)); 
	
	/**
	 * \brief Check to see if a matrix contains a certain value.
	 **/
	template<class mattype> int contains(const cv::Mat &mat, mattype val); 
	
	/**
	 * \brief Quickly computer the n choose k function. The result is accurate through at least
	 * the range of 32-bit integers. 
	 **/
	double nchoosek(int n, int k); 
	
	/**
	 * \brief Checks to see if any member of mat is non-zero
	 */
	int any(CvMat* mat) ; 
	
	/**
	 * \brief Checks to see if any member of mat is non-zero
	 */
	template<class mattype> int any(const cv::Mat &mat) ; 
	
	
	/**
	 * \brief Round the elements of mat towards zero
	 **/
	void fix(CvMat* mat) ; 
	
	/**
	 * \brief Write the value of a matrix to a FileStorage in a Base64 format that is much
	 * more compact than OpenCV's default format. The data is keyed by the supplied
	 * name. 
	 **/
	void writeMatBinary(cv::FileStorage &storage, const std::string &name, const cv::Mat &mat);
	
	/**
	 * \brief Read the value of a matrix to a FileStorage in a Base64 format that is much
	 * more compact than OpenCV's default format. The data is keyed by the supplied
	 * name. If you attempt to read a matrix that was not saved with "writeMatBinary,"
	 * this will probably crash your program.
	 **/
	void readMatBinary(const cv::FileNode &storage, cv::Mat &mat);
	
	
	 //
	 // \brief Write the value of a matrix to a FileStorage in a Base64 format that is much
	 //* more compact than OpenCV's default format. The data is keyed by the supplied
	 //* name.  The data is compressed with zlib before writing. If you attempt to read 
	 //* a matrix that was not saved with "writeMatBinaryCompressed," this will probably 
	 //* crash your program.
	 //**/
	//void writeMatBinaryCompressed(cv::FileStorage &storage, const std::string &name, const cv::Mat &mat);
	
	
	///**
	 //* \brief Read the value of a matrix to a FileStorage in a Base64 format that is much
	 //* more compact than OpenCV's default format. The data is keyed by the supplied
	 //* name.  The data is uncompressed with zlib after writing. If you attempt to read 
	 //* a matrix that was not saved with "writeMatBinaryCompressed," this will probably 
	 //* crash your program.
	 //**/
	//void readMatBinaryCompressed(cv::FileStorage &storage, const std::string &name,  cv::Mat &mat);
	
	/**
	 * Converts binary data to a Base64 string that can be written to a text
	 * file, e.g. the YAML format written by OpenCV.
	 **/
	void binaryToAscii(std::string &dest, const uchar *data, size_t bytes) ;
	
	/**
	 * \brief Converts a Base64 string to binary data.
	 **/
	void asciiToBinary(const std::string &data, uchar *dest, size_t bytes) ;
	
	/**
	 * \brief Converts binary data to a non-standard hex format that can be written
	 * to a text file, e.g. the YAML format written by OpenCV. This format
	 * requires 1.5 the storage space of Base64.
	 **/
	void binaryToHex(std::string &dest, const uchar *data, size_t bytes); 
	
	/**
	 * \brief Converts a non-standard hex format to binary data. This format requires
	 * 1.5 the storage space of Base64.
	 **/
	void hexToBinary(const std::string &data, uchar *dest, size_t bytes); 
	
	/**
	 * \brief Splits a string into a collection of string vectors, each with maximum 
	 * length maxlen. This is useful because OpenCV imposes a maximum string 
	 * length when writing to a file, somewhere between 2048 and 4096.
	 **/
	void splitString(std::vector<std::string> &dest, const std::string &data, size_t maxlen=512); 
	
	/**
	 * \brief Rejoins a vector of strings into a single string.
	 **/
	void joinString(const std::vector<std::string> &data, std::string &dest); 
	
	///**
	 //* \brief Uses zlib to compress data. The output dest is allocated in this function
	 //* and dest_size is set to the number of bytes of the compressed data. Returns a
	 //* zlib error code if compression fails.
	 //**/
	//int compress(uchar* &dest, size_t &dest_size, const uchar* src, size_t src_size) ;
	
	///**
	 //* \brief Uncompress data that was previously compressed. The output dest is allocated
	 //* in this function, to the amount specified by dest_size. This should be the 
	 //* size of the original uncompressed data, which presumably you know, because
	 //* you compressed it. Returns a zlib error code if decompression fails.
	 //**/
	//int uncompress(const uchar* src, size_t src_size, uchar* &dest, size_t dest_size); 
	
	/**
	 * \brief Draw a rotated rectangle on an image. The point is to be like
	 * opencv's cv::rectangle function, but to allow rotation.
	 **/
	void rectangleRotated(cv::Mat & image, cv::Point center,cv:: Size size, 
						  double angle, const cv::Scalar& color, int thickness=1, 
						  int lineType=8, int shift=0); 
	
	/**
	 * \brief Calculate derivative image.
	 **/
	void unIntegrate(const cv::Mat& src, cv::Mat &dest, int type=CV_8U);
	 
	 
	 /**
	 * \brief Calculate sqrt of derivative image.
	 **/
	void unSqIntegrate(const cv::Mat& src, cv::Mat &dest, int type=CV_8U); 
	
	 
	/**
	 * \brief Compute RBF values at supplied query points, given input data 
	 * and labels. 
	 *
	 * @param input NxM, N data points, M Dims
	 * @param labels NxO, N data points, O outputs
	 * @param weights Nx1, 1 weight per data point
	 * @param xqueries QxM, Q query points, M Dims
	 * @param tau: variance of gaussian weighting window
	 * @param eps: minimal attention paid to all points
	 *
	 * @return QxO RBF values, Q query points, O output dimensions. 
	 **/
	cv::Mat RBF(const cv::Mat &input, const cv::Mat &labels, const cv::Mat &weights, 
				const cv::Mat &xqueries, double tau=.05, double eps=.001); 
	
}

template<class mattype>	void NMPTUtils::map(cv::Mat &mat, mattype (*function)(mattype)) {
	cv::MatIterator_<mattype> it = mat.begin<mattype>(), it_end = mat.end<mattype>(); 
	for (;it != it_end; it++) {
		*it = function(*it); 
	}
}


template<class mattype> int NMPTUtils::contains(const cv::Mat &mat, mattype val) {
	if (mat.rows == 0 || mat.cols == 0) return 0; 
	cv::MatConstIterator_<mattype> it = mat.begin<mattype>(), it_end = mat.end<mattype>(); 
	for (;it != it_end; it++) {
		if (*it == val) return 1;
	}
	return 0; 
}


template<class mattype> int  NMPTUtils::any(const cv::Mat &mat)  {	
	if (mat.rows == 0 || mat.cols == 0) return 0; 
	cv::MatConstIterator_<mattype> it = mat.begin<mattype>(), it_end = mat.end<mattype>(); 
	for (;it != it_end; it++) {
		if (*it) return 1;
	}
	return 0;
}



#endif
