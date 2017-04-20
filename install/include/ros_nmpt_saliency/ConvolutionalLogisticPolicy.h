/*
 *  ConvolutionalLogisticPolicy.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/10/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _CONVOLUTIONALLOGISTICPOLICY_H
#define _CONVOLUTIONALLOGISTICPOLICY_H

#include <opencv2/core/core_c.h>
#include <iostream>


/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A class for implementing convolutional logistic policies,
 * which were first used in the IPOMDP domain in Butko and Movellan, 2008 (see \ref bib_sec). 
 *
 * Currently, only a few special cases of CLPs are implimented. In the future, we hope to have
 * the most general case coded, along with policy-gradient based optimization methods. 
 *
 * The following policies are implemented: 
 *
 * \li BOX: Convolution kernel is a box-filter.
 * \li GAUSSIAN: Convolution kernel is a gaussian filter.
 * \li IMPULSE: Convolution kernel is an impulse response.
 * \li MAX: Convolution kernel is an impulse response with infinite weight.
 *
 * In the CLP, the belief-image is convolved with some kernel, and then a next fixation
 * is chosen by computing a soft-max over the convolution, and then sampling from
 * the result. The shape of the convolution kernel can be optimized via policy gradient.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class ConvolutionalLogisticPolicy  {
public:
	const static int FULL = 0; 
	
	/**
	 * ConvolutionalLogisticPolicy::BOX - Use box-filter for convolution kernel.
	 **/
	const static int BOX = 1; 
	
	/**
	 * ConvolutionalLogisticPolicy::GAUSSIAN - Use gaussian-envelope for convolution kernel.
	 **/
	const static int GAUSSIAN = 2; 
	
	/**
	 * ConvolutionalLogisticPolicy::IMPULSE - Use impulse response for convolution kernel.
	 **/
	const static int IMPULSE = 3; 
	
	/**
	 * ConvolutionalLogisticPolicy::MAX - Use infinite-impulse response for convolution kernel (fixate mode of belief-distribution).
	 **/
	const static int MAX = 4; 
	
	/**
	 * \brief Constructor: Create a CLP with space allocated to hold parameters
	 * for a grid of size gridSize. 
	 *
	 * The default kernel is a Gaussian envelope with standard-deviation of
	 * five grid cells, and a peak value of 300. Use setPolicy() and setHeuristicPolicyParameters() after 
	 * instantiating the policy to change this. Generally this is done indirectly via the MIPOMDP
	 * setPolicy() and setHeuristicPolicyParameters() functions. 
	 **/
	ConvolutionalLogisticPolicy(CvSize gridSize);
	
	
	/**
	 * \brief Deep Copy Constructor: Create a CLP that is identical to the one 
	 * to copy.
	 **/
	ConvolutionalLogisticPolicy(ConvolutionalLogisticPolicy* clpToCopy);
	
	/**
	 * \brief Default Destructor. 
	 * 
	 * Deallocates all memory associated with the CLP. 
	 **/
	virtual ~ConvolutionalLogisticPolicy(); 
	
	/**
	 *\brief Pick an appropriate point to fixate for the current belief-state (probability that the
	 * object is located at each point).
	 *
	 * In the CLP, the belief-image is convolved with some kernel, and then a next fixation
	 * is chosen by computing a soft-max over the convolution, and then sampling from
	 * the result.
	 *
	 * @param currentBelief The current posterior probability estimate that the object we are 
	 * searching for is located in each respective grid-cell. This belief is maintained and
	 * updated by an MIPOMDP object. 
	 *
	 * @return The grid-cell that should be fixated by the eyes next. Note that in most
	 * cases this is a stochastic output, and calling this function repeatedly for different
	 * results will result in different return values. 
	 **/
	virtual CvPoint getFixationPoint(IplImage* currentBelief); 
	
	/**
	 * \brief Compute the infomax reward associated with this belief state.
	 * 
	 * @param currentBelief The current posterior probability estimate that the object we are 
	 * searching for is located in each respective grid-cell. This belief is maintained and
	 * updated by an MIPOMDP object. 
	 * 
	 * @return The scalar infomax reward associated with this belief state.
	 **/ 
	double getReward(IplImage* currentBelief); 
	
	/**
	 * \brief Tell the CLP what kind of convolution policy to use in the future. 
	 * 
	 * @param policyNumber Must be one of: 
	 * \li ConvolutionalLogisticPolicy::BOX
	 * \li ConvolutionalLogisticPolicy::GAUSSIAN
	 * \li ConvolutionalLogisticPolicy::IMPULSE
	 * \li ConvolutionalLogisticPolicy::MAX
	 **/
	void setPolicy(int policyNumber); 
	
	/**
	 * \brief Tell the CLP the shape of the convolution kernel to use.
	 * 
	 * @param softmaxGain Has the following effect for: 
	 * \li ConvolutionalLogisticPolicy::BOX - The kernel is a square with integral=softmaxGain
	 * \li ConvolutionalLogisticPolicy::GAUSSIAN - The kernel is a Gaussian with integral=softmaxGain
	 * \li ConvolutionalLogisticPolicy::IMPULSE - The kernel is an impulse response with value softmaxGain
	 * \li ConvolutionalLogisticPolicy::MAX - No effect
	 *
	 * @param boxSize Has the following effect for: 
	 * \li ConvolutionalLogisticPolicy::BOX - The kernel is a boxSize x boxSize grid-cell square.
	 * \li ConvolutionalLogisticPolicy::GAUSSIAN - The kernel is a Gaussian with standard-deviation boxSize grid-cells.
	 * \li ConvolutionalLogisticPolicy::IMPULSE - No effect
	 * \li ConvolutionalLogisticPolicy::MAX - No effect
	 **/
	void setHeuristicPolicyParameters(double softmaxGain, double boxSize); 
	
	
	/**
	 * \brief Write to a file.
	 */
	friend std::ostream& operator<< (std::ostream& ofs, ConvolutionalLogisticPolicy* a);
	
	/**
	 * \brief Read from a file.
	 */
	friend std::istream& operator>> (std::istream& ifs, ConvolutionalLogisticPolicy* a);
	
	float randomFloat(); 
protected:	
	void readFromStream(std::istream& in); 
	void addToStream(std::ostream& out); 
	
	CvSize gridSize; 
	double softmaxGain; 
	double boxSize; 
	int polnum; 
	
	IplImage* actProb;       //has size of grid
	
};


std::ostream& operator<< (std::ostream& ofs, ConvolutionalLogisticPolicy* a);
std::istream& operator>> (std::istream& ifs, ConvolutionalLogisticPolicy* a);

#endif
