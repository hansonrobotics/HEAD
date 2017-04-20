/*
 *  MatrixKalmanFilter.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 6/1/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef _KALMANFILTER_H
#define _KALMANFILTER_H

#include <opencv2/core/core.hpp>
#include <iostream>

#include "DebugGlobals.h"
#include "StructTypes.h"


/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A Kalman Filter for estimating the values
 * of a matrix, which is like doing online linear regression from a probability
 * theory perspective.
 *
 * Linear Regression problems take the form Cx = y. x and y are observed, and
 * C must  be inferred from data. The Kalman Filter gives a Mean estimate of 
 * the values of C's elements, and maintains a covariance matrix Sigma of 
 * uncertainties on those estimates.
 *
 * There are many parameters to a Kalman Filter that may be set:
 *
 * It is useful to think of there being a stochastic relation between Cx and y,
 * specifically y is drawn from a normal distribution with mean Cx and Covariance
 * Q. 
 *
 * It is also useful to think of there being a drift in C, i.e. its values may 
 * change over time. This drift is 0 mean, and has covariance R. If R is 0, you
 * are making a strong assertion that the elements of C will never change.
 *
 * Finally, you may set a prior mean estimate Mu of C, and a prior uncertainty,
 * Sigma. 
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */
class MatrixKalmanFilter {	
public:
	
    
    /**
	 * \brief Assignment Operator. Perform a deep copy of another MatrixKalmanFilter.
	 **/
	MatrixKalmanFilter& operator=(const MatrixKalmanFilter& rhs);
    
    /**
	 * \brief Copy Constructor. Perform a deep copy of another MatrixKalmanFilter.
	 **/
	MatrixKalmanFilter(const MatrixKalmanFilter &copy); 
	
    
    /**
	 * \brief Default Constructor. Initializes a trivial Matrix Kalman Filter with 
     * one state and one observation (i.e. it would estimate a scalar value).
	 **/
	MatrixKalmanFilter(); 
    
    /**
	 * \brief Main Constructor. Initialize with numStates (size of the input vector x) and numObs
     * (size of the output vector y in xC=y). 
     * 
     * MuPrior, SigmaPrior, R, and Q are set to default values. Use setter functions to modify them.
	 *
	 * @param numStates The number of inputs, i.e. the length of the vector x in Cx=y, when we are
     * trying to estimate C. The matrix C has size numStates x numObs.
	 * 
	 * @param numObs The number of outputs, i.e. the length of the vector y in Cx=y.The matrix 
     * C has size numStates x numObs.
	 **/
	MatrixKalmanFilter(size_t numStates, size_t numObs=1); 
    
    /**
	 * \brief Destructor
     */
	virtual ~MatrixKalmanFilter(); 
	
	
    /**
     * \brief Set the mean of the estimate of C to have each element with identical value.
     */
	void setMuPrior(double val = 0.0); 
    
    /**
     * \brief Set the uncertainty in the estimate of C to a diagonal matrix with identical
     * values along the diagonal.
     */
	void setSigmaPrior(double diagval = 1.0); 
    
    /**
     * \brief Set R, the drift in the estimate of C, to a diagonal matrix with identical
     * values along the diagonal.
     */
	void setR(double diagval = 0.0); // dynamics noise
    
    /**
     * \brief Set Q, the noise in observations y, to a diagonal matrix with identical
     * values along the diagonal.
     */
	void setQ(double diagval = 1.0); // obs noise
	
    
    /**
     * \brief Set the mean of the estimate of C to a specific matrix.
     */
	void setMuPrior(const cv::Mat &muPrior); 
    
    
    /**
     * \brief Set the uncertainty in the estimate of C to a specific matrix.
     */
	void setSigmaPrior(const cv::Mat &sigmaPrior); 
    
    /**
     * \brief Set R, the drift in the estimate of C, to a specific matrix.
     */
	void setR(const cv::Mat &rVal);
    
    /**
     * \brief Set Q, the noise in observations y, to a specific matrix.
     */
	void setQ(const cv::Mat &qVal); 
	
    /**
     * \brief Set Q, the noise in observations y, to be a diagonal matrix
     * with values given by the vector qVal. qVal must have numObsx1 elements.
     */
	void setQDiag(const cv::Mat &qVal); 
	
    /**
     * \brief Reinitialize this MatrixKalmanFilter to have a new number of 
     * states and observations. This resets all default values for Mu, Sigma,
     * Q, and R.
     */
	void setNumStatesAndObs(size_t numStates=1, size_t numObs=1); 
	
	
    /**
     * \brief Compute the expected value of y given x and the current
     * estimate of C, i.e. Cx = y.
     * 
     * @param x Input vector
     *
     * @param y Output vector: computation is passed back out by y.
     */
	void getObsMean(const cv::Mat &x, cv::Mat &y); 
    
    /**
     * \brief Compute log likelihood of y given x, based on the current
     * mean and variance in the estimate of C.
     **/
	double obsLogLikelihood(const cv::Mat &x, const cv::Mat &y); 
    
    /**
     * \brief Compute log likelihood of y given x, based on the current
     * mean and variance in the estimate of C.
     **/
	likelihood modelLogLikelihood(const cv::Mat &x, const cv::Mat &y); 
    
    /**
     * \brief Update the current mean and variance in the estimate of C
     * based on a newly observed input (x) and output (y)
     **/
	void updateModel(const cv::Mat &x, const cv::Mat &y); 
    
    /**
     * \brief Rectify the estimate of C so that no elements are less than 0.
     * Calling this after updateModel will approximate non-negative matrix
     * factorization.
     **/
	void rectify(); 
	
    /**
     * \brief The vectorized mean of the current C estimate,
     * row major. This vector has size (numStates * numObs) x 1. This is the
     * actual underlying estimate data. It is exposed for convenient inspection.
     * Don't modify it.
     **/	
	cv::Mat mu;        // estimate of motion parameters, mn x 1
    
    /**
     * \brief The covariance matrix for the current C estimate.
     * This vector has size (numStates * numObs) x (numStates * numObs) . This is the
     * actual underlying estimate data. It is exposed for convenient inspection.
     * Don't modify it.
     **/	
	cv::Mat Sigma;   // estimate of motion uncertainty, mn x mn
	
	
	/**
	 * \brief Write to a file.
	 */
	friend std::ostream& operator<< (std::ostream& ofs, const MatrixKalmanFilter &feat); 
	
	/**
	 * \brief Read from a file.
	 */
	friend std::istream& operator>> (std::istream& ifs, MatrixKalmanFilter &feat); 
	
	
private:
	
	void copyFrom(const MatrixKalmanFilter &copy); 
	
	cv::Mat R;       // motion parameter drift parameter, mn x mn
	cv::Mat Q;       // motion reliability parameter, n x n
	
	
	cv::Mat matNx1a; 
	cv::Mat matNx1b; 
	cv::Mat matMNxMNa; 
	cv::Mat matMNxMNb; 
	cv::Mat matMNxNa; 
	cv::Mat matMNx1a; 
	cv::Mat matNxNa; 
	cv::Mat mat1x1a; 
    
    void getCMatFromFeatures(const cv::Mat &features, cv::Mat &cMat);  // features block mat, nxmn
    
	
	
	void addMatrixToStream(std::ostream& out, const cv::Mat &matrix) const; 
	cv::Mat readMatrixFromStream(std::istream &in); 
	
	virtual void readFromStream(std::istream& in); 
	virtual void addToStream(std::ostream& out) const; 
	
}; 

std::ostream& operator<< (std::ostream& ofs, const MatrixKalmanFilter &model) ;
std::istream& operator>> (std::istream& ifs, MatrixKalmanFilter &model) ;
#endif