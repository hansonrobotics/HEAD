
#ifndef __FASTSALIENCE
#define __FASTSALIENCE

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include "OpenCV2BoxFilter.h"

/**
 * \ingroup MPGroup
 * \brief <tt> <b> Machine Perception Primitive: </b> </tt> An implementation of the "Fast Salience Using Natural-statistics" algorithm
 * from Butko, et al., 2008. FastSUN is an efficient implementation of Zhang et al.'s SUN
 * algorithm, which is documented in Zhang, et al., 2008 (see \ref bib_sec).
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */
class FastSalience {
public:
	/**
	 * \brief Detailed Constructor.
	 * @param salwidth The width of the input images supplied to the algorithm. Note: No image scaling is
	 * done by the FastSalience algorithm, so if you want a salience map of a downscaled image, you must
	 * scale the image down to \c salwidth before calling updateSalience. 
	 * @param salheight The height of the input images supplied to the algorithm. Note: No image scaling is
	 * done by the FastSalience algorithm, so if you want a salience map of a downscaled image, you must
	 * scale the image down to \c salheight before calling updateSalience. 
	 * @param numtemporal Number of timescales of Difference of Expontential filters to track. 
	 * @param numspatial Number of sizes of Difference of Box filters to use. 
	 * @param firsttau Exponential Falloff parameter for the first Difference of Exponentials scale. Lower numbers
	 * give slower falloff. Must be greater than 0. 
	 * @param firstrad Radius of smallest Difference of Boxes filter center. The diameter of the box is 2*rad+1, so 
	 * the smallest allowed first radius is 0. 
	 */
	FastSalience(int numtemporal, int numspatial, float firsttau=1.0, int firstrad=0);
	
	
	/**
	 * \brief Default Constructor: construct a FastSalience feature detector
	 * with default parameters.
	 **/ 
	FastSalience(); 
	
	
    virtual void detect( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
						const cv::Mat& mask=cv::Mat() ) ;
	
    virtual void read( const cv::FileNode& fn ) ;
    virtual void write( cv:: FileStorage& fs ) const ;
	
	friend cv::FileStorage& operator << (cv::FileStorage &, const FastSalience &); 
	friend cv::FileNode& operator >> (cv::FileNode &, const FastSalience &); 
	
	/**
	 * \brief Simple Destructor.
	 *
	 * Deallocates all memory associated with the FastSalience object. 
	 */
	virtual ~FastSalience(); 
	
	/**
	 * \brief Compute the salience map for a new input frame. 
	 * @param colorframe A BGR Color image with width=salwidth and height=salheight. The input 
	 * image type should be CV_8U3, CV_8U1, CV_32FC1, or CV_32FC3. The salience map 
	 * that is computed is accessible via getSalImageDouble and getSalImageFloat
	 */
	void updateSalience(const cv::Mat &image) ;
	
	/**
	 *\brief The current salience map -- negative log likelihood values.
	 */
	void getSalMap(cv::Mat &dest) const; 	
	
	/**
	 *\brief The current salience iamge,  normalized to the range 0-1 for display.
	 */
	void getSalImage(cv::Mat &dest) const; 
	
	
	/**
	 * \brief Set the Generalized-Gaussian feature distribution to be something other than Laplacian. 
	 *
	 * Image features often exhibit marginal histograms that are of a generalized Gaussian form. The power
	 * of these distributions is often between .5 and .7, but performing this power computation
	 * is numerically slow. By using a Laplacian distribution (power of one) by defuault, the salience
	 * estimate is worse, but much faster to compute. Setting "power" to some lower value may give better
	 * but slower resultes.
	 *
	 * @param value	The power of a generalized-gaussian feature distribution. 
	 */	void setGGDistributionPower(double value);
	
	/**
	 *\brief Toggle use of Difference of Exponential (motion) features. On by default. 
	 * @param flag	If 0, turn off DoE features. Otherwise, calculate them. 
	 */
	void setUseDoEFeatures(int flag); 
	
	/**
	 *\brief Toggle use of Difference of Box (spatial) features. On by default. 
	 * @param flag	If 0, turn off DoB features. Otherwise, calculate them. 
	 */
	void setUseDoBFeatures(int flag); 
	
	
	/**
	 *\brief Toggle use of Color Contrast (red-green and blue-yellow) information. On by default. 
	 * @param flag	If 0, turn off Color Contrast information. Otherwise, calculate it. 
	 */
	void setUseColorInformation(int flag); 
	
	/**
	 * \brief Toggle use of custom Generalized Gaussian Distribution parameters. On by default. 
	 *
	 * If this is off, we assume that image features are drawn from a Laplace distribution of zero
	 * mean and unit variance. However, these parameters can be estimated and adjusted online 
	 * to better reflect the statistics of the current environment. If on, these online estimates will
	 * be used (at small computational cost). 
	 *
	 * @param flag	If 0, turn off the use of histogram estimate information. Otherwise, use it. 
	 */
	void setUseGGDistributionParams(int flag); 	
	
	/**
	 * \brief Toggle estimation of custom Generalized Gaussian Distribution parameters. On by default. 
	 *
	 * Initially we assume that image features are drawn from a Laplace distribution of zero
	 * mean and unit variance. However, these parameters can be estimated and adjusted online 
	 * to better reflect the statistics of the current environment. If off, the current
	 * estimate of the mean and variance are used. If on, the estimate updates with each new frame
	 * (at small computational cost).  One approach would be to estimate the parameters for 
	 * a few frames, and then when the estimates no longer change much, to turn off estimation
	 * and just use the "learned" parameters. 
	 *
	 * @param flag	If 0, turn off estimation of histogram information. Otherwise, calculate it. 
	 */
	void setEstimateGGDistributionParams(int flag); 
	
	/**
	 * \brief Find key-point interest detectors using non-maximal suppression
	 * on the salience map.
	 *
	 * @param radius	Non-maximal suppression radius.
	 */
    std::vector<cv::KeyPoint> getKeyPoints(int radius=2) const; 
	
	FastSalience(const FastSalience &copy); 
	FastSalience & operator=(const FastSalience &rhs); 
	
private:
	
	//Volatile variables
	cv::Mat redChannel, greenChannel, blueChannel; 
	
	OpenCV2BoxFilter channelFilter; 
	
	std::vector<cv::Mat> redBoxConvolutionAtScale, greenBoxConvolutionAtScale, blueBoxConvolutionAtScale;
	
	cv::Mat rgDoB, byDoB, iDoB, DoB, DoE, salImageDouble, salImageFloat;  
	//std::vector<cv::Mat>  DoE; 
	
	std::vector<cv::Mat> temporalImageI, temporalImageRG, temporalImageBY; 
	
	
	
	void init(int numtemporal=2, int numspatial=6, float firsttau=1.0, int firstrad=0);
	void copy(const FastSalience &rhs); 
	void calcLogProb(cv::Mat &logProb, cv::Mat &accum, const cv::Mat &scaleBig, 
					 const cv::Mat &scaleSmall, double &meanEst, double &absMeanEst ) const ; 
	
	//Persistent Variables
	int useDoB; 
	int useDoE; 
	int useColor; 
	int useParams;
	int estParams; 
	
	double power; 
	
	int nspatial,ntemporal;
	//int height, width; // size of salience map
	
	float tau0; //tau[0]
	int rad0; // rad[0] the samllest box radius size, box border = 2*r+1
	
	cv::Mat tau, rad; 
	
	cv::Mat absMeanDoBI, absMeanDoBRG, absMeanDoBBY; 
	cv::Mat absMeanDoEI, absMeanDoERG, absMeanDoEBY; 
	
	cv::Mat meanDoBI, meanDoBRG, meanDoBBY; 
	cv::Mat meanDoEI, meanDoERG, meanDoEBY; 

	double ALPHA; 
};

cv::FileStorage& operator << (cv::FileStorage &, const FastSalience &); 
cv::FileNode& operator >> (cv::FileNode &, FastSalience &); 


#endif
