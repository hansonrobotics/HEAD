/*
 *  PatchList.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/25/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef PATCHLIST_H
#define PATCHLIST_H

#include <opencv2/core/core_c.h>
#include <vector>
#include "ImagePatch.h"
#include "StructTypes.h"


/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A class that does the necessary book keeping
 * for filtering an image quickly at many scales. 
 *
 * Specifically, a PatchList gives BoxFeature, FeatureRegressor, and 
 * GentleBoostCascadedClassifier a way to apply a single weak learner to 
 * all remaining candidate object image patches. GentleBoost requires that
 * each weak learner output be accumulated with the output of other weak 
 * learners. PatchList maintains this accumulator. 
 *
 * They key parameters controlling a PatchList are the size of the image (using
 * setImage), the base patch size, and the scale increment factor. Here as
 * an example with an image of size 320x240, a base patch size of 30, and
 * a scale increment of 1.2. 
 *
 * At scale 0, the image is simply copied, and an integral image is computed. It
 * can be filtered in a region up to 29 pixels from the border by a filter of
 * size 30x30 (which is the base patch size). ImageSize represents the size
 * of this valid filtering region, which is 291x211 pixels and is useful for 
 * displaying an image representation of the fitlering output. Each of these
 * pixels contains the output of the filtering of one patch, so there are 
 * 61401 patches at scale 0.
 *
 * At scale 1, the size of patches to search increases by a factor of 1.2, 
 * giving a patchWidth of 36. Internally, the image is downscaled by a factor
 * of 1.2, giving a 266x200 image, with a 237x171 valid filtering region, and
 * 40527 total patches.
 *
 * At scale 12, the patch size would be 30*(1.2^12)=267x267. This is larger than
 * the height of the image, and so the largest scale is 11. Since the scales
 * start at 0, there are 12 total scales: 
 *
 * The PatchList has 12 scales containing 170925 total patches.
 * \li Scale 0: PatchSize - 30x30; FilterSize - 30x30; ImageSize - 291x211; Patches - 61401
 * \li Scale 1: PatchSize - 36x36; FilterSize - 30x30; ImageSize - 237x171; Patches - 40527
 * \li Scale 2: PatchSize - 43x43; FilterSize - 30x30; ImageSize - 193x137; Patches - 26441
 * \li Scale 3: PatchSize - 51x51; FilterSize - 30x30; ImageSize - 156x109; Patches - 17004
 * \li Scale 4: PatchSize - 62x62; FilterSize - 30x30; ImageSize - 125x86; Patches - 10750
 * \li Scale 5: PatchSize - 74x74; FilterSize - 30x30; ImageSize - 99x67; Patches - 6633
 * \li Scale 6: PatchSize - 89x89; FilterSize - 30x30; ImageSize - 78x51; Patches - 3978
 * \li Scale 7: PatchSize - 107x107; FilterSize - 30x30; ImageSize - 60x37; Patches - 2220
 * \li Scale 8: PatchSize - 128x128; FilterSize - 30x30; ImageSize - 45x26; Patches - 1170
 * \li Scale 9: PatchSize - 154x154; FilterSize - 30x30; ImageSize - 33x17; Patches - 561
 * \li Scale 10: PatchSize - 185x185; FilterSize - 30x30; ImageSize - 22x9; Patches - 198
 * \li Scale 11: PatchSize - 222x222; FilterSize - 30x30; ImageSize - 14x3; Patches - 42
 *
 * Other parameters that can be set are maxSize, minSize, and stepWidth. MaxSize
 * changes the maximum searched patch size. If (0,0), then the smaller image
 * dimension is used. MinSize changes the minimum searched patch size. If (0,0),
 * the base patch size is used. Otherwise, the image is scaled in size by 
 * a factor of basePatchSize/minSize before searching. StepWidth reduces the 
 * number of patches searched by skipping rows and columns. If StepWidth is 1,
 * each patch location is searched. If it is 2, every other one is searched. If
 * it is 2/3, then two consecutive patches are searched and one is skipped.
 * 
 * PatchList is effecicient about not doing more memory allocation than it 
 * needs. A rule of thumb is that giving a PatchList images of different sizes
 * will incur a slowdown, but if all images are the same size, then subsequent
 * calls to setImage will be faster. 
 *
 * For example, a typical time for the first call to setImage with a 320x240 
 * image, a base patch size of 30x30, and a scale increment of 1.2 is 15 ms,
 * while subsequent calls with the same image size and parameters takes 1.5 ms.
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */
class PatchList{
	//A class that does the nessecary book keeping for filtering an image 
	//at many scales
	//void filterPatchWithBox(FeatureRegressor* reg, int atWidth, int atHeight, vector<integral_type*>& srcInds, vector<double*>& destInds); 
public:  
	//static int default_width; 
	//static int default_height; 
	
	/**
	 * \brief Constructor.
	 *
	 * @param minSize Don't search for objects smaller than minSize. If minSize
	 * is (0,0) then baseObjectSize is used. 
	 * @param maxSize Don't search for objects larger than maxSize. If maxSize
	 * is (0,0), then the upper bound on the object size is the smallest 
	 * dimension of the image size. 
	 * @param scaleInc Relative size of the object in the next scale, compared
	 * to the current one. scaleInc must be greater than 1. 
	 * @param stepWidth Search interval between subsequent patches. A stepWidth
	 * of 1 means slide the object evaluation window over one pixel. 2 means 
	 * only evaluate patches at every other pixel. 1.5 means search two and 
	 * skip the third, etc.
	 * @param basePatchSize The size of the patches that will be evaluated 
	 * for containing the object. This should match the size of whatever
	 * filter will be applied to the patches. It may be set later, using
	 * setBasePatchSize(). At subsequent scales, the image is scaled up or 
	 * down relative to this size.
	 */
	PatchList(cv::Size minSize=cv::Size(0,0), cv::Size maxSize=cv::Size(0,0), 
			  double scaleInc=1.2, double stepWidth=1,
			  cv::Size basePatchSize=cv::Size(30,30)); 
	
	
	/**
	 * \brief Destructor
	 */
	virtual ~PatchList(); 
	
	/**
	 * \brief A list of pointers to the top-left of image patches still 
	 * considered as candidate locations of the object. This pointer is exposed
	 * to allow BoxFeature to evaluate the image with maximal efficiency. 
	 */
	integral_type** srcInds;  //cleared in clearPointers
	
	/**
	 * \brief A list of pointers to corresponding to the filtering outputs of 
	 * the remaining candidate patches in srcInds. 
	 * 
	 * This is a place for 
	 * BoxFeature, FeatureRegressor, GentleBoostCascadedClassifier etc. 
	 * to store their outputs. 
	 */
	double** destInds;        //cleared in clearPointers
	
	
	/**
	 * \brief Get the base patch size, which is size of the object detector that
	 * you plan to apply. It is also the default search minSize, and other
	 * scale sizes are computed relative to it.
	 * 
	 * @return The base patch size.
	 */
	cv::Size getBasePatchSize(); 
	
	
	/**
	 * \brief Set the base patch size, which is size of the object detector that
	 * you plan to apply. It is also the default search minSize, and other
	 * scale sizes are computed relative to it.
	 *
	 * This can be changed somewhat freely, but setImage() must be called afterward.
	 * 
	 * @param baseSize The base patch size.
	 */
	void setBasePatchSize(cv::Size baseSize); 
	
	/**
	 * \brief Set the image to filter or process for object detection.
	 *
	 * @param newImage The image to process must be a single channel image of 
	 * IPL_DEPTH_8U. It can be of any size, but passing images of the same size
	 * repeatedly will boost performance. 
	 */
	void setImage(const cv::Mat &newImage);
	DEPRECATED(void setImage(IplImage* newImage)); 
	
	/**
	 * \brief Prepare the data structure to search for objects at a certain
	 * scale. 
	 *
	 * You must call setImage() before because it is impossible to determine
	 * various scale info without knowing the image size. When querying
	 * scales for information, the maximum valid value is getNumScales()-1, and
	 * the minimum is 0. 
	 * 
	 * Scales number from 0 to getNumScales()-1. 0 searches for smallest
	 * objects, getNumScales()-1 searches for the largest.
	 */
	void resetListToScale(int scale); 
	
	/**
	 * \brief Adds the values in destInds to an accumulator. Any accumulator
	 * values that are below threshold are removed from the list.
	 *
	 * @param threshold Accumulator values below threshold will be removed
	 * from the PatchList. -INFINITY will insure all patches are kept.
	 */
	void accumulateAndRemovePatchesBelowThreshold(double threshold); 
	
	/**
	 * \brief Remove image patch candidates that have a nearby patch (within
	 * radius horizontal or vertical pixels) with a higher value. 
	 * 
	 * @param radius Radius (in pixels) to do non-maximal suppression. Pixels
	 * are scale-adjusted, so the suppression region grows with the size of
	 * the object.
	 */
	void keepOnlyLocalMaxima(int radius); 
	
	/**
	 * \brief Get info about the patches that have not yet been removed from
	 * the PatchList since calling resetListToScale(). Each SearchResult 
	 * contains info about the location of the patch in the original image,
	 * and its current accumulator value.
	 * 
	 * @param searchResults The vector that will be filled with the remaining 
	 * patches. The contents of the vector passed in are entirely overwritten.
	 */
	void getRemainingPatches(std::vector<SearchResult>& searchResults); 
	
	
	void getRemainingPatches(std::vector<SearchResult>& searchResults, std::vector<cv::Rect>blackoutRegions, int spatialRadius=0, int scaleRadius=0); 
	
	
	/**
	 * \brief Compute a pixel-by-pixel estimate for this scale that there 
	 * is an object located at that pixel. 
	 */
	void getProbImage(cv::Mat &dest, double faceAreaPrior = .5, double faceInvisibleProb = .1); 
	DEPRECATED( IplImage* getProbImage(double faceAreaPrior = .5, double faceInvisibleProb = .1)); 
	
	/**
	 * \brief Get the effective object size if searching at a given (or the 
	 * current) scale. 
	 * 
	 * You must call setImage() before because it is impossible to determine
	 * the number of patch sizes without knowing the image size. When querying
	 * scales for information, the maximum valid value is getNumScales()-1, and
	 * the minimum is 0. 
	 * 
	 * @param scale The scale to query. If -1, use the current. Otherwise, any 
	 * valid scale can be queried.
	 */
	cv::Size getPatchSizeAtScale(int scale=-1); 
		
	/**
	 * \brief Get the filter size that should be applied at this scale. For
	 * PatchList, this is always getBasePatchSize(). In PatchList, this always
	 * returns getBasePatchSize(). In FastPatchList, this always returns
	 * getPatchSizeAtScale(scale). 
	 * 
	 * You must call setImage() before. When querying
	 * scales for information, the maximum valid value is getNumScales()-1, and
	 * the minimum is 0. 
	 * 
	 * @param scale The scale to query. If -1, use the current. Otherwise, any 
	 * valid scale can be queried.
	 */
	cv::Size getFilterSizeAtScale(int scale=-1); 
	
	/**
	 * \brief Get the size of the valid convolution image, for viewing filtering
	 * and accumulator results.
	 * 
	 * You must call setImage() before because it is impossible to determine
	 * the filtered image size without knowing the image size. When querying
	 * scales for information, the maximum valid value is getNumScales()-1, and
	 * the minimum is 0. 
	 * 
	 * @param scale The scale to query. If -1, use the current. Otherwise, any 
	 * valid scale can be queried.
	 */
	cv::Size getImageSizeAtScale(int scale=-1); 
	
	/**
	 * \brief Get the width step of the integral image, i.e. how many 
	 * integral_type elements are there in srcInds[i][j] before a pixel that
	 * is directly below [j]. 
	 * 
	 * You must call setImage() before because it is impossible to determine
	 * the size of the integral image without knowing the image size. When querying
	 * scales for information, the maximum valid value is getNumScales()-1, and
	 * the minimum is 0. 
	 * 
	 * @param scale The scale to query. If -1, use the current. Otherwise, any 
	 * valid scale can be queried.
	 */
	int getIntegralWidthStepAtScale(int scale=-1); 
	
	/**
	 * \brief Get the number of remaining patches at this scale. Initially, this
	 * has all patches in the scale, but may be reduced by repeated calls to 
	 * accumulateAndRemovePatchesBelowThreshold(). 
	 * 
	 * @return Number of patches that have not been removed.
	 */
	int getCurrentListLength(); 
	
	/**
	 * \brief Get the total number of patches available to this PatchList at 
	 * all scales.
	 * 
	 * @return Total number of patches available to this PatchList.
	 */
	int getTotalPatches(); 
	
	/**
	 * \brief Get the total number of scales. 
	 * 
	 * You must call setImage() before because it is impossible to determine
	 * the number of scales without knowing the image size. When querying
	 * scales for information, the maximum valid value is getNumScales()-1, and
	 * the minimum is 0. 
	 * 
	 * @return Total number of patches available to this PatchList.
	 */
	int getNumScales(); 

	
	/**
	 * \brief Get patches near a region of interest. This is useful for augmenting
	 * the number of positive examples in a dataset by grabbing nearby patches,
	 * or for finding the image patch that is closest to a labelled location. 
	 * 
	 * @param roi A region of interest in the image.
	 * @param spatialRadius Take patches that are slightly
	 * left/right/up/down from roi.Note that a radius of 1 gives 9  
	 * patches, and a radius of 2 gives 25. 
	 * @param scaleRadius Take patches that are slightly 
	 * larger or smaller than the roi. This size is radius is in
	 * terms of search scale, not pixels.  Note that a radius of 1 gives 3
	 * patches, and a radius of 2 gives 5.	 
	 */
	std::vector<ImagePatch*> getNearbyPatches(cv::Rect roi, int spatialRadius=0, int scaleRadius=0); 
	
	
	/**
	 * \brief Get the exact same pixels from the image patch that the filtering
	 * process sees. This is useful for pulling out hard background patches for
	 * training.
	 * 
	 * @param im Destination image for pixels. 
	 * 
	 * @param r A Patch SearchResult from getRemainingPatches() to query.
	 */
	void fillImageWithPixelsOfSearchPatch(cv::Mat &dest, SearchResult r);
	DEPRECATED(void fillImageWithPixelsOfSearchPatch(IplImage* im, SearchResult r));
	
	
	/**
	 * \brief Improve efficiency by only copying integral data (not image data). This
	 * makes fillImageWithPixelsOfSearchPatch fail. 
	 * 
	 * @param flag Set to non-zero if you want efficiency, but not to look at patches.
	 */
	void setDontCopyImageData(int flag=0); 
	
	/**
	 * \brief Improve detail of patches when getting pixels of patch -- at the cost of
	 * extra memory. Only affects FastPatchList. 
	 * 
	 * @param flag Set to non-zero if you want big patches for training with FastPatchList.
	 */
	void setDontScaleDownPatches(int flag=0); 
	
	/**
	 * \brief Output of image filtering process as an image which can be 
	 * visualized. 
	 *
	 * This image contains that data that is written in destInds
	 * by BoxFeature and FeatureRegressor and GentleBoostCascadedClassifier. 
	 */
	void getFilterImage(cv::Mat &image); 
	
	
	/**
	 * \brief Output of filtering accumulation process as an image which can be 
	 * visualized. 
	 *
	 * This image contains that data that was accumulated by successive calls
	 * to accumulateAndRemovePatchesBelowThreshold(). 
	 */
	void getAccumImage(cv::Mat &image); 
		
protected: 
	
	//
	// \brief Get the exact same pixels from the image patch that the filtering
	// process sees. This is useful for pulling out hard background patches for
	// training.
	// 
	// @param im Destination image for pixels. 
	// 
	// @param patchIndex Index of a patch in getRemainingPatches() to query.
	///
	
	//void fillImageWithPixelsOfPatchIndex(IplImage* im, int patchIndex); 
	
	void resetContainerSizes(); 
	
	cv::Mat filterImage;    //cleared in clearPointers
	
	cv::Mat accumulatorImage; //cleared in clearPointers
	
	cv::Mat scaleCanvas; 
	
	cv::Mat indImage; 
	
	//IplImage* scaleCanvas;                 //cleared in destructor
	
	int numScales; 
	int currentScale;
	int totalPatches; 
	int currentListLength; 
	cv::Size origImageSize; 
	
	std::vector<ImagePatch*> images;            //cleared in destructor
	
	std::vector<int> numAtScale; 
	
	std::vector<cv::Size> scalePatchSizes; 
	std::vector<cv::Size> scaleImageSizes; 
	std::vector<cv::Size> scaleFilterSizes; 
	
	//std::vector<int*> accInds; 
	//std::vector<int> imLoc; 
	std::vector<int*> imLocAtScales; 
	std::vector<double**> destsAtScales; 
	std::vector<double**> accAtScales; 
	std::vector<const integral_type**> srcAtScales; 
	
	cv::Size minsize, maxsize; 
	double stepwidth, scaleinc; 
	int shouldResetAccumulator; 
	
	double** accInds;                      //cleared in clearPointers
	int* imLoc;                            //cleared in clearPointers
	
	//int** imLocAtScales;                   //cleared in clearPointers
	//const integral_type*** srcAtScales;    //cleared in clearPointers
	//double*** destsAtScales;               //cleared in clearPointers
	//double*** accAtScales;                 //cleared in clearPointers
	//IplImage* indImage;                    //cleared in clearPointers
	
	CvPoint matLocOfImLoc(double* accIndsPtr); 
	int isLocalMaximum(double* accIndsPtr, int radius); 
	
	cv::Size getMaxEffectivePatchSize(); 
	cv::Size getMinEffectivePatchSize() ; 
	int getEffectiveNumScales() ; 
	double getBaseWidthScale();
	double getBaseHeightScale(); 
	double getEffectiveBaseWidthRatio(int scale);
	double getEffectiveBaseHeightRatio(int scale); 
	cv::Size getEffectivePatchSizeAtScale(int scale) ;
	cv::Size getEffectiveResampledImageSizeAtScale(int scale) ;
	
	void checkAndWarn(int scale); 
	
	void clearPointers();
	virtual void resetPointers();
	void setNullPointers(); 
	virtual int setImageAllScalesNeedsPointerReset(IplImage* newImage) ;
	
	std::vector<SearchResult> getNearbySearchResults(cv::Rect roi, int spatialRadius, int scaleRadius); 
	
	int getScaleNearROI(cv::Rect roi, int scaleRadius);
	
	int oldNumScales; 
	
	int default_width; 
	int default_height; 
	
	int copyImageData; 
	int scaleDownPatches; 
	
};

#endif
