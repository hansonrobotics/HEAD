/*
 *  ImagePatchPyramid.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef _IMAGEPATCHPYRAMID_H
#define _IMAGEPATCHPYRAMID_H

#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

#include "ObjectDetector.h"
#include "OpenCVHaarDetector.h"	

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> The main data structure for the MIPOMDP algorithm. It is
 * reponsible for generating the MIPOMDP observation vector. The ImagePatchPyramid (IPP) is 
 * described in detail in Butko & Movellan 2009 (see \ref bib_sec). 
 * 
 * The IPP data structure divides an image up into grid-cells. It then extracts image patches 
 * comprising different numbers of grid-cells, and scales them down to a common size, and 
 * searches each patch for the target object. Any objects that are found in any grid-cell in any patch
 * are added to a count-vector element corresponding to that grid-cell. This count-vector
 * forms the bases for the MIPOMDP Observation. 
 *
 * In order to perform the above behavior, an IPP needs to know how big the images it receives are,
 * to what smaller size it should scale each image patch, the size (height/width) of the grid-cell
 * tiling of the image, how many image patches there will be, the size (height/width) in grid-cells
 * of each image patch, and what object detector to apply. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */

class ImagePatchPyramid  {
public: 
	
	/**
	 * \brief Main Constructor: Manually create an IPP. 
	 *
	 * An IPP needs to know how big the images it receives are,
	 * to what smaller size it should scale each image patch, the size (height/width) of the grid-cell
	 * tiling of the image, how many image patches there will be, the size (height/width) in grid-cells
	 * of each image patch, and what object detector to apply. 
	 *
	 * @param inputImageSize The size of the images that will be given to the IPP to turn into MIPOMDP
	 * observations. This allocates memory for underlying data, but it can be changed easily later 
	 * if needed without recreating the object by calling the changeInputSize() functions. 
	 * 
	 * @param subImageSize The common size to which all image patches will be reduced, creating the
	 * foveation effect. The smaller subImageSize is, the faster search is, and the more extreme the 
	 * effect of foveation. This allocates memory for underlying data, but it can be changed easily 
	 * later if needed without recreating the object by calling the changeInputSize() functions.
	 *
	 * @param gridSize Size of the discretization of the image. The number of POMDP states is the 
	 * product of the demensions of this size (e.g. 21x21).
	 * 
	 * @param numSubImages Number of Patches in the Image Patch Pyramid. 
	 * 
	 * @param subImageGridPoints A matrix that describes the size and shape of each level (patch) of
	 * the IP Pyramid. This must be a matrix with size [numSubImages x 2]. Each row contains the 
	 * width and height of the corresponding levels. These should be in order of *decreasing* size,
	 * so that the largest Image Patch is first. For example, in Butko and Movellan CVPR 2009, we
	 * used [21 21; 15 15; 9 9; 3 3]. Finaly, note that it is not necessary that the largest patch
	 * cover the entire image. However, when the largest patch is the same size as grid-cell-matrix,
	 * special optimizations become available that reduce the complexity of the algorithm when the 
	 * same image, or same frame of video, is fixated multiple times. 
	 *
	 * @param detector A pointer to an object detector. This object detector will be applied to 
	 * each patch in the IPP, forming the basis for the MIPOMDP observation model. 
	 **/
	 
	ImagePatchPyramid(CvSize inputImageSize, CvSize subImageSize, CvSize gridSize, 
					  int numSubImages, CvMat* subImageGridPoints, OpenCVHaarDetector* detector); 
	
	/**
	 * \brief Placeholder Constructor. 
	 * 
	 * Used to create a placeholder for an IPP, which can then be read 
	 * from a file stream using the \>\> operator. Typical usage for this constructor is:
	 *
	 * <code>
	 * ipp = new ImagePatchPyramid(); <br>
	 * in >> ipp; 
	 * </code>
	 */
	ImagePatchPyramid();
	
	/**
	 * \brief Deep Copy Constructor: Create an IPP that is identical to 
	 * the one copied.
	 **/
	ImagePatchPyramid(ImagePatchPyramid* ippToCopy);
	
	/**
	 * \brief Default Destructor. 
	 * 
	 * Deallocates all memory associated with the IPP. 
	 */
	virtual ~ImagePatchPyramid(); 
	
	/**
	 *\brief The main method for generating an observation vector: given an image,
	 * generate a count of object-detector firings in each grid-cell based on a 
	 * fixation point. After this method is called, the resulting observation is 
	 * stored in the objectCount element. 
	 *
	 * @param grayFrame The image to search. This image should have size inputImageSize,
	 * and be of type IPL_DEPTH_8U with a single channel. 
	 * 
	 * @param searchPoint The grid-cell center of fixation. 
	 **/
	virtual void searchFrameAtGridPoint(IplImage* grayFrame,  CvPoint searchPoint); 
	
	/**
	 * \brief Apply the object detector to the entire image. This is used for comparing
	 * the speed and accuracy of the foveated search strategy. After this method is 
	 * called, the count of objects that the object detector found in each grid-cell 
	 * in the high resolution image is stored in objectCount. The location of the object
	 * is inferred as being the grid-cell with the highest count.
	 * 
	 * @param grayFrame The image to search. This image should have size inputImageSize,
	 * and be of type IPL_DEPTH_8U with a single channel. 
	 * 
	 * @return The grid-cell location with the highest count. 
	 **/ 
	virtual CvPoint searchHighResImage(IplImage* grayFrame); 
	
	/**
	 * \brief Number of times the object-detector fired in each grid-cell suring the
	 * last call to searchFrameAtGridPoint() or searchHighResImage(). 
	 * 
	 * Has IplImage type IPL_DEPTH_8U, 1 channel. 
	 **/
	IplImage* objectCount; //has size of grid	
	
	
	/**
	 * \brief A visually informative representation of the IPP Foveal represention. 
	 *
	 * This is meant as an image that is appropriate for display in a GUI, to visualize the 
	 * algorithm in action. The image has the same size as inputImageSize. In order to 
	 * increase efficiency, generation of this visualization should be disabled if it
	 * is not going to be accessed. This can be achieved by calling setGeneratePreview(0). 
	 **/
	IplImage* foveaRepresentation; //has size of full gray image
	
	/**
	 * \brief The total number of levels that the IP Pyramid has. 
	 *
	 * Note that this may be different from the number of scales that the IP Pyramid is 
	 * *using*. If the subImageSize is too small (below getMinSize()), the smallest scale 
	 * is dropped and subImageSize is scaled up proportionally to the next scale. This process
	 * is repeated until subImageSIze is greater than getMinSize(). To find out how many 
	 * scales that the IPP is using, call getUsedScales(). 
	 **/
	int getNumScales(); 
	
	
	/**
	 * \brief The total number of levels that the IP Pyramid is currently using. 
	 *
	 * Note that this may be different from the number of scales that the IP Pyramid  
	 * has. If the subImageSize is too small (below getMinSize()), the smallest scale 
	 * is dropped and subImageSize is scaled up proportionally to the next scale. This process
	 * is repeated until subImageSIze is greater than getMinSize(). To find out how many 
	 * scales that the IPP is has, call getNumScales(). 
	 **/
	int getUsedScales(); 
	
	/**
	 * \brief The height (measured in grid-cells) of the ith largest scale, indexed from 0.
	 **/
	int getGridCellHeightOfScale(int i); 
	
	/**
	 * \brief The width (measured in grid-cells) of the ith largest scale, indexed from 0.
	 **/
	int getGridCellWidthOfScale(int i); 
	
	/**
	 *\brief Find the region of the belief map that is visible in any scales 
	 * when fixating a grid-point.
	 * 
	 * NOTE: We assume that all patches are concentric, and that the largest
	 * image patch comes first. 
	 *
	 * @param searchPoint The grid-cell center of fixation. 
	 **/
	CvRect getVisibleRegion(CvPoint searchPoint); 
	
	/**
	 * \brief The expected size of the next input image. 
	 **/
	CvSize getInputImageSize(); 
	
	/**
	 * \brief The common reference size that image patches are down-scaled to. 
	 *
	 * This is not necessarily the value set in the constructor, or in 
	 * changeInputImageSize(). If the requested subImageSize is too small 
	 * (below getMinSize()), the smallest scale is dropped and subImageSize is scaled 
	 * up proportionally to the next scale. This process is repeated until subImageSIze 
	 * is greater than getMinSize().
	 **/
	CvSize getSubImageSize(); 
	
	/**
	 * \brief Get the minimum allowed subImageSize. 
	 * 
	 * If the subImageSize is too small (below getMinSize()), the smallest scale 
	 * is dropped and subImageSize is scaled up proportionally to the next scale. This process
	 * is repeated until subImageSIze is greater than getMinSize(). By default, minSize
	 * is 60x40. 
	 **/
	CvSize getMinSize(); 
	
	/**
	 * \brief Map a pixel location in the original image into a grid-cell. 
	 **/
	CvPoint gridPointForPixel(CvPoint pixel); 
	
	/**
	 * \brief Find the pixel in the original image that is in the center of a grid-cell. 
	 **/
	CvPoint pixelForGridPoint(CvPoint gridPoint); 
	
	/**
	 * \brief Find out if the foveaRepresentation preview image is being set after every
	 * fixation. This may be turned off in order to improve efficiency. 
	 **/
	int getGeneratePreview(); 
	
	/**
	 * \brief Check whether same-frame optimizations are being used. 
	 *
	 * Under certain conditions, the computation needed to search a frame a second time are 
	 * less than the computations needed to search it a first time. In these conditions, 
	 * the same-frame optimizations will automatically be used. However, this requires
	 * setting setNewImage() each time the image to search changes (i.e. a new frame). If you 
	 * are in a situation in which you know that each frame will only be fixated at one 
	 * point, you may wish to turn same-frame optimizations off.  
	 **/
	int getSameFrameOptimizations(); 
	
		
	/**
	 *\brief Change the size of the input image and the downsampled image patches.  Omitting
	 * a newSubImageSize causes the smallest-used-scale to have a 1-1 pixel mapping with the
	 * downsampled image patch -- i.e. information is not lost in the smallest scale. 
	 *
	 * @param newInputSize The size of the next image that will be searched. 
	 **/
	void changeInputImageSize(CvSize newInputSize);
	
	/**
	 * \brief Change the size of the input image and the downsampled image patches.  
	 * 
	 * @param newInputSize The size of the next image that will be searched. 
	 * 
	 * @param newSubImageSize The desired size of the downsampled image patches. 
	 * If the subImageSize is too small (below getMinSize()), the smallest scale 
	 * is dropped and subImageSize is scaled up proportionally to the next scale. This process
	 * is repeated until subImageSIze is greater than getMinSize(). By default, minSize
	 * is 60x40. 
	 **/
	void changeInputImageSize(CvSize newInputSize, CvSize newSubImageSize);
	
	/**
	 * \brief Turns on/off the code that modifies foveaRepresentation to visualize the 
	 * process of fixating. 
	 *
	 * @param flag Set to 0 if visualization is not desired (more efficient) or to 1 if
	 * visualization is desired.
	 **/
	void setGeneratePreview(int flag); 
	
	/**
	 * \brief Set the minimum allowed subImageSize. 
	 * 
	 * If the subImageSize is too small (below getMinSize()), the smallest scale 
	 * is dropped and subImageSize is scaled up proportionally to the next scale. This process
	 * is repeated until subImageSIze is greater than getMinSize(). By default, minSize
	 * is 60x40. 	 
	 * 
	 * The will have no effect on the current subImageSize, or getUsedScales() until 
	 * changeInputImageSize() is called. 
	 *
	 * @param minsize The minimum allowed subImageSize.
	 **/
	void setMinSize(CvSize minsize); 
	
	/**
	 * \brief Tell the IPP not to use the next-frame optimizations for the next frame. 
	 *
	 * Every time you are searching an image that is different from the one you searched
	 * before, this should be set (unless same-frame optimizations are turned off by the
	 * useSameFrameOptimizations() function). 
	 *
	 * When interfacing with the IPP via the MIPOMDP class, setNewImage() is always called
	 * for all search methods except searchFrameAtGridPoint(). 
	 **/
	void setNewImage(); 
	
	
	/**
	 * \brief Set whether same-frame optimizations are being used. 
	 *
	 * Under certain conditions, the computation needed to search a frame a second time are 
	 * less than the computations needed to search it a first time. In these conditions, 
	 * the same-frame optimizations will automatically be used. However, this requires
	 * setting setNewImage() each time the image to search changes (i.e. a new frame). If you 
	 * are in a situation in which you know that each frame will only be fixated at one 
	 * point, you may wish to turn same-frame optimizations off.  
	 *
	 * Generally same-frame optimizations should not be turned on unless you know that 
	 * they were turned on automatically. Turning them on when inappropriate will lead to 
	 * incorrect behavior. In general, it is appropriate to turn them on if the first scale
	 * (largest scale) in the IPP is the same size as entire visual field. 
	 *
	 * @param flag If 0, Same Frame Optimizations will not be used. If 1, Same Frame 
	 * Optimizations will be used regardless of whether or not it's appropriate. Be careful
	 * setting this to 1. 
	 **/
	void useSameFrameOptimizations(int flag); 	
	
	
	/**
	 * \brief A pointer to the oject detector used in searching. By exposing this variable, it 
	 * is easy to switch out the object detector that is applied to an IPP, or to change its 
	 * properties.
	 *
	 * For example, with an OpenCVHaarDetector, the search granularity and minimum-patch-size
	 * parameters can be changed.
	 **/
	OpenCVHaarDetector* detector; 	
	
	/**
	 * \brief Change the file used by the object detector for doing detecting.
	 * This is critical if a weights file is located at an absolute path that 
	 * may have changed from training time. 
	 * 
	 * When an ObjectDetector is loaded from disk, it will try to reload its
	 * weights file from the same source used in training. If this fails, a
	 * warning will be printed, and the detector's source will need to be set.
	 */
	void setObjectDetectorSource(std::string newFileName); 
	
	
	/**
	 * \brief Save a variety of visual representations of the process of fixating with an IPP to 
	 * image files. 
	 * 
	 * This method saves a series of .png image files, each with a prefix given
	 * by base_filename. Images with the following suffix are created: 
	 * \li FullInputImage - The full input image contained in grayFrame. 
	 * \li Scale-[0:N] - The down-sampled representation of each image patch. 
	 * \li FoveatedInputImage - A reconstruction of the full image using the donwsampled patches.
	 * \li FoveatedInputImageWithLooking - Same as above, with white boxes drawn around each scale.
	 * \li FoveatedInputImageWithGrid - Same as above, but with a grid overlayed showing the grid-cells.
	 * \li FullInputImageWithGrid - Full image with black rectangles showing grid-cell locations. 
	 * \li FullInputImageWithLooking - Same as above, but with wite boxes drawn around each scale.
	 * 
	 * Additionally, one CSV file is created, suffix "FaceCounts.csv", which records the output
	 * of the object detector on the foveated representation in each grid-cell. 
	 *
	 * @param grayFrame The image to search. This image should have size inputImageSize,
	 * and be of type IPL_DEPTH_8U with a single channel. 
	 *
	 * @param searchPoint The center of fixation.
	 *
	 * @param base_filename All of the files generated by this function will be given this as
	 * a prefix. 
	 **/
	void saveVisualization(IplImage* grayFrame, CvPoint searchPoint, const char* base_filename); 
	
	
	/**
	 * \brief Write to a file.
	 **/
	friend std::ostream& operator<< (std::ostream& ofs, ImagePatchPyramid* a);
	
	/**
	 * \brief Read from a file.
	 **/
	friend std::istream& operator>> (std::istream& ifs, ImagePatchPyramid* a);
protected: 	
	void readFromStream(std::istream& in); 
	void addToStream(std::ostream& out); 
	void reInitialize(); 
	
	int newImage; 
	CvSize inputImageSize; 
	CvSize subImageSize; 
	CvSize gridSize; 
	int repeatFirstScale; 
	int observeOff; 
	int minw ;//The minimum size of the larger subimage dimension
	int minh ;//The minimum size of the smaller subimage dimension
	int numSubImages;            //How many scales do we integrate views across? 	
	CvMat* actXtoGridCornerX;    //If a subview is centered at an X/Y location, where does its corner start? 	
	CvMat* actYtoGridCornerY;    //If a subview is centered at an X/Y location, where does its corner start? 
	CvMat* pixelXFromGridCornerX;    //[Nx2] matrix maps to [x-corner-start x-corner-end] of grid column x 
	CvMat* pixelYFromGridCornerY;    //[Mx2] matrix maps to [y-corner-start y-corner-end] of grid row y 
	CvMat* subImageGridPoints;   // [numSubImages x 2] What is the size (width / height) of each scale in terms of grid points? 
	
	IplImage* currentScaledImage; //has size of scaled down image
	IplImage* objectCountFirstScale; //has size of grid
	
	//virtual void countObjects(CvSeq* faces, CvPoint searchPoint, int scale); 
	virtual void countObjects(std::vector<cv::Rect>, CvPoint searchPoint, int scale); 
	
	CvRect getFullImageROIForPointAtScale(CvPoint searchPoint, int scale); 

};

std::ostream& operator<< (std::ostream& ofs, ImagePatchPyramid* a);
std::istream& operator>> (std::istream& ifs, ImagePatchPyramid* a);

#endif
