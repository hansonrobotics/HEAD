/*
 *  FastPatchList2.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/24/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */


#ifndef FASTPATCHLIST2_H
#define FASTPATCHLIST2_H

#include <opencv2/core/core.hpp>
#include "PatchList2.h"

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A class that does the necessary book keeping
 * for filtering an image quickly at many scales. FastPatchList is an
 * implementation that sacrifices filtering accuracy for filtering speed. 
 * 
 * It computes only a single integral image, and filters attempt to scale 
 * themselves up, sometimes imperfectly, to filter at different sizes. This 
 * contratsts with PatchList, which maintains accuracy at the expense of 
 * computing many integral images. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class FastPatchList2 : public PatchList2{
public: 
	/**
	 * \brief Constructor.
	 *
	 * @param basePatchSize The size of the patches that will be evaluated 
	 * for containing the object. This should match the size of whatever
	 * filter will be applied to the patches. It may be set later, using
	 * setBasePatchSize(). At subsequent scales, the image is scaled up or 
	 * down relative to this size.
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
	 * @param dontCopyImage Set to non-zero if you want efficiency, but not to look at patches.
	 */
	FastPatchList2(cv::Size basePatchSize, cv::Size minSize=cv::Size(0,0), 
			   cv::Size maxSize=cv::Size(0,0), double scaleInc=1.2, 
			   double stepWidth=1, int dontCopyImage=0, int scaleStepWidth=1); 
	
	/**
	 * \brief Default Constructor.
	 **/
	FastPatchList2();
	
	/**
	 * \brief Copy Constructor.
	 **/
	FastPatchList2(const FastPatchList2 &copy) ; 
	
	/**
	 * \brief Assignment operator
	 **/
	FastPatchList2 & operator=(const FastPatchList2 &rhs); 
	
	/**
	 * \brief Destructor
	 */
	virtual ~FastPatchList2(); 
	
	
	
protected: 
	int scalestepwidth; 
	virtual int setImageAllScalesNeedsPointerReset(const cv::Mat &newImage) ;
	virtual void resetPointers();
	
};

#endif
