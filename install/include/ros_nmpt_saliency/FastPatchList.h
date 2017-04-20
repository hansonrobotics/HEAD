/*
 *  FastPatchList.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/25/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef FASTPATCHLIST_H
#define FASTPATCHLIST_H

#include <opencv2/core/core_c.h>
#include "ImagePatch.h"
#include "PatchList.h"

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
class FastPatchList : public PatchList{
public: 

	
	/**
	 * \brief Constructor.
	 *
	 * @param minSize Don't search for objects smaller than minSize. If minSize
	 * is (0,0) then baseObjectSize is used. 
	 * @param maxSize Don't search for objects larger than maxSize. If maxSize
	 * is (0,0), then the upper bound on the object size is the image size.
	 * @param scaleInc Relative size of the object in the next scale, compared
	 * to the current one. scaleInc must be greater than 1. 
	 * @param stepWidth Search interval between subsequent patches. A stepWidth
	 * of 1 means slide the object evaluation window over one pixel. 2 means 
	 * only evaluate patches at every other pixel. 1.5 means search two and 
	 * skip the third, etc.
	 * @param scaleStepWidth A boolean (0 or non-zero) value, indicating whether
	 * the stepWidth should scale up with the patch size. Setting 1 gives the
	 * same relative coverage to all scales, setting to 0 gives relatively 
	 * finer coverage to larger scales, and searching will take considerably
	 * more time.
	 */
	FastPatchList(cv::Size minSize = cv::Size(0,0), cv::Size maxSize=cv::Size(0,0), 
				  double scaleInc=1.2, double stepWidth=1, 
				  cv::Size basePatchSize=cv::Size(30,30), int scaleStepWidth=1); 
protected: 
	int scalestepwidth; 
	virtual int setImageAllScalesNeedsPointerReset(IplImage* newImage) ;
	virtual void resetPointers();
	//void setNullPointers(); 

};

#endif
