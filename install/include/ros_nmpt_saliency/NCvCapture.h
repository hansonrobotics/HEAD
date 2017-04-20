/*
 *  NCvCapture.h
 *
 *  Created by Nicholas Butko on 11/3/09.
 *  Copyright 2009. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution. 
 * 3. The name of the author may not be used to endorse or promote products 
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __APPLE__

#define NCvCapture CvCapture
#define NCvVideoWriter CvVideoWriter
#define ncvCaptureFromFile(a) cvCaptureFromFile(a)
#define ncvCreateFileCapture(a) cvCreateFileCapture(a)
#define ncvCaptureFromCAM() cvCaptureFromCAM()
#define ncvCreateCameraCapture() cvCreateCameraCapture()
#define ncvCaptureFromCAM(a) cvCaptureFromCAM(a)
#define ncvCreateCameraCapture(a) cvCreateCameraCapture(a)
#define ncvCreateVideoWriter(a, b, c, d) cvCreateVideoWriter(a, b, c, d)
#define ncvCreateVideoWriter(a, b, c, d, e) cvCreateVideoWriter(a, b, c, d, e)

#else

#include <opencv2/core/core_c.h>

#define NCV_DISABLE_AUTO_RESTART 999

/**
 *\class NCvCapture
 *\ingroup NCvGroup
 *\brief <tt>Auxilliary Tool:</tt> A workaround for 64-bit OpenCV Capture on OSX.
 *
 * NCvCapture uses the QTKit APIs, which have the following advantages over the
 * current quicktime based libraries used in OpenCV:
 * \li They can be compiled in 32- or 64-bit mode.
 * \li The height/width of webcam images can be set, drastically increasing
 * the rate of video capture.
 * \li Multiple processes can access the same camera at once. 
 * \li Programmatically specify which attached camera you use, instead of 
 * having the system decide arbitrarily.
 *
 * These libraries are designed so that as little code changes are needed as 
 * possible:
 * \li Include NMPT/NCvCapture.h
 * \li Rather than creating a CvCapture struct, create an NCvCapture object.
 * \li Use cvCaptureFromCAM, etc. as you normally would.
 * \li When compiling using g++, link to the following frameworks: QTKit,
 * \li Foundation, AppKit, QuartzCore.
 * 
 * A few limitations of these libraries are listed below. Patches are welcome; 
 * send patches to nbutko at ucsd dot edu:
 * \li Video capture in OSX 10.5 is much slower than video capture in OSX 10.6.
 * \li Some USB webcams become unstable when the resolution is changed, 
 * especially by multiple processes trying to access the same camera at once.
 *
 * Integration on non-OSX systems: If the compiler does not report an apple
 * operating system, the NCvCapture routines will fall back to their CvCapture
 * equivalents, using defines and macros: 
 *
 * #ifndef __APPLE__
 * \li #define NCvCapture CvCapture
 * \li #define NCvVideoWriter CvVideoWriter
 * \li #define ncvCaptureFromFile(a) cvCaptureFromFile(a)
 * \li #define ncvCreateFileCapture(a) cvCreateFileCapture(a)
 * \li #define ncvCaptureFromCAM() cvCaptureFromCAM()
 * \li #define ncvCreateCameraCapture() cvCreateCameraCapture()
 * \li #define ncvCaptureFromCAM(a) cvCaptureFromCAM(a)
 * \li #define ncvCreateCameraCapture(a) cvCreateCameraCapture(a)
 * \li #define ncvCreateVideoWriter(a, b, c, d) cvCreateVideoWriter(a, b, c, d)
 * \li #define ncvCreateVideoWriter(a, b, c, d, e) cvCreateVideoWriter(a, b, c, d, e)
 * 
 * #else ...
 * #endif
 * 
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */

class NCvCapture; 

/**
 *\class NCvVideoWriter
 * \ingroup NCvGroup
 *\brief <tt>Auxilliary Tool:</tt> A workaround for 64-bit OpenCV Capture on OSX.
 *
 * NCvCapture uses the QTKit APIs, which have the following advantages over the
 * current quicktime based libraries used in OpenCV:
 * \li They can be compiled in 32- or 64-bit mode.
 * \li The height/width of webcam images can be set, drastically increasing
 * the rate of video capture.
 * \li Multiple processes can access the same camera at once. 
 * \li Programmatically specify which attached camera you use, instead of 
 * having the system decide arbitrarily.
 *
 * These libraries are designed so that as little code changes are needed as 
 * possible:
 * \li Include NMPT/NCvCapture.h
 * \li Rather than creating a CvCapture struct, create an NCvCapture object.
 * \li Use cvCaptureFromCAM, etc. as you normally would.
 * \li When compiling using g++, link to the following frameworks: QTKit,
 * \li Foundation, AppKit, QuartzCore.
 * 
 * A few limitations of these libraries are listed below. Patches are welcome; 
 * send patches to nbutko at ucsd dot edu:
 * \li Video capture in OSX 10.5 is much slower than video capture in OSX 10.6.
 * \li Some USB webcams become unstable when the resolution is changed, 
 * especially by multiple processes trying to access the same camera at once.
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */
class NCvVideoWriter; 


/**
 * \fn NCvCapture* ncvCaptureFromFile(const char* filename)
 * \ingroup NCvGroup
 * \brief Get a capture object from a video file source. This can be used to
 * extract OpenCV's IplImages from video, using ncvGrabFrame.
 * 
 * @param filename Relative or absolute file path to the video file. 
 */
NCvCapture* ncvCaptureFromFile(const char* filename);

/**
 * \fn NCvCapture* ncvCreateFileCapture(const char* filename)
 * \ingroup NCvGroup
 * \brief Get a capture object from a video file source. This can be used to
 * extract OpenCV's IplImages from video, using ncvGrabFrame.
 * 
 * @param filename Relative or absolute file path to the video file. 
 */
NCvCapture* ncvCreateFileCapture(const char* filename);

/**
 * \fn NCvCapture* ncvCaptureFromCAM(int index=-1) 
 * \ingroup NCvGroup
 * \brief Get a capture object from a camera source. This can be used to
 * extract OpenCV's IplImages from a live video feed, using ncvGrabFrame.
 * 
 * @param index The number of the camera that you want to query, indexed from 0.
 * If a number higher than the number of attached cameras is specified, a 
 * warning is printed, and a modulus operation is used to determine which camera
 * is opened. 
 */
NCvCapture* ncvCaptureFromCAM(int index=-1); 

/**
 * \fn NCvCapture* ncvCreateCameraCapture(int index = -1)
 * \ingroup NCvGroup
 * \brief Get a capture object from a camera source. This can be used to
 * extract OpenCV's IplImages from a live video feed, using ncvGrabFrame.
 * 
 * @param index The number of the camera that you want to query, indexed from 0.
 * If a number higher than the number of attached cameras is specified, a 
 * warning is printed, and a modulus operation is used to determine which camera
 * is opened. 
 */
NCvCapture* ncvCreateCameraCapture(int index = -1); 

/**
 * \fn NCvVideoWriter* ncvCreateVideoWriter(const char* filename, int fourcc, double fps, CvSize frame_size, int is_color=1)
 * \ingroup NCvGroup
 * \brief Construct a video file for writing IplImages to a video sequence.
 *
 * @param filename Relative or absolute file path to the output video file.
 * @param fourcc The four character code used for specifying a compression 
 * scheme, e.g. use CV_FOURCC('a','v','c','1') for h.264 compression.
 * @param fps Output video frames per second.
 * @param frame_size Width and Height of the video. 
 * @param is_color Is the video in color (as opposed to black and white)?
 */
NCvVideoWriter* ncvCreateVideoWriter(const char* filename, int fourcc, 
									 double fps, CvSize frame_size, 
									 int is_color=1); 


/**
 * \fn void cvReleaseCapture(NCvCapture** capture)
 * \ingroup NCvGroup
 * \brief Free memory associated with an NCvCapture object.
 *
 * @param capture Address of the capture object pointer you want to free.
 */
void cvReleaseCapture(NCvCapture** capture); 

/**
 * \fn void cvReleaseVideoWriter(NCvVideoWriter** writer)
 * \ingroup NCvGroup
 * \brief Free memory associated with an NCvVideoWriter object.
 *
 * @param writer Address of the video writer object pointer you want to free.
 */
void cvReleaseVideoWriter(NCvVideoWriter** writer); 


/**
 * \fn int cvWriteFrame(NCvVideoWriter* writer, const IplImage* image)
 * \ingroup NCvGroup
 * \brief Add an image frame to the end of a video file.
 *
 * @param writer The video writer to add the frame to.
 * @param image The frame to add.
 */
int cvWriteFrame(NCvVideoWriter* writer, const IplImage* image); 

/**
 * \fn int cvGrabFrame(NCvCapture* capture)
 * \ingroup NCvGroup
 * \brief Grab a frame now from the capture ource if possible, copy it to a memory
 * buffer to be retrieved later. Unless you have a reason to prefer this method,
 * use cvQueryFrame instead.
 *
 * @param capture A video or camera capture source.
 * @return 0 if a frame could not be retrieved, non-zero otherwise.
 */
int cvGrabFrame(NCvCapture* capture); 

/**
 * \fn IplImage* cvRetrieveFrame(NCvCapture* capture)
 * \ingroup NCvGroup
 * \brief Retrieve the last grabbed frame from the memory buffer. Unless you have a reason to prefer this method,
 * use cvQueryFrame instead.
 *
 * @param capture A video or camera capture source.
 */
IplImage* cvRetrieveFrame(NCvCapture* capture); 

/**
 * \fn IplImage* cvQueryFrame(NCvCapture* capture)
 * \ingroup NCvGroup
 * \brief Wait for a frame to become available for grabbing, and return it. This
 * is the default way to get the next frame from the capture object.
 *
 * @param capture A video or camera capture source.
 */
IplImage* cvQueryFrame(NCvCapture* capture); 

/**
 * \fn double cvGetCaptureProperty(NCvCapture* capture, int property_id)
 * \ingroup NCvGroup
 * \brief Query properties of a capture source. 
 *
 * For camera capture sources, the following can be queried: 
 * \li CV_CAP_PROP_FRAME_WIDTH: The width of the camera frame being captured. 
 * This may be inaccurately reported before video capture has begun.
 * \li CV_CAP_PROP_FRAME_HEIGHT: The height of the camera frame being captured. 
 * This may be inaccurately reported before video capture has begun.
 *
 * For video file capture sources, the following can be queried:
 * \li CV_CAP_PROP_POS_MSEC: The current position in the video, in milliseconds.
 * \li CV_CAP_PROP_POS_FRAMES: The current position in the video, in number of frames.
 * \li CV_CAP_PROP_POS_AVI_RATIO: The current position in the video, from 0
 * (beginning) to 1 (end). 
 * \li CV_CAP_PROP_FRAME_WIDTH: The width of the camera frame being captured. 
 * This may be inaccurately reported before video capture has begun.
 * \li CV_CAP_PROP_FRAME_HEIGHT: The height of the camera frame being captured. 
 * This may be inaccurately reported before video capture has begun.
 * \li CV_CAP_PROP_FPS: The frame rate of the video. Since videos may have 
 * variable frame rates, this is measured by length of time between the OS reported 
 * time stamp of the previous frame, and the current frame.
 *
 * @param capture A video or camera capture source.
 * @param property_id One of the above properties that is appropriate for the s
 * source. If an unrecognized or unsupported property is queried, 0 is returned.
 */
double cvGetCaptureProperty(NCvCapture* capture, int property_id); 

/**
 * \fn int cvSetCaptureProperty(NCvCapture* capture, int property_id, double value)
 * \ingroup NCvGroup
 * \brief Set properties of a capture source. 
 *
 * For camera capture sources, the following can be set: 
 * \li CV_CAP_PROP_FRAME_WIDTH: The width of the camera frame being captured. 
 * This may be inaccurately reported before video capture has begun.
 * \li CV_CAP_PROP_FRAME_HEIGHT: The height of the camera frame being captured. 
 * This may be inaccurately reported before video capture has begun.
 *
 * Note: NCvCapture will not request from the operating system to change the size
 * of the video stream until both a new width and a new height have been
 * specified. 
 *
 * For video file capture sources, the following can be set:
 * \li CV_CAP_PROP_POS_MSEC: The current position in the video, in milliseconds.
 * \li CV_CAP_PROP_POS_FRAMES: The current position in the video, in number of frames.
 * \li CV_CAP_PROP_POS_AVI_RATIO: The current position in the video, from 0
 * (beginning) to 1 (end). 
 *
 * @param capture A video or camera capture source.
 * @param property_id One of the above properties that is appropriate for the s
 * source. If an unrecognized or unsupported property is queried, 0 is returned,
 * otherwise 1. 
 * @param value The desired value to set.
 */
int cvSetCaptureProperty(NCvCapture* capture, int property_id, double value); 

#endif
