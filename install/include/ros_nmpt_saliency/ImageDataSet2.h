/*
 *  ImageDataset2.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/25/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#ifndef _IMAGEDATASET2_H
#define _IMAGEDATASET2_H

#include <opencv2/core/core.hpp>
#include <string>
#include <vector>


/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A data structure for maintaining a list of 
 * image files and image labels to facilitate training and testing. 
 *
 * Each ImageDataSet entry consists of a filename and an array of doubles that
 * constitute some information particular to each image. While this class is
 * agnostic to the semantic content of the labels, other classes assume that the
 * 0 and 1 elements of this array contain the (x, y) coordinates of the center
 * of the target object. 
 *
 * An image data set can be constructed programmatically by adding one element
 * at a time, or it can be created from two files. The first file should have a
 * list of file names, one per line. The second file should have the same number
 * of lines, and on each line should be the labels for that image, separated by
 * spaces. All images should have the same number of labels. The data type
 * of each label is assumed to be double. 
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */
class ImageDataSet2 {
public:
	/**
	 *\brief Constructor. Create a data set automatically by parsing a file containing 
	 * list of files and a file containing a list of labels. 
	 *
	 *@param imageListFile A file containing a list of file names, one per line. 
	 * The files should either be absolute file paths, or should be relative to 
	 * the directory from which the program is being run. 
	 *
	 *@param imageLabelsFile A file containing the same number of lines as 
	 * imageListFile, and on each line should be the labels for that image, 
	 * separated by spaces. All images should have the same number of labels. The 
	 * data type of each label is assumed to be double. 
	 **/
	ImageDataSet2(const std::string &imageListFile, 
				  const std::string &imageLabelsFile=""); 
		
	/**
	 * \brief Default Constructor.
	 **/
	ImageDataSet2();
	
	/**
	 * \brief Copy Constructor.
	 **/
	ImageDataSet2(const ImageDataSet2 &copy) ; 
	
	/**
	 * \brief Assignment operator
	 **/
	ImageDataSet2 & operator=(const ImageDataSet2 &rhs); 
	
	/**
	 * \brief Destructor
	 */
	virtual ~ImageDataSet2(); 
	
	
	/**
	 * \brief Split the dataset into two sets: Removes a continuous block of elements from this 
	 * dataset and adds them to a new one. The total number of elements in both sets is the same 
	 * as the original number of elements in this set. 
	 *
	 * @param splitStart Index of the first element to remove, inclusive. After the split, the
	 * element previously at this index in in this list will be located at index 0 in the new 
	 * list. 
	 *
	 * @param splitEnd Index of the last element to remove, incluseive. After the split, the 
	 * element located at the index splitEnd+1 will be located at index splitStart in this list.
	 **/
	ImageDataSet2 split(int splitStart, int splitEnd); 
	
	/**
	 * \brief Split the dataset into two sets: Removes specified elements from this 
	 * dataset and adds them to a new one. The total number of elements in both sets is the same 
	 * as the original number of elements in this set. 
	 *
	 * @param removeToNewList An array of the indexes of elements that you want put into a 
	 * new list. For example, to remove every-other element, this would be 0, 2, 4, 6, ...
	 * After removal, the remaining elements will maintain their same ordering, but the list
	 * will be compacted. So in the above example, element 1 will move to index 0, element 3 
	 * will move to index 1, element 5 will move to index 2, etc. 
	 *
	 * @param numToRemove The length of the supplied list. 
	 **/
	ImageDataSet2 split(const std::vector<int> &removeToNewList, int numToRemove); 
	
	/**
	 * \brief Combine the elements from another dataset into this dataset. This method is 
	 * non-destructive, so the second dataset still has all of its elements (i.e. the number
	 * of elements across both datasets is not conserved). 
	 * 
	 * @param setToAdd After merging, all of setToAdd's records will be added to those in
	 * the current object
	 **/
	void merge(const ImageDataSet2 &setToAdd); 
	
	/**
	 * \brief Access the name of the file located at a certain index. 
	 * 
	 * @param fileNumber Index of the record to access. Must be between 0 and getNumEntries()-1, inclusive.
	 * 
	 * @return A pointer to a string containing the file name. 
	 **/
	std::string getFileName(int fileNumber) const; 
	
	/**
	 * \brief Access the name of the labels located at a certain index. 
	 * 
	 * @param fileNumber Index of the record to access. Must be between 0 and getNumEntries()-1, inclusive.
	 * 
	 * @return A pointer to a string containing the array of labels. This array has length numLabelsPerImage(). 
	 **/
	std::vector<double> getFileLabels(int fileNumber) const; 
	
	/**
	 * \brief Number of labels describing each image. 
	 **/
	int numLabelsPerImage() const; 
	
	/**
	 * \brief Number of records currently in the dataset.
	 **/
	int getNumEntries() const; 
	
	/**
	 * \brief Programmatically add a single record to the dataset. 
	 *
	 * @param filename The location (absolute or relative) of an image file that will be described by labels. 
	 *
	 * @param labels An array of labels describing the image of length numLabelsPerImage(); 
	 **/
	void addEntry(const std::string &filename, const std::vector<double> &labels = std::vector<double>(0)) ; 
	
	/**
	 * \brief Check whether any elements have been added.
	 **/ 
	int empty() const ; 
	
	/**
	 * \brief Write to a file.
	 */
	friend cv::FileStorage& operator << (cv::FileStorage &fs, const ImageDataSet2 &data); 
	
	/**
	 * \brief Read from a file.
	 */
	friend void operator >> ( const cv::FileNode &fs, ImageDataSet2 &data) ; 
	
private:
	std::vector<std::string> filenames; 
	std::vector<std::vector<double> > labels; 
}; 

cv::FileStorage& operator << (cv::FileStorage &fs, const ImageDataSet2 &data); 
void operator >> ( const cv::FileNode &fs, ImageDataSet2 &data) ; 

#endif
