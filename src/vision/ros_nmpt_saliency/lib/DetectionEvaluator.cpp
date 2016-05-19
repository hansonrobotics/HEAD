/*
 *  DetectionEvaluator.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 8/20/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "DetectionEvaluator.h"
#include <iostream>
#include <sstream>
#include "NMPTUtils.h"

using namespace std;
using namespace NMPTUtils; 
using namespace cv; 

double DetectionEvaluator::acceptArea = .5; 

bool operator<(const EvaluationMetrics& a, const EvaluationMetrics& b) {
	return a.threshold < b.threshold; 
}

DetectionEvaluator::DetectionEvaluator(ImageDataSet* imageDataSet) {
	init(imageDataSet); 
}

void DetectionEvaluator::init(ImageDataSet* labeledImages) {
	fileNames.clear(); 
	targets.clear(); 
	for (int i = 0; i < labeledImages->getNumEntries(); i++) {
		string s = labeledImages->getFileName(i); 
		vector<double> labels = labeledImages->getFileLabels(i); 
		Rect box = cvRect(labels[0]-labels[2]/2, labels[1]-labels[2]/2,
							labels[2],labels[2]); 
		int found = 0; 
		for (unsigned int j = 0; j < fileNames.size(); j++) {
			if (fileNames[j] == s) {
				if (box.width > 0 && box.height > 0)
					targets[j].push_back(box); 
				found = 1; 
				break; 
			}
		}
		if (!found) {
			fileNames.push_back(s); 
			vector<Rect> r; 
			r.clear(); 
			if (box.width > 0 && box.height > 0)
				r.push_back(box);
			targets.push_back(r); 
		}
	}
}

DetectionEvaluator::DetectionEvaluator(const string &fileListFile, const string &labelFile) {
	ImageDataSet* labeledImages=(ImageDataSet*) NULL; 
	if (fileListFile.empty() || labelFile.empty()) {
		cout << "Warining: To evaluate detection, both an images file and a labels file must be provided." << endl; 
		return; 
	}
	labeledImages = ImageDataSet::loadFromFile(fileListFile, labelFile); 
	init(labeledImages); 
	delete(labeledImages); 
}


DetectionEvaluator::~DetectionEvaluator() {
}





vector<string> DetectionEvaluator::getFileNames() {
	return fileNames; 
}
vector<vector<Rect> > DetectionEvaluator::getTargetLocations() {
	return targets; 
}

vector<EvaluationMetrics> DetectionEvaluator::evaluatePerformance(const vector<vector<SearchResult> >& imBoxes) {
	vector<EvaluationMetrics> retval; 
	retval.clear(); 
	vector<vector<EvaluationMetrics> > ms; //ms is the list of threshold/performance for each image
	ms.clear(); 
	for (unsigned int i = 0; i < imBoxes.size(); i++) {
		ms.push_back(evaluateImagePerformance(i, imBoxes[i])); 
		/*
		cout << "Evaluated performance of Image " << i << endl; 
		for (unsigned int j = 0; j < ms[i].size(); j++) {
			cout << "For threshold " << ms[i][j].threshold << " there were " 
			<< (1-ms[i][j].hit_ratio)*100 << "% misses (" << ms[i][j].num_hits 
			<< " hits & " << ms[i][j].num_misses << " misses) and " << ms[i][j].num_false_alarms
			<< " false alarms." << endl; 
		}
		
		cout << "=========================================" << endl; 
		 */
	}
	//cout << "Evaluated thresholds for " << ms.size() << " images." << endl; 
	for (unsigned int i = 0; i < ms.size(); i++) { //for each image
		//cout << "In image " << i << " there are " << ms[i].size() << " thresholds." << endl; 
		for (unsigned int j = 0; j < ms[i].size(); j++) { //for each threshold
			EvaluationMetrics m; 
			m.threshold = ms[i][j].threshold; 
			m.num_hits = 0; 
			m.num_misses = 0; 
			m.num_false_alarms = 0; 
			m.hit_ratio = 0; 
			m.hits.clear(); 
			m.misses.clear();
			m.false_alarms.clear(); 
			//get average performance over all images at that threshold
			for (unsigned int k = 0; k < ms.size();  k++) {
				EvaluationMetrics n = getPerformanceAtThreshold(ms[k], m.threshold); 
				/*cout << "For image " << k << " with threshold " << m.threshold
				<< " there were " << n.num_hits << " hits, " << n.num_misses 
				<< " misses, " << n.num_false_alarms << " false alarms, and hr " 
				<< n.hit_ratio << endl; */
				m.num_hits = m.num_hits+n.num_hits; 
				m.num_misses = m.num_misses+n.num_misses; 
				m.num_false_alarms=m.num_false_alarms+n.num_false_alarms; 
				m.hit_ratio = m.hit_ratio+n.hit_ratio; 
			}
			m.num_hits = m.num_hits/ms.size(); 
			m.num_misses = m.num_misses/ms.size(); 
			m.num_false_alarms = m.num_false_alarms/ms.size(); 
			m.hit_ratio=m.hit_ratio/ms.size(); 
			retval.push_back(m); 
		}
	}
	//cout << "In all, " << retval.size() << "thresholds were evaluated." << endl; 
	sort(retval.begin(), retval.end()); 
	return retval; 
}

EvaluationMetrics DetectionEvaluator::getPerformanceAtThreshold(const vector<EvaluationMetrics>& ms, 
																		double threshold) {
	//ms is list of performance/threshold on a given image, and we want to know
	//what the performance would be by setting an arbitrary threshold. This is
	//equivalent to setting the threshold to something inbetween two thresholds.
	//If the threshold is higher than any in the list, all patches are rejected
	//-there are no false alarms, no hits, and everything is missed.
	//If the threshold is lower than any in the list, all patches are accepted,
	//which is equivalent to setting the threshold to the lowest value in the 
	//list
	//If there threshold is inbetween, then it is equivalent to finding the 
	//in-between place, and taking the performance of the higher.
	EvaluationMetrics retval; 
	retval.threshold=threshold; 
	retval.num_hits = 0; 
	retval.num_misses = (ms[0].num_hits+ms[0].num_misses) ; //Figure this out hit_ratio = num_hits/num_total
	retval.num_false_alarms = 0; 
	retval.hit_ratio = 0; 
	retval.hits.clear();
	retval.misses.clear();
	retval.false_alarms.clear(); 
	for (int i = (int)ms.size()-1; i >= 0; i--) {
		if (ms[i].threshold < threshold) {
			//cout << " " << ms[i].threshold << "<" << threshold;  
			break;
		}
		//cout << i << "-"; 
		retval.num_hits = ms[i].num_hits; 
		retval.num_misses = ms[i].num_misses; 
		retval.num_false_alarms = ms[i].num_false_alarms; 
		retval.hit_ratio = ms[i].hit_ratio; 
	}
	//cout << endl; 
	return retval; 
}

vector<EvaluationMetrics> DetectionEvaluator::evaluateImagePerformance(int imNum, 
																	   const vector<SearchResult>& boxes) {
	vector<EvaluationMetrics> retval; 
	for (unsigned int i = 0; i < boxes.size(); i++) {
		EvaluationMetrics m = evaluateImagePerformanceWithThreshold(imNum, boxes, boxes[i].value); 
		m.hits.clear(); 
		m.misses.clear(); 
		m.false_alarms.clear(); 
		retval.push_back(m); 
	}
	sort(retval.begin(), retval.end()); 
	return retval; 
}

EvaluationMetrics DetectionEvaluator::evaluateImagePerformanceWithThreshold(int imNum, 
																			vector<SearchResult> boxes,
																			double threshold) {
	EvaluationMetrics retval; 
	retval.hits.clear(); 
	retval.misses.clear(); 
	retval.false_alarms.clear(); 
	
	sort(boxes.begin(), boxes.end()); 
	reverse(boxes.begin(), boxes.end()); 
	//boxes are sorted by threshold in descending order
	//if one box is below threshold, all remaining boxes are below threshold.
	
	for (unsigned int i = 0; i < targets[imNum].size(); i++) {
		int found = 0; 
		for (unsigned int j = 0; j < boxes.size(); j++) {
			if (boxes[j].value < threshold) break; 
			if (rectAreaOverlapRatio(targets[imNum][i], boxes[j].imageLocation) >= acceptArea) {
				retval.hits.push_back(boxes[j].imageLocation); 
				boxes.erase(boxes.begin()+j); 
				found = 1; 
				break; 
			}
		}
		if (!found) retval.misses.push_back(targets[imNum][i]); 
	}
	for (unsigned int i = 0; i < boxes.size(); i++) {
		if (boxes[i].value < threshold) break; 
		retval.false_alarms.push_back(boxes[i].imageLocation); 
	}
	
	retval.num_hits = retval.hits.size(); 
	retval.num_misses = retval.misses.size(); 
	retval.num_false_alarms = retval.false_alarms.size(); 
	retval.hit_ratio = retval.num_hits*1.0/targets[imNum].size(); 
	retval.threshold = threshold; 
	
	return retval; 
}

string DetectionEvaluator::outputOpenCVDescriptionFormatForHaarTraining() {
	
	stringstream retval; 
	
	for (unsigned int i = 0; i < fileNames.size(); i++) {
		retval << fileNames[i] << " "; 
		retval << targets[i].size() ; 
		for (unsigned int j = 0; j < targets[i].size(); j++) {
			retval << " " << (int)targets[i][j].x << " " << (int)targets[i][j].y 
			<< " " << (int)targets[i][j].width << " " << (int)targets[i][j].height; 
		}
		retval << endl; 
	}
	return retval.str(); 
}
