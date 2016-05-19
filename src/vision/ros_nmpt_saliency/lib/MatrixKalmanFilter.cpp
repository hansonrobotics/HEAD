/*
 *  MatrixKalmanFilter.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 6/1/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "MatrixKalmanFilter.h"

using namespace cv; 
using namespace std; 

MatrixKalmanFilter& MatrixKalmanFilter::operator=(const MatrixKalmanFilter& rhs){
	copyFrom(rhs); 
	return *this; 
}

MatrixKalmanFilter::MatrixKalmanFilter(const MatrixKalmanFilter &copy){
	copyFrom(copy); 
}


MatrixKalmanFilter::MatrixKalmanFilter(){	
	setNumStatesAndObs(1, 1);
}

MatrixKalmanFilter::MatrixKalmanFilter(size_t numStates, size_t numObs){	
	setNumStatesAndObs(numStates, numObs);
}

MatrixKalmanFilter::~MatrixKalmanFilter() {
}


void MatrixKalmanFilter::copyFrom(const MatrixKalmanFilter &copy) {
	
	R = copy.R.clone(); 
	Q = copy.Q.clone(); 
	mu = copy.mu.clone(); 
	Sigma = copy.Sigma.clone();   
	
	matNx1a = copy.matNx1a.clone(); 
	matNx1b = copy.matNx1b.clone(); 
	matMNxMNa = copy.matMNxMNa.clone(); 
	matMNxMNb = copy.matMNxMNb.clone(); 
	matMNxNa = copy.matMNxNa.clone(); 
	matMNx1a = copy.matMNx1a.clone(); 
	matNxNa = copy.matNxNa.clone(); 
	mat1x1a = copy.mat1x1a.clone(); 
}


void MatrixKalmanFilter::setNumStatesAndObs(size_t numStates, size_t numObs) {
	//if (_IMM_DEBUG) cout << "setNumStatesAndObs: Clearning memory. " << endl; 
	//clearMatMemory(); 
	
	
	if (_IMM_DEBUG) cout << "setNumStatesAndObs: Creating memory. " << endl; 
	matNx1a.create(numObs, 1, CV_64F); 
	matNx1b.create(numObs, 1, CV_64F); 
	mat1x1a.create(1, 1, CV_64F); 
	matMNxNa.create(numStates, numObs,  CV_64F); 
	matMNxMNa.create(numStates, numStates,  CV_64F); 
	matMNxMNb.create(numStates, numStates,  CV_64F); 
	matMNx1a.create(numStates, 1,  CV_64F); 
	matNxNa.create(numObs, numObs,  CV_64F); 
	mu.create(numStates, 1, CV_64F);  
	Sigma.create(numStates, numStates, CV_64F);  
	R.create(numStates, numStates, CV_64F);  
	Q.create(numObs, numObs, CV_64F);   // motion reliability parameter, n x n	
	
	
	if (_IMM_DEBUG) cout << "setNumStatesAndObs: setting default priors. " << endl; 
	setMuPrior();
	setR();
	setQ();
	setSigmaPrior(); 
	
}

void MatrixKalmanFilter::setMuPrior(double vectorVal) {
	mu.setTo(vectorVal); 
}

void MatrixKalmanFilter::setMuPrior(const Mat &muPrior) {
    Mat newMu = muPrior.reshape(1); 
    muPrior.copyTo(mu);
}


void MatrixKalmanFilter::setSigmaPrior(const Mat &SigmaPrior) {
    SigmaPrior.copyTo(Sigma);
}

void MatrixKalmanFilter::setSigmaPrior(double diagval) {
	Sigma = Mat::eye(Sigma.rows, Sigma.cols, Sigma.type()) * diagval;
}

void MatrixKalmanFilter::setQ(const Mat &QVal){
    QVal.copyTo(Q);
}

void MatrixKalmanFilter::setQDiag(const Mat &QVal){
    int numRows = QVal.rows < Q.rows? QVal.rows:Q.rows; 
    
	Q.setTo(0); 
    for (int i = 0; i < numRows; i++) {
        Q.at<double>(i, i) = QVal.at<double>(i,0); 
    }
}

void MatrixKalmanFilter::setQ(double diagval){	
	Q = Mat::eye(Q.rows, Q.cols, Q.type()) * diagval;
}


void MatrixKalmanFilter::setR(const Mat &RVal){
    RVal.copyTo(R);
}


void MatrixKalmanFilter::setR(double diagval){
	R = Mat::eye(R.rows, R.cols, R.type()) * diagval;
}

likelihood MatrixKalmanFilter::modelLogLikelihood(const Mat &x,const Mat &obs) {
    Mat cMat;
    getCMatFromFeatures(x, cMat); 
	likelihood retval; 
	//getMotionFeatures(cMat, features); 
	//getCMatFromFeatures(features, cMat); 
	
	//matNx1a = c*alpha
	matNx1a = cMat*mu; 
	//cvGEMM(cMat, mu, 1, NULL, 1, matNx1a, 0); 
	
	//matNx1a = tau-c*alpha
	matNx1a = obs - matNx1a; 
	//cvSub(obs, matNx1a, matNx1a); 
	
	//matMNxNa = S*Ct
	matMNxNa = Sigma*cMat.t(); 
	//cvGEMM(Sigma, cMat, 1, NULL, 1, matMNxNa, CV_GEMM_B_T); 
	
	//matNxNa = (C*S*Ct+Q)^-1
	matNxNa = cMat*matMNxNa+ Q; 
	matNxNa	= matNxNa.inv(); 
	//cvGEMM(cMat, matMNxNa, 1, Q, 1, matNxNa, 0); 
	//cvInvert(matNxNa, matNxNa);
	
	//matNxNb = (C*S*Ct+Q)^-1 * (tau-c*alpha)
	matNx1b = matNxNa*matNx1a; 
	//cvGEMM(matNxNa, matNx1a, 1, NULL, 1, matNx1b, 0); 
	retval.grad = -matNx1b; 
	
	//mat1x1a = (tau-c*alpha)T * (C*S*Ct+Q)^-1 * (tau-c*alpha)
	mat1x1a = matNx1a.t()*matNx1b; 
	//cvGEMM(matNx1a, matNx1b, 1, NULL, 1, mat1x1a, CV_GEMM_A_T); 
	
	retval.val =  -0.5 * mat1x1a.at<double>(0,0);//cvGetReal2D(mat1x1a, 0, 0); 
	
	return retval; 
}

double MatrixKalmanFilter::obsLogLikelihood(const Mat &x, const Mat &obs) {
	return modelLogLikelihood(x, obs).val; 
}


void MatrixKalmanFilter::getObsMean(const Mat &x, Mat &obs) {
	if (_IMM_DEBUG) cout << "Getting observation mean for MatrixKalmanFilter" << endl; 
    Mat cMat;
    getCMatFromFeatures(x, cMat); 
	obs =  cMat*mu; 
	//cvGEMM(cMat, mu, 1, NULL, 1, obs, 0); 
}

void MatrixKalmanFilter::updateModel(const Mat &x, const Mat &obs) {
    Mat cMat;
    getCMatFromFeatures(x, cMat); 
	
	//getMotionFeatures(cMat, features); 
	//getCMatFromFeatures(features, cMat); 
	
	if (_IMM_DEBUG) cout << "matNx1a = c*alpha" << endl; 
	matNx1a = cMat*mu; 
	//cvGEMM(cMat, mu, 1, NULL, 1, matNx1a, 0); 
	
	if (_IMM_DEBUG) cout << "matNx1a = tau-c*alpha" << endl; 
	matNx1a = obs - matNx1a; 
	//cvSub(obs, matNx1a, matNx1a); 
	
	if (_IMM_DEBUG) cout << "Sigma = Sigma+R" << endl; 
	Sigma += R; 
	//cvAdd(Sigma, R, Sigma); 
	
	if (_IMM_DEBUG) cout << "matMNxNa = S*Ct" << endl; 
	matMNxNa = Sigma*cMat.t(); 
	//cvGEMM(Sigma, cMat, 1, NULL, 1, matMNxNa, CV_GEMM_B_T); 
	
	if (_IMM_DEBUG) cout << "matNxNa = (C*S*Ct+Q)^-1" << endl; 
	matNxNa = cMat*matMNxNa+Q; 
	matNxNa = matNxNa.inv(); 
	//cvGEMM(cMat, matMNxNa, 1, Q, 1, matNxNa, 0); 
	//cvInvert(matNxNa, matNxNa);	
	
	if (_IMM_DEBUG) cout << "matMNxNa = S*Ct * (C*S*Ct+Q)^-1 = K" << endl; 
	matMNxNa = matMNxNa * matNxNa; 
	//cvGEMM(matMNxNa, matNxNa, 1, NULL, 1, matMNxNa, 0); 
	
	if (_IMM_DEBUG) cout << "matMNx1a = K * (tau-c*alpha)" << endl; 
	matMNx1a = matMNxNa*matNx1a; 
	//cvGEMM(matMNxNa, matNx1a, 1, NULL, 1, matMNx1a, 0); 
	
	if (_IMM_DEBUG) cout << "alpha = alpha+ K * (tau-c*alpha)" << endl; 
	mu += matMNx1a; 
	//cvAdd(mu, matMNx1a, mu); 
	
	if (_IMM_DEBUG) cout << "matMNxMNa = KC" << endl; 
	matMNxMNa = matMNxNa * cMat; 
	//cvGEMM(matMNxNa, cMat, 1, NULL, 1, matMNxMNa, 0); 
	
	if (_IMM_DEBUG) cout << "matMNxMNa = I - KC" << endl; 
	matMNxMNb = Mat::eye(matMNxMNb.rows, matMNxMNb.cols, matMNxMNb.type()); 
	matMNxMNa = matMNxMNb-matMNxMNa; 
	//cvSetIdentity(matMNxMNb); 
	//cvSub(matMNxMNb, matMNxMNa, matMNxMNa); 
	
	if (_IMM_DEBUG) cout << "S = (I - KC) * S" << endl; 
	Sigma = matMNxMNa * Sigma; 
	//cvGEMM(matMNxMNa, Sigma, 1, NULL, 1, Sigma, 0); 	
	
}


void MatrixKalmanFilter::rectify() {
	Mat mask; 
	compare(mu,0,mask,CMP_LT); 
	mu.setTo(0,mask); 
	//threshold(mu, mu, 0, 0, THRESH_TOZERO); 
}

void MatrixKalmanFilter::addToStream(ostream& out) const {
	if (_IMM_DEBUG) cout << "Adding KF Object To Stream" << endl; 
	if (_IMM_DEBUG) cout << "Num states / Num obs" << endl; 
	out << matMNxNa.rows << " " << matMNxNa.cols << endl; 
	if (_IMM_DEBUG) cout << "Adding Mu To Stream" << endl; 	
	addMatrixToStream(out, mu); 
	if (_IMM_DEBUG) cout << "Adding Sigma To Stream" << endl; 	
	addMatrixToStream(out, Sigma); 
	if (_IMM_DEBUG) cout << "Adding R To Stream" << endl; 	
	addMatrixToStream(out, R); 
	if (_IMM_DEBUG) cout << "Adding Q To Stream" << endl; 	
	addMatrixToStream(out, Q); 
}

void MatrixKalmanFilter::readFromStream(istream& in) {
	if (_IMM_DEBUG) cout << "Reading KF Object from Stream" << endl; 
	int numStates,numObs; 
	
	if (_IMM_DEBUG) cout << "Reading Num States / Num Obs from Stream" << endl; 
	in >> numStates >> numObs; 
	if (_IMM_DEBUG) cout << "Setting numStates and numObs to " << numStates << ", " << numObs << endl; 
	setNumStatesAndObs(numStates, numObs); 
	
	if (_IMM_DEBUG) cout << "Clearning old mean" << endl; 
	
	//cvReleaseMat(&mu); 
	if (_IMM_DEBUG) cout << "Reading Mu from Stream" << endl; 
	mu = readMatrixFromStream(in); 
	
	//cvReleaseMat(&Sigma); 
	if (_IMM_DEBUG) cout << "Reading Sigma from Stream" << endl; 
	Sigma = readMatrixFromStream(in); 
	
	//cvReleaseMat(&R); 
	if (_IMM_DEBUG) cout << "Reading R from Stream" << endl; 
	R = readMatrixFromStream(in); 	
	
	//cvReleaseMat(&Q); 
	if (_IMM_DEBUG) cout << "Reading Q from Stream" << endl; 
	Q = readMatrixFromStream(in); 
}

void MatrixKalmanFilter::addMatrixToStream(ostream& out, const Mat& matrix) const {
	if (_IMM_DEBUG) cout << "Adding matrix to stream" << endl; 
	out.precision(20);
	out << matrix.rows << " " << matrix.cols << endl; 
	for (int i = 0; i < matrix.rows; i++) {
		for (int j = 0; j < matrix.cols; j++) {
			out << matrix.at<double>(i, j) << " " ; 
		}
		out << endl; 
	}
}

Mat MatrixKalmanFilter::readMatrixFromStream(istream &in) {
	if (_IMM_DEBUG) cout << "Reading matrix from stream" << endl; 
	
	int rows, cols; 
	in >> rows >> cols; 
	if (_IMM_DEBUG) cout << "Reading matrix of size " << rows << "x" << cols << endl; 
	
	Mat retval(rows, cols, CV_64F); 
	double val; 
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			in >> val; 
			if (_IMM_DEBUG) cout << "(" << i << ", " << j << ") = " << val << endl; 
			retval.at<double>(i, j) =  val; 
		}
	}
	return retval; 
}


ostream& operator<< (ostream& ofs, const MatrixKalmanFilter &model) {
	model.addToStream(ofs); 
	return ofs; 
}

istream& operator>> (istream& ifs, MatrixKalmanFilter &model) {
	model.readFromStream(ifs); 
	return ifs; 
}

void MatrixKalmanFilter::getCMatFromFeatures(const Mat &features, Mat &cMat){
	cMat.create(mu.rows/features.rows, mu.rows, CV_64F); 
	cMat.setTo(0); 
	for (int i = 0; i < cMat.rows; i++) {
		for (int j = 0; j < features.rows; j++) {
			cMat.at<double>( i, j+i*features.rows) = features.at<double>(j, 0);
		}
	}
}

