/*
 *  NMPTUtils.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/16/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "NMPTUtils.h"
#include "DebugGlobals.h"
#include <iostream>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
//#include <zlib.h>
//using namespace std; // clashes with OpenCV here
using namespace cv; 

#define FIX(x) (x<0?ceil(x):floor(x))

int NMPTUtils::any(CvMat* mat) {
	for (int i = 0; i < mat->height; i++) {
		for (int j = 0; j < mat->width; j++) {
			if (cvGetReal2D(mat, i, j) != 0)
				return 1; 
		}
	}
	return 0; 
}

void NMPTUtils::fix(CvMat* mat) {
	for (int i = 0; i < mat->height; i++) {
		for (int j = 0; j < mat->width; j++) {
			cvSetReal2D(mat, i, j, FIX(cvGetReal2D(mat, i, j))); 
		}
	}
}

void NMPTUtils::printMat(const Mat &mat) {
	Mat dmat; 
	mat.convertTo(dmat, CV_64F); 
    std::cout << "[" ;
	for (int i = 0; i < dmat.rows; i++) {
        for (int j = 0; j < dmat.cols; j++) {
            std::cout << "  " << dmat.at<double>( i, j);
		}
		if (i < dmat.rows-1) {
            std::cout << std::endl << "  ";
		} else {
		}
	}
    std::cout << "  ]" << std::endl;
}

void NMPTUtils::printMat(const CvMat &mat) {
	
    std::cout << "[" ;
	for (int i = 0; i < mat.rows; i++) {
		for (int j = 0; j < mat.cols; j++) {
            std::cout << "  " << cvGetReal2D(&mat,i,j) ;
		}
		if (i < mat.rows-1) {
            std::cout << std::endl << "  ";
		} else {
		}
	}
    std::cout << "  ]" << std::endl;
}

double NMPTUtils::randomNormal() {
	return theRNG().gaussian(1); //values.at<double>(0,0); 
}

double NMPTUtils::randomFloat() {
	return theRNG().uniform(0., 1.); 
}

string NMPTUtils::commaSeparatedFlattenedMat(const Mat &mat) {
    std::stringstream ss;
	int numels = mat.rows*mat.cols; 
	int ind = 1; 
	for (int i = 0; i < mat.rows; i++) {
		for (int j = 0; j < mat.cols; j++) {
			ss << mat.at<double>(i,j); 
			if (ind++ < numels)
				ss << ", "; 
		}
	}
	return ss.str(); 
}

int NMPTUtils::getVideoCaptureFromCommandLineArgs(VideoCapture& capture, const int argc, const char** argv) {
	int retval; 
	if (argc < 2) {
        std::cout << "Getting Default Camera " << CV_CAP_ANY << std::endl;
		capture.open(CV_CAP_ANY);		
		retval = 2; 
	} else {
		if (argv[1][0] == '-') {
            std::cout << argv[0] << ": A program for processing camera or video input." << std::endl;
            std::cout << "Usage:" << std::endl ;
            std::cout << "\t" << argv[0] << "\t\t\t: Get input from the default camera." << std::endl;
            std::cout << "\t" << argv[0] << "\t[number]\t\t: Get input from camera identified by number (must be less than 10)." << std::endl;
            std::cout << "\t" << argv[0] << "\t[filename]\t: Get input from specified video file." << std::endl;
            std::cout << "\t" << argv[0] << "\t--help\t\t: Print this message and quit." << std::endl;
			return 0; 
		}else if (strlen(argv[1]) == 1) {
            std::cout << "Getting Camera " << atoi(argv[1]) << std::endl;
			capture.open(atoi(argv[1]));
			retval = 2; 
		} else {
            std::cout << "Getting Movie " << argv[1] << std::endl;
			capture.open(argv[1]); 
			retval = 1; 
		}
	}
	
    if (! capture.isOpened()) {
        std::cout << argv[0] << ": Failed to get input from camera or movie file." << std::endl;
		return 0; 
	}
	return retval; 
}

double NMPTUtils::notfinite(double a) {
    return !std::isfinite(a);
}

void NMPTUtils::distNorm(Mat &normDist, const Mat &unnormDist) {
	unnormDist.convertTo(normDist, CV_64F); 
	if (!checkRange(normDist, 1)) {
		map<double>(normDist, &notfinite); 
	}
	double norm = sum(normDist)[0]; 
	normDist = normDist*(1/norm); 	
}

void NMPTUtils::sample(Mat &samples, const Mat& histogram, Size sampleSize) {
	int numels = sampleSize.width * sampleSize.height; 
	Mat normHist ; 
	distNorm(normHist, histogram.reshape(1,1)); 
	
	//normHist = normHist.reshape(1,1); 
	Mat randEls = Mat::zeros(1,numels, CV_64F); 
	samples.create(sampleSize,CV_64F); 	
	MatIterator_<double> it = samples.begin<double>();
	
	randu(randEls, 0., 1.); 
	
	int useFast = numels > 10; 
	if (useFast) {
		sort(randEls, randEls, CV_SORT_EVERY_ROW | CV_SORT_ASCENDING);
		int h = 0; 
		double c = normHist.at<double>(0,h); 
		for (int i = 0; i < numels; i++) {
			while (randEls.at<double>(0,i) > c) {
				h++; 
				c += normHist.at<double>(0,h); 
			}
			*it = h; 
			it++; 
		}
		printMat(samples); 
	} else {
		for (int i = 0; i < numels; i++) {
			double target = randomFloat(); 
			int h = 0; 
			double c = normHist.at<double>(0,h); 
			while(target > c) {
				h++; 
				c += normHist.at<double>(0,h); 
			}
			*it = h; 
			it++; 
		}
	}
	randShuffle(samples); 
}

int NMPTUtils::sample(const Mat& histogram) {
	Mat normHist ; 
	distNorm(normHist, histogram.reshape(1,1)); 
	double target = randomFloat(); 
	int h = 0; 
	double c = normHist.at<double>(0,h); 
	while(target > c) {
		h++; 
		c += normHist.at<double>(0,h); 
	}
	return h; 
}

double NMPTUtils::nchoosek(int n, int k) {
	double lognchoosek = lgamma(n+1)-lgamma(k+1)-lgamma(n-k+1); 
	return (double) round(exp(lognchoosek)); 
}


void NMPTUtils::unIntegrate(const cv::Mat& src, cv::Mat &dest, int type) {
	int w = src.cols-1; 
	int h = src.rows-1; 
	Mat temp = src(Rect(1,1,w,h)) - src(Rect(1, 0, w, h)) + src(Rect(0,0,w,h)) - src(Rect(0,1, w, h)) ; 
	temp.convertTo(dest, type); 
}

void NMPTUtils::unSqIntegrate(const cv::Mat& src, cv::Mat &dest, int type) {
	int w = src.cols-1; 
	int h = src.rows-1; 
	Mat temp = src(Rect(1,1,w,h)) - src(Rect(1, 0, w, h)) + src(Rect(0,0,w,h)) - src(Rect(0,1, w, h)) ; 
	sqrt(temp, temp); 
	temp.convertTo(dest, type); 
}


double NMPTUtils::rectAreaOverlapRatio(Rect r1, Rect r2) {
	double A1 = r1.width*r1.height; 
	double A2 = r2.width*r2.height; 
	double A1A2 = rectAreaIntersect(r1, r2); 
	return A1A2/(A1+A2-A1A2); 
}

double NMPTUtils::rectAreaIntersect(Rect r1, Rect r2) {
	double r1left = r1.x; 
	double r1right = r1.x+r1.width; 
	double r1top = r1.y; 
	double r1bottom = r1.y+r1.height; 
	
	double r2left = r2.x; 
	double r2right = r2.x+r2.width; 
	double r2top = r2.y; 
	double r2bottom = r2.y+r2.height;
	
	double r3left = (r1left>r2left)?r1left:r2left; 
	double r3top = (r1top>r2top)?r1top:r2top;
	double r3right = (r1right<r2right)?r1right:r2right; 
	double r3bottom = (r1bottom<r2bottom)?r1bottom:r2bottom; 
	
	double r3width=r3right-r3left; 
	double r3height=r3bottom-r3top; 
	if (r3width<0||r3height<0) return 0; 
	return r3width*r3height; 	
}

void NMPTUtils::writeMatBinary(FileStorage &fs, const string &name, const
							   Mat &m) {
	fs << name << "{"<<"rows"<<m.rows<<"cols"<<m.cols<< "type" << m.type(); 
	if (m.rows > 0 && m.cols > 0) {
		Mat mat(m.rows,m.cols,m.type());
		m.copyTo(mat);
		string dstring;
		binaryToAscii(dstring, mat.data, mat.rows*mat.step);
		vector<string> vdstring;
		splitString(vdstring, dstring, 64);
		fs << "data" << "[" ;//<< vdstring << "]"; 
		for(size_t i = 0; i < vdstring.size(); i++)
			fs << vdstring[i]  ;
		fs << "]" ; 
	}
	fs << "}";
}


void NMPTUtils::readMatBinary(const FileNode &tm, Mat &mat) {
	//FileNode tm = fs[name];
	int rows = (int)tm["rows"], cols = (int)tm["cols"], type = (int)tm["type"];
	mat.create(rows,cols,type);
	if (rows > 0 && cols > 0) {
		vector<string> vs;
		
		FileNode tl = tm["data"];
        //std::cout << tl.type() << std::endl;
		CV_Assert(tl.type() == FileNode::SEQ); 
		vs.resize(tl.size()); 
		for (size_t i = 0; i < tl.size(); i++) {
			tl[i] >> vs[i]; 
		}
//		CV_Assert(tl.size() == (size_t)numRegs); 
//		tm["data"] >> vs;
		string s;
		joinString(vs, s);
		asciiToBinary(s, mat.data, mat.rows*mat.step);
	}
	mat = mat.clone(); 
}


/*
 void NMPTUtils::writeMatBinaryCompressed(FileStorage &fs, const string &name, const Mat &m) {
 Mat mat(m.rows,m.cols,m.type());
 m.copyTo(mat); 
 string dstring; 
 uchar* compdata; 
 size_t compsize; 
 NMPTUtils::compress(compdata, compsize, mat.data, mat.rows*mat.step); 
 binaryToAscii(dstring, compdata, compsize); 
 free(compdata); 
 vector<string> vdstring; 
 splitString(vdstring, dstring, 1024); 
 fs << name << "{"<<"rows"<<mat.rows<<"cols"<<mat.cols<< "type" << mat.type() << "compressed_size" << (int)compsize << "data" << "[:";
 for(size_t i = 0; i < vdstring.size(); i++) 
 fs << vdstring[i]  ;
 fs << "]" << "}";
 }
 
 void NMPTUtils::readMatBinaryCompressed(FileStorage &fs, const string &name,  Mat &mat) {
 FileNode tm = fs[name]; 
 int rows = (int)tm["rows"], cols = (int)tm["cols"], type = (int)tm["type"];
 size_t compsize= (int)tm["compressed_size"]; 
 mat.create(rows,cols,type); 
 vector<string> vs; 
 tm["data"] >> vs; 
 string s; 
 joinString(vs, s); 
 uchar* compdata= (uchar*)malloc(compsize*sizeof(uchar)); 
 asciiToBinary(s, compdata, compsize); 
 
 uchar* uncompdata; 
 size_t uncompsize = mat.rows*mat.step; 
 NMPTUtils::uncompress(compdata, compsize, uncompdata,uncompsize);
 free(compdata);
 memcpy(mat.data, uncompdata,uncompsize); 
 free(uncompdata); 
 mat = mat.clone(); 
 }
 */


void NMPTUtils::binaryToAscii(std::string &dest, const uchar *data, size_t bytes) {
	const uchar ascii[] =  {
		'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M',
		'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
		'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
		'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'};  
	dest.resize(ceil(4.0*bytes/3),'0'); 
	size_t i = 0; 
	size_t j = 0; 
	for (; i < bytes-2; i+=3) {
		uchar c1 = data[i]/4;                      //[0 0 d1.256 d1.128 d1.64 d1.32 d1.16 d1.8]
		uchar c2 = (data[i]%4)*16+data[i+1]/16;    //[0 0 d1.4 d1.2 d2.256 d2.128 d2.64 d2.32]
		uchar c3 = (data[i+1]%16)*4+data[i+2]/64;  //[0 0 d2.16 d2.8 d2.4 d2.2 d3.256 d3.128]
		uchar c4 = data[i+2]%64;                   //[0 0 d3.64 d3.32 d3.16 d3.8 d3.4 d3.2]
		dest[j++] = ascii[c1];//c1+'0'; 
		dest[j++] = ascii[c2];//c2+'0'; 
		dest[j++] = ascii[c3];//c3+'0'; 
		dest[j++] = ascii[c4];//c4+'0'; 		
	}
	if (i == bytes-2) {
		uchar c1 = data[i]/4;
		uchar c2 = (data[i]%4)*16+data[i+1]/16; 
		uchar c3 = (data[i+1]%16)*4;            
		dest[j++] = ascii[c1];//c1+'0'; 
		dest[j++] = ascii[c2];//c2+'0'; 
		dest[j++] = ascii[c3];//c3+'0'; 
	} else {
		uchar c1 = data[i]/4;
		uchar c2 = (data[i]%4)*16;         
		dest[j++] = ascii[c1];//c1+'0'; 
		dest[j++] = ascii[c2];//c2+'0'; 
	}
}

void NMPTUtils::asciiToBinary(const std::string &data, uchar *dest, size_t bytes) {
	size_t i = 0; 
	size_t j = 0; 
	const uchar ascii[] =  {
		'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M',
		'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
		'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
		'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'};  
	uchar decode[128]; 
	for (size_t i = 0; i < 64; i++) {
		decode[(size_t)ascii[i]] = i; 
	}
	for (; i < bytes-2; i+=3) {
		uchar c1 = decode[(size_t)data[j++]];//data[j++] -'0';
		uchar c2 = decode[(size_t)data[j++]];//data[j++] -'0';
		uchar c3 = decode[(size_t)data[j++]];//data[j++] -'0'; 
		uchar c4 = decode[(size_t)data[j++]];//data[j++] -'0';	
		dest[i]  = c1*4+(c2/16);      //[c1.64 c1.32 c1.16 c1.8 c1.4 c1.2 c2.64 c2.32] 
		dest[i+1] = (c2%16)*16 + c3/4;//[c2.16 c2.8 c2.4 c2.2 c3.64 c3.32 c3.16 c3.8]
		dest[i+2] = (c3%4)*64+c4;     //[c3.4 c3.2 c4.64 c4.32 c4.16 c4.8 c4.4 c4.2]
	}
	if (i == bytes-2) {
		uchar c1 = decode[(size_t)data[j++]];//data[j++] -'0';
		uchar c2 = decode[(size_t)data[j++]];//data[j++] -'0';
		dest[i]  = c1*4+(c2/16);      //[c1.64 c1.32 c1.16 c1.8 c1.4 c1.2 c2.64 c2.32] 
		dest[i+1] = (c2%16)*16 ;      //[c2.16 c2.8 c2.4 c2.2 0 0 0 0]
	} else {
		uchar c1 = decode[(size_t)data[j++]];//data[j++] -'0';
		dest[i]  = c1*4;              //[c1.64 c1.32 c1.16 c1.8 c1.4 c1.2 0 0] 
	}
}

void NMPTUtils::binaryToHex(std::string &dest, const uchar *data, size_t bytes) {
	dest.resize(bytes*2);
	for (size_t i = 0; i < bytes; i++) {
		uchar c = data[i]; 
		dest[2*i] = (c/16)+'0'; 
		dest[2*i+1] = (c%16)+'0'; 
	}
}

void NMPTUtils::hexToBinary(const std::string &data, uchar *dest, size_t bytes) {
	for (size_t i = 0; i < bytes; i++) {
		uchar c1 = data[2*i]-'0'; 
		uchar c2 = data[2*i+1]-'0'; 
		dest[i] = c1*16+c2;
	}
}

void NMPTUtils::splitString(std::vector<std::string> &dest, const std::string &data, size_t maxlen) {
	double destlen = data.size(); 
	destlen = ceil(destlen/maxlen); 
	dest.resize(destlen); 
	int i = 0; 
	for (size_t start = 0; start < data.size(); start += maxlen) {
		size_t end = start+maxlen; 
		end = end<data.size()?end:data.size(); 
		end = end-start; 
		dest[i] = data.substr(start,end); 
		i++; 
	}
}

void NMPTUtils::joinString(const std::vector<std::string> &data, std::string &dest) {
	dest.clear(); 
	for (size_t i = 0; i < data.size(); i++) {
		dest.append(data[i]); 
	}
}

/*
 int NMPTUtils::compress(uchar* &dest, size_t &dest_size, const uchar* src, size_t src_size) {
 z_stream c_stream; // compression stream 
 int err;
 
 c_stream.zalloc = (alloc_func)0;
 c_stream.zfree = (free_func)0;
 c_stream.opaque = (voidpf)0;
 
 err = deflateInit(&c_stream, Z_BEST_SPEED);
 //deflateParams(&c_stream, Z_BEST_COMPRESSION, Z_FILTERED);
 
 if (err) {
 std::cout << "Error " << err << " in deflateInit." << std::endl;
 return 0; 
 }
 
 size_t initsize = deflateBound(&c_stream, src_size); 
 dest = (uchar*)calloc(initsize, sizeof(uchar)); 
 dest_size = initsize; 
 
 
 c_stream.next_in = (Bytef*)src;
 c_stream.avail_in = src_size;
 for (;;) {
 c_stream.next_out = dest;
 c_stream.avail_out = dest_size;
 err = deflate(&c_stream, Z_FINISH);
 if (err == Z_STREAM_END)
 break; 
 if (err) {
 std::cout << "Error " << err << " in deflate." << std::endl;
 return err; 
 }
 }
 
 if (err != Z_STREAM_END) {
 std::cout << "Error in " << err << " deflateEnd." << std::endl;
 return err; 
 }
 dest_size = c_stream.total_out; 
 return 0; 
 }
 
 int NMPTUtils::uncompress(const uchar* src, size_t src_size, uchar* &dest, size_t dest_size) {
 int err;
 z_stream d_stream; // decompression stream 
 size_t initsize = dest_size; 
 dest = (uchar*)calloc(initsize, sizeof(uchar)); 
 d_stream.zalloc = (alloc_func)0;
 d_stream.zfree = (free_func)0;
 d_stream.opaque = (voidpf)0;
 
 d_stream.next_in  = (Bytef*)src;
 d_stream.avail_in = src_size;
 
 err = inflateInit(&d_stream);
 if(err) {
 std::cout << "Error " << err << " in inflateInit";
 return err; 
 }
 
 for (;;) {
 d_stream.next_out = dest;            // discard the output 
 d_stream.avail_out = dest_size;
 err = inflate(&d_stream, Z_NO_FLUSH);
 if (err == Z_STREAM_END) break;
 if (err) {
 std::cout << "Error " << err << " in inflate" << std::endl;
 return err; 
 }
 
 }
 
 err = inflateEnd(&d_stream);
 if (err) {
 std::cout << "Error " << err << " in inflate end" << std::endl;
 return err; 
 }
 if (dest_size != d_stream.total_out) return d_stream.total_out; 
 return 0; 
 }
 */

void NMPTUtils::rectangleRotated(Mat & image, Point center, Size size, double angle, const Scalar& color, 
								 int thickness, int lineType, int shift) {
	Mat points = Mat::ones(5, 3, CV_64F); 
	Mat transform = getRotationMatrix2D(Point(0,0), angle, 1);
	points.at<double>(0,0) =  - size.width/2.; 
	points.at<double>(0,1) = size.height/2.; 
	
	points.at<double>(1,0) = size.width/2.; 
	points.at<double>(1,1) = size.height/2.; 
	
	points.at<double>(2,0) = size.width/2.; 
	points.at<double>(2,1) = - size.height/2.; 
	
	points.at<double>(3,0) = - size.width/2.; 
	points.at<double>(3,1) = - size.height/2.; 
	
	points.at<double>(4,0) = size.width/2.; 
	points.at<double>(4,1) = 0 ; 
	/*
     std::cout << "points: " << std::endl;
	 printMat(points); 
     std::cout << "transform: " << std::endl;
	 printMat(transform) ; 
     std::cout << "points type: " << points.type() << "; transform type: " << transform.type() << std::endl;
	 */
	points = points*transform.t(); 
	
	for (int i = 0; i < 4; i++) {
		int next = (i+1)%4; 
		Point p1(points.at<double>(i,0)+center.x, points.at<double>(i,1)+center.y); 
		Point p2(points.at<double>(next,0)+center.x, points.at<double>(next,1)+center.y); 
		line(image, p1, p2, color, thickness, lineType, shift); 
	}
	
	Point p(points.at<double>(4,0)+center.x, points.at<double>(4,1)+center.y); 	
	line(image, center, p, color, thickness, lineType, shift); 	
}


/* input: NxM, N data points, M Dims
 * labels: NxO, N data points, O outputs
 * weights: Nx1, 1 weight per data point
 * xqueries: QxM, Q query points, M Dims
 * tau: variance of gaussian weighting window
 * eps: minimal attention paid to all points
 *
 * predictions: QxO, Q query points, O output dimensions. 
 */
Mat NMPTUtils::RBF(const Mat &input, const Mat &labels, const Mat &weights, const Mat &xqueries, double tau, double eps) {
	int N = input.rows; 
	//int M = input.cols; 
	int O = labels.cols; 
	int Q = xqueries.rows; 
	
    if (_RBF_DEBUG) std::cout << "Computing RBF on " << N << " data points, " << Q << " queries." << std::endl;
	
	Mat predictions, xdiff, wtrow;
	double negt2inv = -1.0/(2.0*tau*tau);
	wtrow.create(1,N,CV_64F); 
	predictions.create(Q,O,CV_64F); 

	
	for (int i = 0; i < Q; i++) {
		const Mat currx1 = xqueries.row(i); 
		Mat curry = predictions.row(i); 
		
		if (_RBF_DEBUG) {
            std::cout << "Computing RBF Val for query " ;
			printMat(currx1); 
		}
			
		
		for (int j = 0; j < N; j++) {
			Mat currx2 = input.row(j); 
			xdiff = currx1-currx2; 
			double sqdist = xdiff.dot(xdiff); 
			double wt =  exp(sqdist*negt2inv+eps)*weights.at<double>(j,0); 
			wtrow.at<double>(0,j) = wt; 
		}
		
		if (_RBF_DEBUG) {
            //std::cout << "Unnormalized Weights: " ;
			//printMat(wtrow); 
		}
		
		
		double norm = sum(wtrow)[0]; 
		
        //std::cout << "Norm is " << norm.val[0] << std::endl;
		if (norm > 0)  {
			wtrow *= 1.0/norm;
		}
		else {
			wtrow = 0.; 
		}
		
		curry = wtrow * labels; 
		
		
		if (_RBF_DEBUG) {
            //std::cout << "Normalized Weights: " ;
			//printMat(wtrow); 
            std::cout << "Values " ;
			printMat(curry); 
		}
		
		Mat nreal = curry.clone(); 
		map<double>(nreal, &notfinite); 
		
		if (_RBF_DEBUG) {
            std::cout << "notfinite gave: " ;
			printMat(nreal); 
		}
		
		if (any<double>(nreal)) {
            std::cout << "!!!!Warning! RBF Value was ill conditioned." << std::endl;
			
			curry.setTo(0., (nreal != 0)); 
			
			}
		curry.setTo(-1., curry < -1); 
		curry.setTo(1., curry > 1); 
		
		if (_RBF_DEBUG) {
            std::cout << "Prediction " ;
			printMat(predictions.row(i)); 
		}
		
	}
	
	return predictions; 
}


