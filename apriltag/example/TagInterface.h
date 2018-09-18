#ifndef _TAGINTERFACE_H
#define _TAGINTERFACE_H

#include <iostream>

#include "opencv2/opencv.hpp"

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/getopt.h"
#include "common/homography.h"


struct pose
{
	double x;
	double y;
	double z;
};

class TagInterface
{
public:
	TagInterface();
	~TagInterface();

	void parseOptions(int argc, char *argv[]); //parsing parameters
	void initDetector();
	void process();

private:

	const char *famname;

	getopt_t *getopt;
	apriltag_family_t *tf;
	apriltag_detector_t *td;
	cv::VideoCapture cap;

	void drawTags(apriltag_detection_t *det, cv::Mat &frame);
	pose getPosition(apriltag_detection_t *det);
};


#endif