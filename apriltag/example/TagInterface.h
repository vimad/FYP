#ifndef _TAGINTERFACE_H
#define _TAGINTERFACE_H

#include <iostream>
#include <string>
#include <cstdio>
#include <limits>

#include <unistd.h>
#include <termios.h>
#include <poll.h>

#include "opencv2/opencv.hpp"

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/getopt.h"
#include "common/homography.h"

#include "Pipe.h"

class TagInterface
{
public:
	TagInterface();
	~TagInterface();

	void start(int argc, char *argv[]);

	void parseOptions(int argc, char *argv[]); //parsing parameters
	void initDetector();
	void process();

private:

	const char *famname;
	int isVisualFeedOn;
	int com;
	int tag_id;
	int record;
  double previousHeight;
  int state;
  int tagState;
  int fx,fy,cx,cy;

	bool initialized;
	struct termios initial_settings;

	getopt_t *getopt;
	apriltag_family_t *tf;
	apriltag_detector_t *td;
	cv::VideoCapture cap;

	Pipe pipe;

	void drawTags(apriltag_detection_t *det, cv::Mat &frame);
	pose getPosition(apriltag_detection_t *det);

	bool initialize();
	bool iskeypressed(unsigned timeout_ms);
	bool linebuffered(bool on);
	bool echo(bool on);
};


#endif