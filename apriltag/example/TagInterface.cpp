#include "TagInterface.h"

using namespace std;
using namespace cv;

double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}


TagInterface::TagInterface()
{

}

TagInterface::~TagInterface()
{
	apriltag_detector_destroy(td);
    /*if (!strcmp(famname, "tag36h11"))
        tag36h11_destroy(tf);
    else if (!strcmp(famname, "tag36h10"))
        tag36h10_destroy(tf);
    else if (!strcmp(famname, "tag36artoolkit"))
        tag36artoolkit_destroy(tf);
    else if (!strcmp(famname, "tag25h9"))
        tag25h9_destroy(tf);
    else if (!strcmp(famname, "tag25h7"))
        tag25h7_destroy(tf);*/
	getopt_destroy(getopt);
}

void TagInterface::parseOptions(int argc, char *argv[])
{
	getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    getopt_add_int(getopt, 'i', "target-id", "12", "Set the id of the target landing pad");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) 
    {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }	
}

void TagInterface::initDetector()
{
	apriltag_family_t *tf = NULL;
    famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11"))
        tf = tag36h11_create();
    else if (!strcmp(famname, "tag36h10"))
        tf = tag36h10_create();
    else if (!strcmp(famname, "tag36artoolkit"))
        tf = tag36artoolkit_create();
    else if (!strcmp(famname, "tag25h9"))
        tf = tag25h9_create();
    else if (!strcmp(famname, "tag25h7"))
        tf = tag25h7_create();
    else 
    {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }
    tf->black_border = getopt_get_int(getopt, "border");

    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    // Initialize camera
    cap = VideoCapture(0);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        exit(1);
    }

}

void TagInterface::process()
{
	
    Mat frame, gray;
    int frames = 0;
    double last_t = tic();
    while (true) {
        //cap >> frame;

        for(int i=0;i<5;i++)
        {
            cap >> frame;
        }

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            			  .height = gray.rows,
            			  .stride = gray.cols,
            			  .buf = gray.data
                        };

        zarray_t *detections = apriltag_detector_detect(td, &im);
        cout << zarray_size(detections) << " tags detected" << endl;

        // print out the frame rate at which image frames are being processed
        frames++;
        if (frames % 10 == 0) {
          double t = tic();
          cout << "  " << 10./(t-last_t) << " fps" << endl;
          last_t = t;
        }

        
        for (int i = 0; i < zarray_size(detections); i++)
        {
        	apriltag_detection_t *det;
			zarray_get(detections, i, &det);

            pose position;
            position = getPosition(det);

            cout << "x = "<< position.x << " y = "<<position.y<<" z = "<<position.z<<endl;

			drawTags(det,frame);
        }

        zarray_destroy(detections);

        imshow("Tag Detections", frame);
        char key = waitKey(1);
        if(key == 'p') break;
        /*if (waitKey(1) >= 0)
            break;*/
    }

}

pose TagInterface::getPosition(apriltag_detection_t *det)
{
    matd_t *M = homography_to_pose(det->H, 600, 600, 320, 240);
    double scale = 0.05 / 2.0; //0.05 is the tag size
    MATD_EL(M, 0, 3) *= scale;
    MATD_EL(M, 1, 3) *= scale;
    MATD_EL(M, 2, 3) *= scale;

    pose position;

    position.x = MATD_EL(M, 0, 3);
    position.y = MATD_EL(M, 1, 3);
    position.z = MATD_EL(M, 2, 3);

    return position;
}

void TagInterface::drawTags(apriltag_detection_t *det, cv::Mat &frame)
{
	line(frame, Point(det->p[0][0], det->p[0][1]),
	         Point(det->p[1][0], det->p[1][1]),
	         Scalar(0, 0xff, 0), 2);
	line(frame, Point(det->p[0][0], det->p[0][1]),
	         Point(det->p[3][0], det->p[3][1]),
	         Scalar(0, 0, 0xff), 2);
	line(frame, Point(det->p[1][0], det->p[1][1]),
	         Point(det->p[2][0], det->p[2][1]),
	         Scalar(0xff, 0, 0), 2);
	line(frame, Point(det->p[2][0], det->p[2][1]),
	         Point(det->p[3][0], det->p[3][1]),
	         Scalar(0xff, 0, 0), 2);

	stringstream ss;
	ss << det->id;
	String text = ss.str();
	int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontscale = 1.0;
	int baseline;
	Size textsize = getTextSize(text, fontface, fontscale, 2,
	                                &baseline);
	putText(frame, text, Point(det->c[0]-textsize.width/2,
	                           det->c[1]+textsize.height/2),
	        fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
	
}