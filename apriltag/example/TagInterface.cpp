#include "TagInterface.h"

using namespace std;
using namespace cv;

double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

//******************************************************************************
TagInterface::TagInterface()
{
    initialized = false;

}

//******************************************************************************
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

//******************************************************************************
void TagInterface::start(int argc, char *argv[])
{
	parseOptions(argc,argv);
    initDetector();
    process();
}

//******************************************************************************
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

    getopt_add_int(getopt, 'i', "target-id", "1000", "Set the id of the target landing pad");
    getopt_add_int(getopt, 'v', "visual", "0", "visual feed");
    getopt_add_int(getopt, 'c', "com", "0", "communication");
    getopt_add_int(getopt, 'r', "record", "0", "record-video");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) 
    {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }	
}

//******************************************************************************
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
    
    isVisualFeedOn = getopt_get_int(getopt, "visual");
    com = getopt_get_int(getopt, "com");
    tag_id = getopt_get_int(getopt, "target-id");
    record = getopt_get_int(getopt, "record");

    // Initialize camera
    cap = VideoCapture(0);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        exit(1);
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    //inizialize the pipe
    pipe.init();

}

//******************************************************************************
void TagInterface::process()
{
    //Wait for the start signal
    if(com)
        cout<<"Waiting for message to start"<<endl;
    
    string val = "";
    while(strcmp(val.c_str(),"start") !=0 && com == 1)
    {
        char* rec = pipe.ReceiveMessage();
        string str(rec);
        val = str.substr(0,5);
        cout<<"message got "<<val<< endl;
    } 
	
    //Start Vision system
    Mat frame, gray;
    int frames = 0;
    double last_t = tic();

    VideoWriter video;

    if(record){
        ostringstream strs;
        strs<<"./video/vid-"<<((int)last_t)%1000000<<".avi";
        string strvid = strs.str();
        video = VideoWriter(strvid,CV_FOURCC('M','J','P','G'),10,Size(640,480));
        initialize();
	    linebuffered( false );
        echo( false );
    }

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

        if(zarray_size(detections) == 0)
        {
            if(com)
            {
              pose p;
              p.id = 1;
              p.timestamp = tic();
              p.x = 0; p.y = 0; p.z = 0;
              pipe.SendMessage(p);
            }
        }


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
            if(com)
              pipe.SendMessage(position);

            cout << "x = "<< position.x << " y = "<<position.y<<" z = "<<position.z<<endl;
            
            if(isVisualFeedOn || record)
			    drawTags(det,frame);
        }

        zarray_destroy(detections);

        if(isVisualFeedOn)
        {
            imshow("Tag Detections", frame);
	        char key = waitKey(1);
            if(key == 'p') break;
            /*if (waitKey(1) >= 0)
                break;*/
        }

        if(record){
            video.write(frame);
            if(iskeypressed(0)){
                linebuffered(true);
	            echo(true);
                video.release();
                break;
            }
        }
	
    }

}

//******************************************************************************
pose TagInterface::getPosition(apriltag_detection_t *det)
{
    matd_t *M = homography_to_pose(det->H, 509, 509, 321, 242);
    double scale = 0.2; //tag size / 2
    MATD_EL(M, 0, 3) *= scale;
    MATD_EL(M, 1, 3) *= scale;
    MATD_EL(M, 2, 3) *= scale;

    pose position;

    position.id = 2;
    position.timestamp = tic();

    position.x = MATD_EL(M, 0, 3);
    position.y = MATD_EL(M, 1, 3);
    position.z = MATD_EL(M, 2, 3) * (-1);

    return position;
}

//******************************************************************************
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

//******************************************************************************
//**************************Key press event handling****************************

bool TagInterface::initialize(){
    if (!initialized)
	{
	initialized = (bool)isatty( STDIN_FILENO );
	if (initialized)
	  initialized = (0 == tcgetattr( STDIN_FILENO, &initial_settings ));
	if (initialized)
	  std::cin.sync_with_stdio();
	}
	return initialized;
}

bool TagInterface::iskeypressed(unsigned timeout_ms = 0){
    if (!initialized) {
		cout<<"not wanted"<<endl;
		return false;
	}

	struct pollfd pls[ 1 ];
	pls[ 0 ].fd     = STDIN_FILENO;
	pls[ 0 ].events = POLLIN | POLLPRI;
	return poll( pls, 1, timeout_ms ) > 0;
}

bool TagInterface::linebuffered(bool on = true){
    struct termios settings;

    if (!initialized) return false;

    if (tcgetattr( STDIN_FILENO, &settings )) return false;

    if (on) settings.c_lflag |= ICANON;
    else    settings.c_lflag &= ~(ICANON);

    if (tcsetattr( STDIN_FILENO, TCSANOW, &settings )) return false;

    if (on) setlinebuf( stdin );
    else    setbuf( stdin, NULL );

    return true;
}

bool TagInterface::echo(bool on = true){
    struct termios settings;

    if (!initialized) return false;

    if (tcgetattr( STDIN_FILENO, &settings )) return false;

    if (on) settings.c_lflag |= ECHO;
    else    settings.c_lflag &= ~(ECHO);

    return 0 == tcsetattr( STDIN_FILENO, TCSANOW, &settings );
}