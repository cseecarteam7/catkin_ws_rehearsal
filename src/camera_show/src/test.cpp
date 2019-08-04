#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char**argv)
{
	VideoCapture cap("compete.mp4");
//	VideoCapture cap("/home/cseecar/catkin_ws/video/clockwise.mp4");
	if(!cap.isOpened()){
		std::cout<<"video source error!"<< std::endl;
		return -1;
	}


	Mat frame;
	int key, fr_no=0;
	bool capture = true;
	for(;;){
		cap >> frame;
		if (frame.empty())
			break;

		if ((key = waitKey(30)) == 27) {
			break;
		}

		fr_no++;
		resize(frame,frame,Size(320,240));
		Mat roi (frame, Rect(0,120, 320, 120));
		Mat edge;
		Canny(roi, edge, 60, 120, 3);
		imshow("edge", edge);
		imshow("ROI", roi);
		imshow("test",frame);
	}

	return 0;
}


