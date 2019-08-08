#define ROS
//#define CAMERA_SHOW
//#define CAMERA_SHOW_MORE

#ifdef ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"

#endif

#include <cv.hpp>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>

#define PI 3.1415926

using namespace std;
using namespace cv;

const int Width = 320;
const int Height = 240;
const int XHalf = (Width/2);
const int YHalf = (Height/2);
const int YPoint = 60;
const float slope_threshold = 0.5;

const Scalar Red = Scalar(0, 0, 255);
const Scalar Blue = Scalar(255, 0, 0);
const Scalar Yellow = Scalar(50, 250, 250);
const Scalar Sky = Scalar(215, 200, 60);
const Scalar Pink = Scalar(220, 110, 230);

int flag = 1;
int clock_flag = 0;
int pre_cnt = 0;
int lap_cnt = 0;

clock_t start=0, end=0;

void 
show_lines(Mat &img, vector<Vec4i> &lines, Scalar color = Scalar(0, 0, 0),int thickness=2)
{
	bool color_gen = false;

	if (color == Scalar(0, 0, 0))
		color_gen = true;

	for (int i = 0; i < lines.size(); i++){
		if (color_gen == true)
			color = Scalar(rand() % 256, rand() % 256, rand() % 256);

		line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), color, thickness);
	}

}



void  
split_left_right(vector<Vec4i> lines, vector<Vec4i>&left_lines, vector<Vec4i> &right_lines)
{
	vector<float> slopes;
	vector<Vec4i> new_lines;

	for (int i = 0; i < lines.size(); i++){
		int x1 = lines[i][0];
		int y1 = lines[i][1];
		int x2 = lines[i][2];
		int y2 = lines[i][3];

		//Calculate slope
		float slope;
		if (x2 - x1 == 0) //corner case, avoiding division by 0
			slope = 999.0; //practically infinite slope
		else
			slope = (y2 - y1) / (float)(x2 - x1);

		//Filter lines based on slope
		if (abs(slope) > slope_threshold) {
			slopes.push_back(slope);
			new_lines.push_back(lines[i]);
		}
	}

	for (int i = 0; i < new_lines.size(); i++){
		Vec4i line = new_lines[i];
		float slope = slopes[i];

		int x1 = line[0];
		int y1 = line[1];
		int x2 = line[2];
		int y2 = line[3];

		if (slope > 0 && x1 > XHalf && x2 > XHalf)//
			right_lines.push_back(line);
		else if (slope < 0 && x1 < XHalf && x2 < XHalf)//slope < 0 && x1 < cx && x2 < cx
			left_lines.push_back(line);
	}
}

	bool 
find_line_params(vector<Vec4i> &left_lines, float *left_m, float *left_b)
{

	float  left_avg_x = 0, left_avg_y = 0, left_avg_slope = 0;

	if(left_lines.size() == 0)
		return false;

	for (int i = 0; i < left_lines.size(); i++)//calculate right avg of x and y and slope
	{
		//line(roi, Point(left_lines[i][0],left_lines[i][1]), Point(left_lines[i][2],left_lines[i][3]), color, 3);
		left_avg_x += left_lines[i][0];
		left_avg_x += left_lines[i][2];
		left_avg_y += left_lines[i][1];
		left_avg_y += left_lines[i][3];
		left_avg_slope += (left_lines[i][3] - left_lines[i][1]) / (float)(left_lines[i][2] - left_lines[i][0]);
	}
	left_avg_x = left_avg_x / (left_lines.size() * 2);
	left_avg_y = left_avg_y / (left_lines.size() * 2);
	left_avg_slope = left_avg_slope / left_lines.size();

	// return values
	*left_m = left_avg_slope;

	//b=y-mx //find Y intercet
	*left_b = left_avg_y - left_avg_slope * left_avg_x;

	return true;
}

void find_start_lines(Mat &img, vector<Vec4i> &lines)
{
	int find_flag = 0;

	for (int i = 0; i < lines.size(); i++)
	{
		int x1 = lines[i][0];
		int y1 = lines[i][1];
		int x2 = lines[i][2];
		int y2 = lines[i][3];

		float slope;

		//Calculate slope
		if (x2 - x1 == 0) //corner case, avoiding division by 0
			slope = 999.0; //practically infinite slope
		else
			slope = (y2 - y1) / (float)(x2 - x1);

		//Filter lines based on slope
		if (abs(slope) < 0.065) {
			find_flag = 1;
			putText(img, format("Find START LINE: %d\n", lap_cnt), Point(50,50), FONT_HERSHEY_SIMPLEX,0.5,Yellow,2);
			printf("lap_cnt: %d\n", lap_cnt);

			if(flag){
				printf("@@@@@@@@@@@@@@@@@@ lap_cnt increase!! @@@@@@@@@@@@@@@@@@@@");
				lap_cnt++;
			}
			flag = 0;
			break;   
		}
	}

	if(!find_flag) pre_cnt++;
	printf("pre_cnt: %d\n", pre_cnt);
	
	if(pre_cnt > 2000){
		pre_cnt = 0;
		flag = 1;
	}
	if(lap_cnt == 2 && clock_flag == 0) {
		start = clock();
		clock_flag = 1;
		printf("on clock start(start: %ld)\n", start);
	}
}



	void 
find_lines(Mat &img, vector<cv::Vec4i> &left_lines, vector<Vec4i>& right_lines, float *rdistance, float *ldistance)
{
	static float left_slope_mem = 1, right_slope_mem = 1, right_b_mem = 0, left_b_mem = 0;

	float left_b, right_b, left_m, right_m;

	bool draw_left = find_line_params(left_lines, &left_m, &left_b);
	if (draw_left) {
		float left_x0 = (-left_b) / left_m;
		float left_x120 = (YHalf - left_b) / left_m;
		left_slope_mem = left_m;
		left_b_mem = left_b;
#ifdef CAMERA_SHOW
		line(img, Point(left_x0, 0), Point(left_x120, YHalf), Blue, 3);
		//out << left_lines.size() << " left lines,";
#endif 
	}
	else {
		cout << "\tNo Left Line,";
	}

	bool draw_right = find_line_params(right_lines, &right_m, &right_b);
	if (draw_right) {
		float right_x0 = (-right_b) / right_m;
		float right_x120 = (YHalf - right_b) / right_m;
		right_slope_mem = right_m;
		right_b_mem = right_b;
#ifdef CAMERA_SHOW
		line(img, Point(right_x0, 0), Point(right_x120, YHalf), Red, 3);
#endif
		//cout << right_lines.size() << " right lines" <<endl;
	}
	else {
		cout << "\tNo RIght Line" << endl;
	}
	// y = mx + b ==> x0 = (y0-b)/m
	float left_xPt = ((YPoint - left_b_mem) / left_slope_mem);
	float right_xPt = ((YPoint - right_b_mem) / right_slope_mem);
	*ldistance = XHalf - left_xPt;
	*rdistance = right_xPt - XHalf;
}

int img_process(Mat &frame)
{
	Mat grayframe, edge_frame, roi_gray_ch3;
	Mat roi;
	Mat start_roi;
	cvtColor(frame, grayframe, COLOR_BGR2GRAY);
	Rect rect_roi(0,YHalf,Width,YHalf);
	//Rect start_rect(0, 70, Width, YHalf - 70);
	Rect start_rect(135, 80, 50, YHalf - 80);

	roi = frame(rect_roi);

	cvtColor(roi,grayframe,COLOR_BGR2GRAY) ;
	GaussianBlur(grayframe,grayframe,Size(3,3),1.5);
	cvtColor(grayframe, roi_gray_ch3, COLOR_GRAY2BGR);
	Canny(grayframe,edge_frame,70,150,3); //min_val, max val , filter size

	vector<cv::Vec4i> lines_set;
        cv::HoughLinesP(edge_frame,lines_set,1,PI/180,30,30,10);
	
	start_roi = edge_frame(start_rect);
	vector<cv::Vec4i> start_lines;
	HoughLinesP(start_roi, start_lines, 1, PI/180, 30, 30, 10);
	find_start_lines(start_roi, start_lines);

	// FINISH start_line detection

#ifdef CAMERA_SHOW_MORE
	show_lines(roi_gray_ch3, lines_set);
#endif
	vector<Vec4i> right_lines;
	vector<Vec4i> left_lines;
	split_left_right(lines_set, left_lines, right_lines);
#ifdef CAMERA_SHOW
	show_lines(roi, left_lines, Sky, 2);
	show_lines(roi, right_lines, Pink, 2);
#endif
	float ldistance, rdistance;
	find_lines(roi, left_lines, right_lines, &rdistance, &ldistance);

	int differ=rdistance-ldistance;
#ifdef CAMERA_SHOW
	circle (roi, Point(XHalf, YPoint), 5, Scalar(250,250,250),-1);
	circle(roi, Point(XHalf + differ, YPoint), 5, Scalar(0, 0, 255), 2);
	putText(roi,format("%3d - %3d = %3d",(int)rdistance, (int)ldistance, differ),Point(XHalf-100,YHalf/2),FONT_HERSHEY_SIMPLEX,0.5,Yellow,2);
	//imshow("roi",roi);
	//imshow("edgeframe",edge_frame);
#endif
#ifdef CAMERA_SHOW_MORE
	//imshow("frame", frame);
	//imshow("roi_gray_ch3", roi_gray_ch3);
#endif
	//imshow("start_roi", start_roi);
	return differ;
}


int 
main(int argc, char**argv)
{
	VideoCapture cap(0);
	//VideoCapture cap("/home/cseecar/catkin_ws/video/clockwise.mp4");
	Mat frame;

	if(!cap.isOpened()){
		std::cout<<"no camera!"<< std::endl;
		return -1;
	}

#ifdef ROS
	ros::init(argc, argv, "cam_msg_publisher");
	ros::NodeHandle nh;
	std_msgs::String cam_msg;
	ros::Publisher pub = nh.advertise<std_msgs::String>("cam_msg",100);

	int init_past=1;
	//--------------------------------------------------
	ros::Rate loop_rate(50);
	cout<<"start"<<endl;
#endif

	int differ, key, fr_no = 0;
	bool capture = true;
	
	for(;;){
		if (capture) {
			cap >> frame;
			if (frame.empty())
				break;
		}

		if ((key = waitKey(30)) >= 0) {
			if (key == 27)
				break;
			else if (key = ' ') {
				capture = !capture;
			}
		}

		if (capture == false)
			continue;

		fr_no++;
		resize(frame,frame,Size(Width,Height));
		differ = img_process(frame);

#ifdef ROS
		stringstream result_message;

		if(start == 0)
		result_message << differ << "/" << "65";
		
		else {
			start = clock() - start;
			//printf("@@@@@@@@@@@@@@@@@@@@@@@@@@end : %Lf @@@@@@@@ minus : %Lf\n sec : %0.2f\n", end, end-start, (float)((end-start)/CLOCKS_PER_SEC));
			
			if(((float)start/CLOCKS_PER_SEC) >= 2.5)
				result_message << "00" << "/" << "00";
		}

		cam_msg.data = result_message.str(); 
		pub.publish(cam_msg);
		loop_rate.sleep();

		std::cout << fr_no << ":" << differ << endl;
		if((float)start/CLOCKS_PER_SEC >= 2.5) break;
#endif
	}

	std::cout<<"Camera off"<<endl;
	return 0;
}

