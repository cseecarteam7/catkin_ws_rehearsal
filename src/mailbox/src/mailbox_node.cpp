#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
//#include <iostream>
#include <geometry_msgs/Twist.h>
#include <string>

using namespace std;

geometry_msgs::Twist cmd;
int kbhit();
int getch();
int sub_value=0;
ros::Publisher pub2;
ros::Subscriber sub2;
std_msgs::Int8 msg2;
int key = 0;

int spd;
int angle;
int mid_threshold = 40;
int min_angle = 30;
int max_angle = 150;

string differSpeed; //string data, expected to subscribed ; "differ/speed"


//void msgCallback(const std_msgs::Int16::ConstPtr& given_msg) {
void msgCallback(const std_msgs::String::ConstPtr& given_msg) {

	differSpeed = (given_msg->data); //String type
	string str_differ, str_spd;

	for(int i=0; i<differSpeed.size(); i++) {
		if(differSpeed.at(i) == '/') {
			str_differ = differSpeed.substr(0,i);
			str_spd = differSpeed.substr(i+1,2);
			break;
		}
	}	
	sub_value = atoi(str_differ.c_str()); //differ ;int
	spd = atoi(str_spd.c_str()); //speed ;int
	
	sub_value -= 32; // ideal value = 20
	if(0 <= abs(sub_value) && abs(sub_value) <= mid_threshold){
		angle = 90;
		sub_value = 0;	
	}
	else if(mid_threshold < abs(sub_value) && abs(sub_value) <= 70){
		if(sub_value < 0) sub_value = (sub_value + mid_threshold) * 0.1; // 오른쪽으로 치우침. 왼쪽으로 가야함.
		else sub_value = (sub_value - mid_threshold) * 0.1;
	}
	else {	// 
		if(sub_value < 0) sub_value = (sub_value + mid_threshold) * 0.65;
		else sub_value = (sub_value - mid_threshold) * 0.65;
	}

	/*if (0 <= abs(sub_value) && abs(sub_value) <= 15 && spd > 0) {
		spd = 72;
	}*/

	if(spd != 0){
		printf("differ: %d / speed: %d\n", sub_value, spd);
	}
	angle = 90 - sub_value;

	if(angle > max_angle) angle = max_angle;
	if(angle < min_angle) angle = min_angle;

	cmd.linear.x = spd;
	cmd.angular.z = angle;

	pub2.publish(cmd);
}


int main(int argc, char **argv)
{
	//for pub to motor
	cmd.linear.x = spd;
	ros::init(argc, argv, "msg_publisher");
	ros::NodeHandle nh2;

	pub2 = nh2.advertise<geometry_msgs::Twist>("data_msg", 70);
	sub2 = nh2.subscribe("cam_msg",70,msgCallback);

	ros::Rate loop_rate(1);


	//for sub from cam
	printf("mailbox is started");

	while(ros::ok())
	{
		ros::spinOnce();
		/*
		   printf("\nESC to quit");
		   if(kbhit){
		   key = getch();
		   }else{
		   key = 0;
		   }
		   switch(key){
		   case 27:
		   msg2.data = 5;
		   pub2.publish(msg2);
		   printf("stop and turn off");
		   return 0;


		   case 's':
		   msg2.data = 2;
		   pub2.publish(msg2);
		   break;

		   }
		/*
		ROS_INFO("send msg = %d \n", msg2.data);
		 */

		//loop_rate.sleep();

	}

	return 0;
}
/*
   int kbhit(void)
   {
   struct termios oldt, newt;
   int ch;
   int oldf;
   tcgetattr(STDIN_FILENO, &oldt);
   newt = oldt;
   newt.c_lflag &= ~(ICANON | ECHO);
   tcsetattr(STDIN_FILENO, TCSANOW, &newt);
   oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
   fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
   ch = getchar();
   tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
   fcntl(STDIN_FILENO, F_SETFL, oldf);
   if(ch != EOF)
   {
   ungetc(ch, stdin);
   return 1;
   }
   return 0;
   }
   int getch(){
   int ch;
   struct termios buf, save;
   tcgetattr(0,&save);
   buf = save;
   buf.c_lflag &= ~(ICANON|ECHO);
   buf.c_cc[VMIN] = 1;
   buf.c_cc[VTIME] = 0;
   tcsetattr(0, TCSAFLUSH, &buf);
   ch = getchar();
   tcsetattr(0, TCSAFLUSH, &save);
   return ch;
   }
 */
