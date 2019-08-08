#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
//#include <iostream>
#include <geometry_msgs/Twist.h>

using namespace std;

geometry_msgs::Twist cmd;
int kbhit();
int getch();
int sub_value=0;
ros::Publisher pub2;
ros::Subscriber sub2;
std_msgs::Int8 msg2;
int key = 0;

int spd=65;
int angle=90;
int mid_threshold = 30;
int min_angle = 40;
int max_angle = 140;


void msgCallback(const std_msgs::Int16::ConstPtr& given_msg) {

	sub_value = (given_msg->data);

	if(0 <= abs(sub_value) && abs(sub_value) <= mid_threshold){
		angle = 90;	
	}
	else if(mid_threshold < abs(sub_value) && abs(sub_value) <= 50){
		if(sub_value < 0) sub_value = (sub_value + mid_threshold) * 0.35;
		else sub_value = (sub_value - mid_threshold) * 0.35;
	}
	else {	// 
		if(sub_value < 0) sub_value = (sub_value + mid_threshold) * 0.8;
		else sub_value = (sub_value - mid_threshold) * 0.8;
	}

	angle = 90 - sub_value;

	if(angle > max_angle) angle = max_angle;
	if(angle < min_angle) angle = min_angle;
	printf("## angle: %d\n", angle);

	// add by seung
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

	pub2 = nh2.advertise<geometry_msgs::Twist>("data_msg", 100);
	sub2 = nh2.subscribe("cam_msg",100,msgCallback);

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
