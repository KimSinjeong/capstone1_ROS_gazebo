#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "core_msgs/ball_position.h"

#include "opencv2/opencv.hpp"

float MAP_CX = 200.5;
float MAP_CY = 200.5;
float MAP_RESOL = 0.015;             // Map resoultion [cm]
int MAP_WIDTH = 600;
int MAP_HEIGHT = 600;
int MAP_CENTER = 50;
int OBSTACLE_PADDING = 2;           // Obstacle Size
int OBSTACLE_CONNECT_MAX = 15;      // Range to connect obstacles

int init_ball;
int init_lidar;

int lidar_size;
float lidar_degree[600];
float lidar_distance[600];

int ball_number;
float ball_X[20];
float ball_Y[20];

float cosine = 0;
float sine = 0;
float orient = 0;

float robot_x =0, robot_y=0;

float first=1;
boost::mutex map_mutex;

using namespace cv;
using namespace std;
#define RAD2DEG(x) ((x)*180./M_PI)

bool check_point_range(int cx, int cy)
{
    return (cx<MAP_WIDTH-1)&&(cx>0)&&(cy<MAP_HEIGHT-1)&&(cy>0);
}



void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    map_mutex.lock();
    int count = position->size;
    ball_number=count;
    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->img_x[i];
        ball_Y[i]=position->img_y[i];
      	std::cout << "ball_X : "<< ball_X[i];
      	std::cout << "ball_Y : "<< ball_Y[i]<<std::endl;
    }
    map_mutex.unlock();
}
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    map_mutex.lock();
    int count = scan->angle_max / scan->angle_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
        // std::cout << "degree : "<< lidar_degree[i];
        // std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
    }
    map_mutex.unlock();

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_show_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);

    while(ros::ok){
        cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
        // Drawing Lidar data
        float obstacle_x, obstacle_y;
        int cx, cy;
        int cx1, cx2, cy1, cy2;
        for(int i = 0; i < lidar_size; i++)
        {
	    if(lidar_distance[i] > 0.2){
            obstacle_x = lidar_distance[i]*cos(lidar_degree[i]);
            obstacle_y = lidar_distance[i]*sin(lidar_degree[i]);
            cx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
            cy = MAP_HEIGHT/2 + (int)(obstacle_x/MAP_RESOL);
            cx1 = cx-OBSTACLE_PADDING;
            cy1 = cy-OBSTACLE_PADDING;
            cx2 = cx+OBSTACLE_PADDING;
            cy2 = cy+OBSTACLE_PADDING;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
		
                //cv::line(map, cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),cv::Point(cx, cy),cv::Scalar(63,63,0), 1);
                cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(255,255,0), -1);
            }
	    }
        }

vector<Vec2f> lines;
cv::Mat map2 = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
cv::Mat map3 = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
	cvtColor( map, map, CV_RGB2GRAY );
	
	//GaussianBlur(map,map,Size(7,7),0,0);
	Canny(map,map2,70,150,3);  

cv::HoughLines(map2, lines, 1, CV_PI / 180.0, 80);

 Vec2f params;
 float rho, theta;
 float c, s;
 float x0[50], y0[50];
 // for(int k=0; k<lines.cols; k++)
 //cout << lines.size() <<endl;
 for (int k = 0; k < lines.size(); k++)
 {
  //  params = lines.at<Vec2f>(0, k);
  params = lines[k];
  rho = params[0];
  theta = params[1];
   
  // drawing a line
  c = cos(theta);
  s = sin(theta);
  x0[k] = rho*c;
  y0[k] = rho*s;
  //cout << "real x ,y " << endl;
  //cout << "x : " << x0[k] << " y : " << y0[k] << endl; 
  Point pt1, pt2;
  pt1.x = cvRound(x0[k] + 1000 * (-s));
  pt1.y = cvRound(y0[k] + 1000 * (c));
  pt2.x = cvRound(x0[k] - 1000 * (-s));
  pt2.y = cvRound(y0[k] - 1000 * (c));
  line(map3, pt1, pt2, Scalar(0, 0, 255), 2);
 }

//짧은 직선 쪽 각도 조절

cv::Mat map_s = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
for(int i = 0; i < lidar_size; i++)
        {
	 if(lidar_distance[i] > 0.2){
            obstacle_x = lidar_distance[i]*cos(lidar_degree[i]);
            obstacle_y = lidar_distance[i]*sin(lidar_degree[i]);
            cx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
            cy = MAP_HEIGHT/2 + (int)(obstacle_x/MAP_RESOL);
            cx1 = cx-OBSTACLE_PADDING;
            cy1 = cy-OBSTACLE_PADDING;
            cx2 = cx+OBSTACLE_PADDING;
            cy2 = cy+OBSTACLE_PADDING;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
		if(lidar_degree[i]*180/3.1415 > 105+orient && lidar_degree[i]*180/3.1415 < 255+orient  ){
                //cv::line(map, cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),cv::Point(cx, cy),cv::Scalar(63,63,0), 1);
                cv::rectangle(map_s,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(255,255,0), -1);
                }
	    }
	}
        }
cv::imshow("map_s", map_s);

// 짧은 쪽 직선 찾기
vector<Vec2f> lines_s;
cv::Mat map2_s = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
cv::Mat map3_s = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
	cvtColor( map_s, map_s, CV_RGB2GRAY );
	
	//GaussianBlur(map,map,Size(7,7),0,0);
	Canny(map_s,map2_s,70,150,3);  

cv::HoughLines(map2_s, lines_s, 1, CV_PI / 180.0, 50);
 // for(int k=0; k<lines.cols; k++)
 //cout << lines_s.size() <<endl;
 for (int k = lines.size(); k < lines.size()+lines_s.size(); k++)
 {
  //  params = lines.at<Vec2f>(0, k);
  params = lines_s[k-lines.size()];
  rho = params[0];
  theta = params[1];
   
  // drawing a line
  c = cos(theta);
  s = sin(theta);
  x0[k] = rho*c;
  y0[k] = rho*s;
  //cout << "x : " << x0 << " y : " << y0 << endl; 
  Point pt1, pt2;
  pt1.x = cvRound(x0[k] + 1000 * (-s));
  pt1.y = cvRound(y0[k] + 1000 * (c));
  pt2.x = cvRound(x0[k] - 1000 * (-s));
  pt2.y = cvRound(y0[k] - 1000 * (c));
  line(map3, pt1, pt2, Scalar(0, 0, 255), 2);
 }
	cv::imshow("Frame",map3);
        cv::waitKey(50);

//여러번 찾아진 x,y 값 하나로 만들기 - 직선의 평균
if(lines.size() ==0){
x0[0] = 0;
y0[0] = 0;
}
int num1 = 1, num2 = 1, num3 = 1, num4 =1;
 float sumx1 = x0[0], sumy1 = y0[0];
 float sumx2 = 0, sumy2 = 0;
 float sumx3 = 0, sumy3 = 0;
 float sumx4 = 0, sumy4 = 0;
//cout << lines.size() << endl;
for(int i = 1; i < lines.size(); i++){
	if(abs(sumx1 - x0[i]) > 30 || abs(sumy1 - y0[i]) > 30){
		sumx2 = x0[i];
		sumy2 = y0[i];
	}
 }
for (int i = 1; i < lines.size(); i++){
	if((abs(sumx2 - x0[i]) > 30 || abs(sumy2 - y0[i]) > 30) && (abs(sumx1 - x0[i]) > 30 || abs(sumy1 - y0[i]) > 30)){
		sumx3 = x0[i];
		sumy3 = y0[i];
	}
 }
for (int i = 1; i < lines.size(); i++){
	if((abs(sumx2 - x0[i]) > 30 || abs(sumy2 - y0[i]) > 30) && (abs(sumx1 - x0[i]) > 30 || abs(sumy1 - y0[i]) > 30) && (abs(sumx3 - x0[i]) > 30 || abs(sumy3 - y0[i]) > 30)){
		sumx4 = x0[i];
		sumy4 = y0[i];
	}
 }

// 직선 평균 구하기
float bufx, bufy;
bufx = sumx1; bufy = sumy1;
for(int i = 1; i < lines.size(); i++){
	if(abs(bufx - x0[i]) < 30 && abs(bufy - y0[i]) < 30){
		sumx1 = sumx1 + x0[i];
		sumy1 = sumy1 + y0[i];
		num1++;
	}
 }
bufx = sumx2; bufy = sumy2;
for(int i = 1; i < lines.size(); i++){
	if(abs(bufx - x0[i]) < 30 && abs(bufy - y0[i]) < 30){
		sumx2 = sumx2 + x0[i];
		sumy2 = sumy2 + y0[i];
		num2++;
	}
 }
bufx = sumx3; bufy = sumy3;
for(int i = 1; i < lines.size(); i++){
	if(abs(bufx - x0[i]) < 30 && abs(bufy - y0[i]) < 30){
		sumx3 = sumx3 + x0[i];
		sumy3 = sumy3 + y0[i];
		num3++;
	}
 }
bufx = sumx4; bufy = sumy4;
for(int i = 1; i < lines.size(); i++){
	if(abs(bufx - x0[i]) < 30 && abs(bufy - y0[i]) < 30){
		sumx4 = sumx4 + x0[i];
		sumy4 = sumy4 + y0[i];
		num4++;
	}
 }
sumx1 = sumx1 / num1;
sumx2 = sumx2 / num2;
sumx3 = sumx3 / num3;
sumx4 = sumx4 / num4;
sumy1 = sumy1 / num1;
sumy2 = sumy2 / num2;
sumy3 = sumy3 / num3;
sumy4 = sumy4 / num4;

//cout << "start" << endl;
//cout << "x: " << sumx1 << "y: " << sumy1 << endl;
//cout << "x: " << sumx2 << "y: " << sumy2 << endl;
//cout << "x: " << sumx3 << "y: " << sumy3 << endl;
//cout << "x: " << sumx4 << "y: " << sumy4 << endl;

//로봇 방향 계산
cosine = sumx1 / sqrt(pow(sumx1,2)+pow(sumy1,2));
sine = sumy1 / sqrt(pow(sumx1,2)+pow(sumy1,2));
if(first < 10){
	orient = acos(cosine);
	first = first + 1;
	orient = orient*180/3.1415;
}
else if(abs(orient-acos(cosine)*180/3.1415) < 10 ){
	orient = acos(cosine)*180/3.1415;
}


float angle = orient/180*3.1415;

//로봇 위치 계산
float xx1=0, yy1=0, xx2=0, yy2=0,xx3=0, yy3=0,xx4=0, yy4=0, xr1=0, yr1=0, xr2=0, yr2=0,xr3=0, yr3=0,xr4=0, yr4=0;
float robot_xx = 0, robot_yy = 0;
float dis12=0, dis13=0, dis14=0, dis23=0, dis24=0, dis34=0;

	xx1 = sumx1-300;
	yy1 = 300-sumy1;
	xr1 = -1*(xx1*cos(angle) - yy1*sin(angle));
	yr1 = xx1*sin(angle) + yy1*cos(angle);
	if(sumx2 != 0 || sumy2 !=0 ){
		xx2 = sumx2-300;
		yy2 = 300-sumy2;
		xr2 = -1*(xx2*cos(angle) - yy2*sin(angle));
		yr2 = xx2*sin(angle) + yy2*cos(angle);
		dis12 = sqrt(pow(xr2-xr1,2)+pow(yr2-yr1,2)); 
	}
	if(sumx3 != 0 || sumy3 !=0 ){
		xx3 = sumx3-300;
		yy3 = 300-sumy3;
		xr3 = abs(xx3*cos(angle) - yy3*sin(angle));
		yr3 = abs(xx3*sin(angle) + yy3*cos(angle));
		dis13 = sqrt(pow(xr3-xr1,2)+pow(yr3-yr1,2)); 
		dis23 = sqrt(pow(xr2-xr3,2)+pow(yr2-yr3,2)); 
	}
	if(sumx4 != 0 || sumy4 !=0 ){
		xx4 = 300-sumx4;
		yy4 = 300-sumy4;
		xr4 = xx4*cos(angle) - yy4*sin(angle);
		yr4 = xx4*sin(angle) + yy4*cos(angle);
		dis14 = sqrt(pow(xr4-xr1,2)+pow(yr4-yr1,2)); 
		dis24 = sqrt(pow(xr2-xr4,2)+pow(yr2-yr4,2)); 
		dis34 = sqrt(pow(xr3-xr4,2)+pow(yr3-yr4,2)); 
	}

if(first < 10){
	if(dis12 > 180 && dis12 < 220){
		if(xr1 > 0) robot_x = xr1;
		else robot_x = xr2; 
		if(yr3 < 50) robot_y = yr3;
		if(yr4 < 50) robot_y = yr4;
	}
	else if(dis13 > 180 && dis13 < 220){
		if(xr1 > 0) robot_x = xr1;
		else robot_x = xr3; 
		if(yr2 < 50) robot_y = yr2;
		if(yr4 < 50) robot_y = yr4;
	}
	else if(dis14 > 180 && dis14 < 220){
		if(xr1 > 0) robot_x = xr1;
		else robot_x = xr4;
		if(yr2 < 50) robot_y = yr2;
		if(yr3 < 50) robot_y = yr3;
	}
	else if(dis23 > 180 && dis23 < 220){
		if(xr2 > 0) robot_x = xr2;
		else robot_x = xr3; 
		if(yr1 < 50) robot_y = yr1;
		if(yr4 < 50) robot_y = yr4;
	}
	else if(dis24 > 180 && dis24 < 220){
		if(xr2 > 0) robot_x = xr2;
		else robot_x = xr4; 
		if(yr1 < 50) robot_y = yr1;
		if(yr3 < 50) robot_y = yr3;
	}
	else if(dis34 > 180 && dis34 < 220){
		if(xr3 > 0) robot_x = xr3;
		else robot_x = xr4;
		if(yr1 < 50) robot_y = yr1;
		if(yr2 < 50) robot_y = yr2;
	}
}
else{
	if(dis12 > 180 && dis12 < 220){
		if(orient <= 90 || orient > 270){
			if(sumx1 < sumx2) robot_xx = xr1;
			else robot_xx = xr2; 
		}
		if(orient > 90 && orient <= 270){
			if(sumx1 > sumx2) robot_xx = xr1;
			else robot_xx = xr2; 
		}
		if(abs(robot_y - yr3) < 10) robot_y = yr3;
		else if(abs(robot_y - yr4) < 10) robot_y = yr4;
	}
	else if(dis13 > 180 && dis13 < 220){
		if(orient <= 90 || orient > 270){
			if(sumx1 < sumx3) robot_xx = xr1;
			else robot_xx = xr3; 
		}
		if(orient > 90 && orient <= 270){
			if(sumx1 > sumx3) robot_xx = xr1;
			else robot_xx = xr3; 
		}
		if(abs(robot_y - yr2) < 10) robot_y = yr2;
		else if(abs(robot_y - yr4) < 10) robot_y = yr4;
	}
	else if(dis14 > 180 && dis14 < 220){
		if(orient <= 90 || orient > 270){
			if(sumx1 < sumx4) robot_xx = xr1;
			else robot_xx = xr4; 
		}
		if(orient > 90 && orient <= 270){
			if(sumx1 > sumx4) robot_xx = xr1;
			else robot_xx = xr4; 
		}
		if(abs(robot_y - yr2) < 10) robot_y = yr2;
		else if(abs(robot_y - yr3) < 10) robot_y = yr3;
	}
	else if(dis23 > 180 && dis23 < 220){
		if(orient <= 90 || orient > 270){
			if(sumx2 < sumx3) robot_xx = xr2;
			else robot_xx = xr3; 
		}
		if(orient > 90 && orient <= 270){
			if(sumx2 > sumx3) robot_xx = xr2;
			else robot_xx = xr3; 
		}
		if(abs(robot_y - yr1) < 10) robot_y = yr1;
		else if(abs(robot_y - yr4) < 10) robot_y = yr4;
	}
	else if(dis24 > 180 && dis24 < 220){
		if(orient <= 90 || orient > 270){
			if(sumx2 < sumx4) robot_xx = xr2;
			else robot_xx = xr4; 
		}
		if(orient > 90 && orient <= 270){
			if(sumx2 > sumx4) robot_xx = xr2;
			else robot_xx = xr4; 
		}
		if(abs(robot_y - yr3) < 10) robot_y = yr3;
		else if(abs(robot_y - yr1) < 10) robot_y = yr1;
	}
	else if(dis34 > 180 && dis34 < 220){
		if(orient <= 90 || orient > 270){
			if(sumx3 < sumx4) robot_xx = xr3;
			else robot_xx = xr4; 
		}
		if(orient > 90 && orient <= 270){
			if(sumx3 > sumx4) robot_xx = xr3;
			else robot_xx = xr4; 
		}
		if(abs(robot_y - yr1) < 10) robot_y = yr1;
		else if(abs(robot_y - yr2) < 10) robot_y = yr2;
	}
}
//cout << "start" << endl;
//cout << "x: " << xr1 << "y: " << yr1 << endl;
//cout << "x: " << xr2 << "y: " << yr2 << endl;
//cout << "x: " << xr3 << "y: " << yr3 << endl;
//cout << "x: " << xr4 << "y: " << yr4 << endl;


if(abs(robot_x - robot_xx) < 10 && abs(robot_y - robot_yy) < 10 ){
	robot_x = robot_xx;
}

cout << "orientation: " << orient << " degree" << endl;
cout << "position: x " << robot_x*0.015 << " y " << robot_y*0.015 << endl; 




	//cv::imshow("map_s",map3_s);
	//cv::waitKey(50);

        // Drawing ball
        for(int i = 0; i < ball_number; i++)
        {
            cx =(int)(ball_X[i]/4);
            cy =(int)(ball_Y[i]/4);
            cx1 = cx-OBSTACLE_PADDING*2;
            cy1 = cy-OBSTACLE_PADDING*2;
            cx2 = cx+OBSTACLE_PADDING*2;
            cy2 = cy+OBSTACLE_PADDING*2;

            if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
            {
                cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(0,0,255), -1);
            }
        }
        // Drawing ROBOT
        cv::circle(map,cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),3,cv::Scalar(255,0,0),-1);

 

        if(cv::waitKey(50)==113){  //wait for a key command. if 'q' is pressed, then program will be terminated.
            return 0;
        }
        ros::spinOnce();
    }



    return 0;
}
