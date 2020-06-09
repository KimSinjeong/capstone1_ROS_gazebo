#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <boost/thread.hpp>

#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/robot_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "opencv2/opencv.hpp"

#include <pcl/registration/transforms.h>

// #define RAD2DEG(x) ((x)*180./M_PI)
#define sampleFreq	512.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;
float avg_dist;
bool exist_inf;
bool exist_inf_pillar;

int b_size;
float X[10];
float Y[10];
float X_tmp[10];
float Y_tmp[10];
float inverse_dist[10];

float X_r[10];
float Y_r[10];

float X_g;
float Y_g;

int r_size;
int r_size_old;

bool start = 0;
int mode = 0;
float vel_left = 0;
float vel_right = 0;
bool flag = 1;
int rev_dir = 0;
bool detect_b;
bool detect_r;
bool detect_g;
int c = 0;
int c1 = 0;
int c2 = 0;
int j = 0;
int prev;
int sub_mode = 0;
bool flag_m4m5 = false;
int prev_mode;
bool flag_m2 = false;
bool flag_m8 = false;
bool flag_withm4 = false;
bool flag_m6 = true;
int ball_count = 0;
int rot_spd;
bool isFinish = true;
int k;
bool flag_obs = false;
bool flag_big_rotate;

//imu
float _gx, _gy, _gz, _ax, _ay, _az;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float beta = betaDef;

//lidar
float front;
float behind;
float left;
float right;
float fr;
float fl;
bool detect_front;
bool detect_behind;
bool detect_left;
bool detect_right;
bool detect_fl;
bool detect_fr;
int vibration = 0;
int min_index;

//p3d
float r_pose[3];
float r_orient[4];
float r_theta;


//global position variable
float pillar_x[4] = {4, 5.5, 5.5, 7};
float pillar_y[4] = {1.5, 2.3, 0.7, 1.5};
float goal_x = 8;
float goal_y = 1.5;
float wall_r = 0;
float wall_l = 3;
float wall_f = 8;
float wall_b = 3;
float gB_x;
float gB_y;
float l[3];
bool is_line_left;
float distance_pillar;
int closest_idx;
float target_x;
float target_y;
bool rot_left;
int store_mode;

bool isenabled = false;

float distance_btw(float x1, float y1, float x2, float y2){
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

bool is_turn_left(float t, float v_x, float v_y){
	if(-v_x*sin(t)+v_y*cos(t)>0){
		return true;
	}
	return false;
}

float DiffandNorm(float e_x, float e_y, float s_x, float s_y, int m){
	if(m == 0){
		return (e_x-s_x)/distance_btw(e_x, e_y, s_x, s_y);
	}else{
		return (e_y-s_y)/distance_btw(e_x, e_y, s_x, s_y);
	}
}

bool is_matched(){
	float v1_x = cos(r_theta);
	float v1_y = sin(r_theta);
	float inner_prod;
	float v2_x = target_x-r_pose[0];
	float v2_y = target_y-r_pose[1];
	v2_x = v2_x/distance_btw(target_x, target_y, r_pose[0], r_pose[1]);
	v2_y = v2_y/distance_btw(target_x, target_y, r_pose[0], r_pose[1]);
	inner_prod = v1_x*v2_x+v1_y*v2_y;
	if(abs(1-inner_prod)<0.005){
		return true;
	}else{
		return false;
	}
}

int closest(){
	float tmp;
	int idx = -1;
	distance_pillar = 3.5;
	for(int i =0; i<4; i++){
		tmp = distance_btw(pillar_x[i], pillar_y[i], gB_x, gB_y);
		if(distance_pillar>tmp){
			distance_pillar = tmp;
			idx = i;
		}
	}
	return idx;
}

bool is_close(){
	int normal;
	closest_idx = closest();
	normal = 1/distance_btw(pillar_x[closest_idx], pillar_y[closest_idx], gB_x, gB_y);
	if(distance_pillar<0.45){
		target_x = gB_x + 0.5*normal*(gB_x - pillar_x[closest_idx]);
		target_y = gB_y + 0.5*normal*(gB_y - pillar_y[closest_idx]);
		return true;
	}else{
		return false;
	}
}

void camera2global(){
	gB_x = r_pose[0]+Y[0]*cos(r_theta)+X[0]*sin(r_theta);
	gB_y = r_pose[1]+Y[0]*sin(r_theta)-X[0]*cos(r_theta);
}

void set_vel(float left, float right){
	vel_left = left;
	vel_right = right;
}

bool originated = false;
float data[16];
Eigen::Matrix4f origin, target;

void global_Callback(const core_msgs::robot_position::ConstPtr& msg){
	for (int i = 0; i < 16; i++) data[i] = msg->data[i];

	if (data[0] == 0.) {
        r_theta = (data[4] > 0)?(3.1415926/2):(-3.1415926/2);
    } else if (data[0] < 0.) {
        if (data[4] > 0.) {
            r_theta = atan(data[4]/data[0]) + 3.1415926;
        } else if (data[4] < 0.) {
            r_theta = atan(data[4]/data[0]) - 3.1415926;
        } else {
            r_theta = 3.1415926;
        }
    } else {
        r_theta = atan(data[4]/data[0]);
    }
	r_pose[0] = data[3] + 3.5;
	r_pose[1] = data[7] + 0.5;
	r_pose[2] = 0;

	std::cout<<"pose"<<std::endl;
	std::cout<<"x : "<<r_pose[0]<<std::endl;
	std::cout<<"y : "<<r_pose[1]<<std::endl;
	std::cout<<"theta = "<<RAD2DEG(r_theta)<<std::endl;
}

void set_Origin() {
	originated = true;
	origin << data[0], data[1], data[2], data[3],
		data[4], data[5], data[6], data[7],
		data[8], data[9], data[10], data[11],
		data[12], data[13], data[14], data[15];
}

bool get_Target(float pos[3]) {
	if (!originated) {
		std::cout << "Original position is not set yet" << std::endl;
		return false;
	}
	target << data[0], data[1], data[2], data[3],
		data[4], data[5], data[6], data[7],
		data[8], data[9], data[10], data[11],
		data[12], data[13], data[14], data[15];
	
	target = target*origin.inverse();

	pos[0] = target(0, 3);
	pos[1] = target(1, 3);
	if (target(0, 0) == 0.) {
		pos[2] = (target(1, 0) > 0)?(3.1415926/2):(-3.1415926/2);
	} else if (target(0, 0) < 0.) {
		if (target(1, 0) > 0.) {
			pos[2] = atan(target(1, 0)/target(0, 0)) + 3.1415926;
		} else if (target(1, 0) < 0.) {
			pos[2] = atan(target(1, 0)/target(0, 0)) - 3.1415926;
		} else {
			pos[2] = 3.1415926;
		}
	} else {
		pos[2] = atan(target(1, 0)/target(0, 0));
	}
}


void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{	
	front = 3.5;
	behind = 3.5;
	left = 3.5;
	right = 3.5;
	fl = 3.5;
	fr = 3.5;
	min_index = 0;
	detect_front = false;
	detect_behind = false;
	detect_left = false;
	detect_right = false;
	detect_fl = false;
	detect_fr = false;
	map_mutex.lock();

	int count = scan->angle_max / scan->angle_increment;
    lidar_size=count;
	// std::cout << "-------------------------------------------------------"<<std::endl;
    for(int i = 0; i < count; i++)
    {	
        lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        lidar_distance[i]=scan->ranges[i];
		if(isinf(lidar_distance[i]) || lidar_distance[i]<0.25){
			lidar_distance[i] = 3.5;
		}else{
			if(i ==0 || i == 10 || i == 20 || i == count-10 || i == count-20){
				if(front>lidar_distance[i]){
					front = lidar_distance[i];
					min_index = i;
				}
			}else if(i == 25 || i == 35 || i == 45){
				if(fl>lidar_distance[i]){
					fl = lidar_distance[i];
				}
			}else if(i == 75 || i == 90 || i == 105){
				if(left>lidar_distance[i]){
					left = lidar_distance[i];
				}
			}else if(i == 180 || i == 165 || i == 150 || i == 195 || i == 210){
				if(behind>lidar_distance[i]){
					behind = lidar_distance[i];
				}
			}else if(i == count-75 || i == count-90 || i == count-105){
				if(right>lidar_distance[i]){
					right = lidar_distance[i];
				}
			}else if(i == count - 25 || i == count - 35 || i == count - 45){
				if(fr>lidar_distance[i]){
					fr = lidar_distance[i];
				}
			}
		}	
    }
	if(front<0.5){
		detect_front = true;
	}
	if(behind < 0.35){
		detect_behind =  true;
	}
	if(left<0.35){
		detect_left = true;
	}
	if(right<0.35){
		detect_right = true;
	}
	if(fr<0.47){
		detect_fr = true;
	}
	if(fl<0.47){
		detect_fl = true;
	}
	if(!(detect_front || detect_fl || detect_fr)){
		vibration = 0;
	}
	map_mutex.unlock();

}

void b_camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{		
		int max_i = 0;
		start = true;
		b_size = position->size;
		if(b_size !=0){
			detect_b = true;
			for(int i = 0; i<b_size; i++){
				X_tmp[i] = position->img_x[i];
        		Y_tmp[i] = position->img_y[i];
				inverse_dist[i] = 1/(X_tmp[i]*X_tmp[i] + Y_tmp[i]*Y_tmp[i]);
			}
			for(int j = 0; j<b_size; j++){
				for(int i = 0; i<b_size; i++){
					if(inverse_dist[max_i]<inverse_dist[i]){
						max_i = i;
					}
				}
				X[j] = X_tmp[max_i];
				Y[j] = Y_tmp[max_i];
				inverse_dist[max_i] = 0;
			}
			// std::cout << "receive blue ball"<<std::endl;
		}else{
			// std::cout << "blue ball not detect"<<std::endl;
			detect_b = false;
		}
		

	
}

void r_camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
		start = true;
		r_size = position->size;
        if(r_size != 0){
			detect_r = true;
			for(int i = 0; i<r_size; i++){
				X_r[i] = position->img_x[i];
        		Y_r[i] = position->img_y[i];
			}

			// std::cout << "receive red ball"<<std::endl;
		}else{
			// std::cout << "red ball not detect"<<std::endl;
			detect_r = false;
		}

}

void g_camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
	start = true;
	r_size = position->size;
	if(r_size != 0){
		detect_g = true;
		X_g = position -> img_x[0];
		Y_g = position -> img_y[0];
		// std::cout << "receive red ball"<<std::endl;
	}else{
		// std::cout << "red ball not detect"<<std::endl;
		detect_g = false;
	}
}


int main(int argc, char **argv)
{	
    ros::init(argc, argv, "data_integration_node");
    ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position_b", 1000, b_camera_Callback);
	ros::Subscriber sub2 = n.subscribe<core_msgs::ball_position>("/position_r", 1000, r_camera_Callback);
	ros::Subscriber sub3 = n.subscribe<core_msgs::ball_position>("/position_g", 1000, g_camera_Callback);
	// ros::Subscriber sub4 = n.subscribe<nav_msgs::Odometry>("/ground_truth_pose", 1000, p3d_Callback);
	ros::Subscriber sub5 = n.subscribe<core_msgs::robot_position>("/robot_position", 1000, global_Callback);
	// ros::Publisher pub_left_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/left_wheel_velocity_controller/command", 10);
	// ros::Publisher pub_right_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/right_wheel_velocity_controller/command", 10);
	ros::Publisher pub_right_front_wheel = n.advertise<std_msgs::Float64>("model15/right_front_wheel_velocity_controller/command", 10);
    ros::Publisher pub_left_front_wheel = n.advertise<std_msgs::Float64>("model15/left_front_wheel_velocity_controller/command", 10);
    ros::Publisher pub_right_rear_wheel = n.advertise<std_msgs::Float64>("model15/right_rear_wheel_velocity_controller/command", 10);
    ros::Publisher pub_left_rear_wheel = n.advertise<std_msgs::Float64>("model15/left_rear_wheel_velocity_controller/command", 10);


    while(ros::ok){
			std_msgs::Float64 left_wheel_msg;
			std_msgs::Float64 right_wheel_msg;

			//center of x =-0.05
			if(start && isenabled){
				if(detect_b){
					c1=0;
					c2=0;
					if(mode == 0){ // mode 0 : rotate in place for matching the direction to the ball
						if(X[0]<-0.05){ // if the ball is at the right side
							if(X[0]>-0.06){
								mode = 1;
								set_vel(0,0);
							}else{
								std::cout << "mode : 0, positive"<<std::endl;
								set_vel(-1,1);
							}
						}else{ // if the ball is at the left side
							if(X[0]<-0.04){
								mode = 1;
								set_vel(0,0);
							}else{
								std::cout << "mode : 0, negative"<<std::endl;
								set_vel(1,-1);
							}
						}
						
					}else if(mode == 1){ // mode 1 : go straight
						if(Y[0]<0.21){
							std::cout << "mode : 1, stop"<<std::endl;
							set_vel(0,0);
							flag =1;
							mode  = 5;
							c = 0;
							rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
						}else{
							if(flag && Y[0]<1){
								camera2global();
								std::cout<<"gB_x : "<<gB_x<<", gB_y : "<<gB_y<<std::endl;
								if(is_close()){
									mode = 16;
									set_vel(0,0);
									rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
								}else{
									mode = 0;
									set_vel(0,0);
									flag = 0;

								}
								std::cout << "closest index : "<<closest_idx<<std::endl;
								std::cout << "distance_pillar : "<< distance_pillar<<std::endl;
								std::cout << "target_x :"<<target_x<<", target_y : "<< target_y<<std::endl; 
							}else{
								std::cout << "mode : 1, go"<<std::endl;
								set_vel(5,5);
							}
						}
					}

				}else if(mode < 2){ // when the blue ball is not detect
					if(c1<600){
						c1++;
						std::cout << "detect mode, rotate"<<std::endl;
						if(rev_dir == 0){
							set_vel(-5,5);
						}else{
							set_vel(5,-5);
						}
						mode = 0;
					}else{
						if(c2<300){
							c2++;
							std::cout << "detect mode, go straight"<<std::endl;
							set_vel(3,3);
							mode = 0;
						}else{
							c1=0;
							c2=0;
						}
					}
				}

				if(mode == 16){
					if(sub_mode == 0){
						if(is_matched()){
							sub_mode =1;
							set_vel(0,0);
						}else{
							std::cout << "mode 16, direction matching"<<std::endl;
							if(rot_left){
								set_vel(-2,2);
							}else{
								set_vel(2,-2);
							}
						}
					}else if(sub_mode == 1){
						if(distance_btw(r_pose[0], r_pose[1], target_x, target_y)<0.1){
							sub_mode = 2;
							set_vel(0,0);
							rot_left = is_turn_left(r_theta, gB_x-r_pose[0], gB_y-r_pose[1]);
						}else{
							std::cout << "mode 16, go straight"<<std::endl;
							set_vel(3,3);
						}
					}else if(sub_mode == 2){
						if(abs(1-(DiffandNorm(gB_x, gB_y,r_pose[0],r_pose[1], 0)*cos(r_theta)+DiffandNorm(gB_x, gB_y,r_pose[0],r_pose[1], 1)*sin(r_theta)))<0.005){
							sub_mode = 3;
							set_vel(0,0);
						}else{
							std::cout << "mode 16, rotate"<<std::endl;
							if(rot_left){
								set_vel(-2,2);
							}else{
								set_vel(2,-2);
							}
						}
					}else if(sub_mode == 3){
						if(X[0]<-0.05){ // if the ball is at the right side
							if(X[0]>-0.06){
								sub_mode = 4;
								set_vel(0,0);
							}else{
								std::cout << "mode : 16, positive"<<std::endl;
								set_vel(-1,1);
							}
						}else{ // if the ball is at the left side
							if(X[0]<-0.04){
								sub_mode = 4;
								set_vel(0,0);
							}else{
								std::cout << "mode : 16, negative"<<std::endl;
								set_vel(1,-1);
							}
						}
					}else if(sub_mode == 4){
						if(Y[0]<0.21){
							std::cout << "mode : 16, stop"<<std::endl;
							set_vel(0,0);
							mode = 5;
							sub_mode = 0;
							flag =1;
							rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
						}else{
							std::cout << "mode : 16, go"<<std::endl;
							set_vel(5,5);
						}
					}

				}

				if(mode == 2){ // mode 2 : avoid action - rotate 
					if(c<35){ // until the red ball is not detected on the camera, rotate.
						std::cout << "mode : 2, direction avoid, red ball"<<std::endl;
						c++;
						if(rev_dir == 0){
							set_vel(5,-5);
						}else{
							set_vel(-5,5);
						}
					}else{
						c = 0;
						mode = 3;
						set_vel(0,0);
					}
				}else if(mode == 3){ // mode 3 : avoid action - revolving around
					std::cout << "mode : 3, revolving"<<std::endl;
					if(rev_dir == 0){
						set_vel(3.5,6);
					}else{
						set_vel(6,3.5);
					}
					if(c == 30){
						flag_m2 = false;
					}
					if(c < 150){
						c++;
					}else{
						c = 0;
						mode = 0;
						if(store_mode == 16){
							mode = 16;
							rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
						}
						else if(store_mode == 6){
							mode = 5;
							sub_mode = 0;
							rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
						}
						flag = 1;
						set_vel(0,0);
					}
				}else if(mode == 10){
					if(sub_mode == 0){
						if(flag_big_rotate){
							if(c<50){ // until the red ball is not detected on the camera, rotate.
								std::cout << "mode : 10, direction avoid, pillar"<<std::endl;
								c++;
								if(rev_dir == 0){
									set_vel(5,-5);
								}else{
									set_vel(-5,5);
								}
							}else{
								c = 0;
								sub_mode = 1;
								set_vel(0,0);
							}

						}else{
							if(c<35){ // until the red ball is not detected on the camera, rotate.
								std::cout << "mode : 10, direction avoid, pillar"<<std::endl;
								c++;
								if(rev_dir == 0){
									set_vel(5,-5);
								}else{
									set_vel(-5,5);
								}
							}else{
								c = 0;
								sub_mode = 1;
								set_vel(0,0);
								flag_big_rotate = false;
							}
						}
					}else if(sub_mode == 1){
						std::cout << "mode : 10, revolving"<<std::endl;
						if(rev_dir == 0){
							set_vel(3,6);
						}else{
							set_vel(6,3);
						}
						if(c == 100){
							flag_m2 = false;
						}
						if(c < 150){
							c++;
						}else{
							c = 0;
							mode = 0;
							sub_mode = 0;
							flag = 1;
							if(store_mode == 16){
								mode = 16;
								rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
							}
							else if(store_mode == 6){
								mode = 5;
								sub_mode = 0;
								rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
							}
							set_vel(0,0);
						}
					}
				}
				if((mode < 4 || (mode == 6 && sub_mode !=2 && sub_mode !=1) || mode == 10 || (mode == 16 && sub_mode <2)) && ((detect_r && Y_r[0]<0.35 && X_r[0]<0.3 && X_r[0]>-0.4) || detect_front || detect_fl || detect_fr)&& !flag_m2){ // interrupt if the red ball is in front of the robot without blue ball -> go to mode 2
					sub_mode = 0;
					if(mode != 2 && mode !=3 && mode!=10){
						store_mode = mode;
					}
					if(detect_front || detect_fl || detect_fr){
						if(detect_front){
							if(min_index == 10 || min_index == 20){
								rev_dir = 0;
							}else if(min_index == lidar_size - 10 || min_index == lidar_size - 20){
								rev_dir = 1;
							}else{
								if(rand()%2){
									rev_dir = 0;
								}else{
									rev_dir = 1;
								}
							}
						}
						if(detect_fr){
							rev_dir = 1;
						}
						if(detect_fl){
							rev_dir = 0;
						}
						if(left <0.5){
							rev_dir = 0;
						}
						if(right <0.5){
							rev_dir = 1;
						}
						if(vibration<15){
							vibration++;
						}else{
							vibration = 0;
							mode = 10;
							flag_m2 = true;
							c = 0;
							if(detect_front){
								flag_big_rotate = true;
							}else{
								flag_big_rotate = false;
							}
						}
					}else{
						if(X_r[0]>0){
							rev_dir = 1;
						}else{
							rev_dir = 0;
						}
						mode = 2;
						flag_m2 = true;
						c = 0;
					}
				}

				if(mode == 5){//direction matching
					if(sub_mode == 0){
						std::cout << "mode : 5, go little"<<std::endl;
						if(c<15){
							c++;
							set_vel(5,5);
						}else{
							c = 0;
							sub_mode = 1;
							set_vel(0,0);
						}
					}else if(sub_mode == 1){
						std::cout << "mode : 5, direction matching with goal"<<std::endl;
						if(abs(1-(DiffandNorm(goal_x, goal_y,r_pose[0],r_pose[1], 0)*cos(r_theta)+DiffandNorm(goal_x, goal_y,r_pose[0],r_pose[1], 1)*sin(r_theta)))<0.005){
							mode = 6;
							sub_mode = 0;
							c = 0;
							set_vel(0,0);
						}else{
							if(rot_left){
								set_vel(-2,2);
							}else{
								set_vel(2,-2);
							}
						}
					}

				}else if(mode == 6){ //go to the goal.
					if(sub_mode == 0){
						if(detect_g){
							if(Y_g<0.8){
								sub_mode = 1;
								set_vel(0,0);
							}else{
								std::cout << "mode : 6, go"<<std::endl;
								set_vel(3,3);
							}
						}else{
							if(distance_btw(r_pose[0], r_pose[1], goal_x, goal_y)<1){
								sub_mode = 1;
								set_vel(0,0);
							}else{
								std::cout << "mode : 6, go"<<std::endl;
								set_vel(3,3);
							}

						}
					}else if(sub_mode == 1){
						if(X_g<-0.05){ // if the ball is at the right side
							if(X_g>-0.06){
								sub_mode = 2;
								set_vel(0,0);
							}else{
								std::cout << "mode : 6, positive"<<std::endl;
								set_vel(-1,1);
							}
						}else{ // if the ball is at the left side
							if(X_g<-0.04){
								sub_mode = 2;
								set_vel(0,0);
							}else{
								std::cout << "mode : 6, negative"<<std::endl;
								set_vel(1,-1);
							}
						}
					}else if(sub_mode == 2){
						if(Y_g <0.18){ // if distance is smaller than 0.18, go to mode7(blue ball goal in)
							set_vel(0,0);
							c = 0;
							mode = 7;
							sub_mode = 0;
						}else{
							if(distance_btw(X_g, Y_g, r_pose[0], r_pose[1])<0.25){
								if(c<150){
									c++;
								}else{
									set_vel(0,0);
									c = 0;
									mode = 7;
									sub_mode = 0;
								}
							}else if(distance_btw(X_g, Y_g, r_pose[0], r_pose[1])<0.5){
								if(c<150){
									c++;
									std::cout << "mode : 6, go"<<std::endl;
									set_vel(3,3);
								}else{
									if(r_theta<M_PI){
										if(c<165){
											std::cout << "mode : 6, rotate little"<<std::endl;
											c++;
											set_vel(-3,3);
										}else{
											c = 0;
										}
									}else{
										if(c<165){
											std::cout << "mode : 6, rotate little"<<std::endl;
											c++;
											set_vel(3,-3);
										}else{
											c = 0;
										}
									}
								}
							}
							else{
								c = 0;
								std::cout << "mode : 6, go"<<std::endl;
								set_vel(3,3);
							}
						}
					}
				}

				else if(mode == 7){ // rotate 180 degree to find another ball
					if(sub_mode == 0){
						std::cout << "mode : 7, back"<<std::endl;
						if(c<90){
							c++;
							set_vel(-4,-4);
						}else{
							c = 0;
							set_vel(0,0);
							sub_mode = 1;
						}
					}else if(sub_mode == 1){
						std::cout << "mode : 7, rotate 180"<<std::endl;
						if(c<100){
							c++;
							if(rev_dir == 0){
								set_vel(-5,5);
							}else{
								set_vel(5,-5);
							}
						}else{
							c=0;
							set_vel(0,0);
							sub_mode = 0;
							mode = 0;
							ball_count++;
							flag_m6 = true;
							if(ball_count >2){
								mode = 9;
							}
						}
					}
				}

				left_wheel_msg.data=vel_left;   // set left_wheel velocity
				right_wheel_msg.data=vel_right;  // set right_wheel velocity
				pub_left_front_wheel.publish(left_wheel_msg);   // publish left_wheel velocity
				pub_right_front_wheel.publish(right_wheel_msg);  // publish right_wheel velocity
			}

		bool entrance_finished;
		if (!isenabled && n.getParam("/entrance_finished", entrance_finished)) {
			if (entrance_finished) { // First stop
				isenabled = true;
				left_wheel_msg.data=0.;   // set left_wheel velocity
				right_wheel_msg.data=0.;  // set right_wheel velocity
				pub_left_front_wheel.publish(left_wheel_msg);   // publish left_wheel velocity
				pub_right_front_wheel.publish(right_wheel_msg);  // publish right_wheel velocity
				pub_left_rear_wheel.publish(left_wheel_msg);   // publish left_wheel velocity
				pub_right_rear_wheel.publish(right_wheel_msg);  // publish right_wheel velocity
			}
		}

	    ros::Duration(0.025).sleep();
	    ros::spinOnce();
    }

    return 0;
}
