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
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>

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

int b_size2;
float X2[10];
float Y2[10];
float X2_tmp[10];
float Y2_tmp[10];
float inverse_dist2[10];

float X_r[10];
float Y_r[10];
float X_r2[10];
float Y_r2[10];

float X_g;
float Y_g;

int r_size;
int r_size2;

bool start = 0;
int mode = -1;
float vel_front_left = 0;
float vel_front_right = 0;
float vel_rear_left = 0;
float vel_rear_right = 0;
bool flag = 1;
int rev_dir = 0;
bool detect_b;
bool detect_b2;
bool detect_r;
bool detect_g;
bool detect_r2;
bool detect_g2;
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
bool isFinish = false;
int k;
bool flag_obs = false;
bool flag_big_rotate;
bool is_pos = true;
bool flag_16 = false;
bool flag_20 = false;
bool flag_21 = false;
bool flag_m11 = false;
bool flag_m12 = false;
bool long_continue = false;
int cont_count = 0;
bool flag_m20 = false;
int mode16_count = 0;
bool is_target_close_obstacle;


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
bool detect_front2;
bool detect_front3;
bool detect_behind;
bool detect_left;
bool detect_right;
bool detect_left2;
bool detect_right2;
bool detect_fl;
bool detect_fr;
int vibration = 0;
int min_index;
int left_wpn;
int right_wpn;
int wpn_idx;

//p3d
float r_pose[3];
float r_orient[4];
float r_theta;


//global position variable
float pillar_x[4] = {4, 5.5, 5.5, 7};
float pillar_y[4] = {1.5, 2.3, 0.7, 1.5};
float center_x = 5.5;
float center_y = 1.5;
int center_count = 0;
float goal_x = 8;
float goal_y = 1.5;
float wall_r = 0;
float wall_l = 3;
float wall_f = 8;
float wall_b = 3;
float lB_x;
float lB_y;
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
float lidar_x;
float lidar_y;
float lidar_target_x;
float lidar_target_y;
float blue_x[3];
float blue_y[3];
float distance_target;
float distance_lidar;

//locally position
float l_pos_x;
float l_pos_y;
float l_theta;
float data[16];

//velocity control
int switching = 0;
int switching_fast = 0;
int switching_middle = 0;
int switching_very_fast = 0;
int switching_long = 0;
int breaking = 0;
int start_going = 0;

bool isenabled = false;

//#define RAD2DEG(x) ((x)*180./M_PI)

// void detect_wp(){
// 	int n;
// 	float now;
// 	float next;
// 	int initial = 10;
// 	int i = 0;
// 	left_wpn = 2; // w-0, p-1, n-2
// 	right_wpn = 2;
// 	while(i<50){
// 		now = lidar_distance[initial+i];
// 		next = lidar_distance[initial+i+1];
// 		if(now<1.2 || next<1.2){
// 			if(abs(now-next)> 0.2){
// 				left_wpn = 1;
// 				break;
// 			}
// 			left_wpn = 0;
// 		}
// 		i++;
// 	}
// 	i = 0;
// 	while(i<50){
// 		now = lidar_distance[lidar_size-1-initial-i];
// 		next = lidar_distance[lidar_size-1-initial-i-1];
// 		if(now<1.2 || next<1.2){
// 			if(abs(now-next)> 0.2){
// 				right_wpn = 1;
// 				break;
// 			}
// 			right_wpn = 0;
// 		}
// 		i++;
// 	}

// }

void set_center(){
	float normal;
	if(center_count<30){
		center_x = goal_x*cos(-atan(1/22))-goal_y*sin(-atan(1/22));
		center_y = goal_x*sin(-atan(1/22))+goal_y*cos(-atan(1/22));
		normal = sqrt(center_x*center_x+center_y*center_y);
		center_x = -2.2*center_x/normal;
		center_y = -2.2*center_y/normal;
		center_x = goal_x+center_x;
		center_y = goal_y+center_y;
		center_count++;
	}
}


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
	std::cout<< "inner_prod = " <<inner_prod << std::endl;
	if(abs(1-inner_prod)<0.005){
		return true;
	}else{
		return false;
	}
}

int closest(){
	float tmp;
	int idx = -1;
	float l_lidar_x = 3.5;
	float l_lidar_y = 3.5;
	distance_pillar = 3.5;
	for(int i = 0; i<45; i++){
		tmp = lidar_distance[i];
		l_lidar_x = tmp*cos(lidar_degree[i]);
		l_lidar_y = tmp*sin(lidar_degree[i]);
		tmp = distance_btw(l_lidar_x, l_lidar_y, lB_x, lB_y);
		if(distance_pillar>tmp){
			distance_pillar = tmp;
			idx = i;
			lidar_x = r_pose[0] + l_lidar_x*cos(r_theta)-l_lidar_y*sin(r_theta);
			lidar_y = r_pose[1] + l_lidar_x*sin(r_theta)+l_lidar_y*cos(r_theta);
		}
		
	}
	for(int i = lidar_size-1; i>lidar_size-46; i--){
		tmp = lidar_distance[i];
		l_lidar_x = tmp*cos(lidar_degree[i]);
		l_lidar_y = tmp*sin(lidar_degree[i]);
		tmp = distance_btw(l_lidar_x, l_lidar_y, lB_x, lB_y);
		if(distance_pillar>tmp){
			distance_pillar = tmp;
			idx = i;
			lidar_x = r_pose[0] + l_lidar_x*cos(r_theta)-l_lidar_y*sin(r_theta);
			lidar_y = r_pose[1] + l_lidar_x*sin(r_theta)+l_lidar_y*cos(r_theta);
		}
	}
	return idx;
}

bool is_close(){
	int normal;
	closest_idx = closest();
	normal = 1/distance_btw(lidar_x, lidar_y, gB_x, gB_y);
	std::cout<<"distance btw ball pillar = "<<distance_btw(lidar_x, lidar_y, gB_x, gB_y)<<std::endl;
	if(distance_pillar<0.4){
		target_x = gB_x + 0.5*normal*(gB_x - lidar_x);
		target_y = gB_y + 0.5*normal*(gB_y - lidar_y);
		return true;
	}else{
		return false;
	}
}

void target_closest(){
	float tmp;
	float l_lidar_x = 3.5;
	float l_lidar_y = 3.5;
	distance_target = 3.5;
	for(int i = 0; i<lidar_size; i++){
		tmp = lidar_distance[i];
		l_lidar_x = tmp*cos(lidar_degree[i]);
		l_lidar_y = tmp*sin(lidar_degree[i]);
		tmp = distance_btw(l_lidar_x, l_lidar_y, target_x-r_pose[0], target_y-r_pose[1]);
		if(distance_target>tmp){
			distance_target = tmp;
		}
	}
}

void red_closest(){
	float tmp;
	float l_lidar_x = 3.5;
	float l_lidar_y = 3.5;
	distance_target = 3.5;
	for(int j = 0; j<r_size; j++){
		l_lidar_x = Y_r[j];
		l_lidar_y = -X_r[j];
		tmp = distance_btw(l_lidar_x, l_lidar_y, target_x-r_pose[0], target_y-r_pose[1]);
		if(distance_target>tmp){
			distance_target = tmp;
		}
	}

}

bool is_target_close(){
	int normal;
	bool target_flag = false;
	is_target_close_obstacle = false;
	target_closest();
	if(distance_target<0.5){
		normal = 1/distance_btw(lidar_x, lidar_y, gB_x, gB_y);
		if(distance_btw(r_pose[0], r_pose[1], gB_x + 0.5*normal*(gB_y - lidar_y), gB_y - 0.5*normal*(gB_x - lidar_x))>distance_btw(r_pose[0], r_pose[1], gB_x - 0.5*normal*(gB_y - lidar_y), gB_y + 0.5*normal*(gB_x - lidar_x))){
			target_flag = true;
		}
		if(target_flag){
			target_x = gB_x - 0.5*normal*(gB_y - lidar_y);
			target_y = gB_y + 0.5*normal*(gB_x - lidar_x);
		}else{
			target_x = gB_x + 0.5*normal*(gB_y - lidar_y);
			target_y = gB_y - 0.5*normal*(gB_x - lidar_x);
		}
		target_closest();
		if(distance_target<0.4){
			is_target_close_obstacle = true;
		}
		return true;
	}else{
		return false;
	}
}


bool far_than_ball(){
	float tmp;
	distance_lidar = lidar_distance[0];
	for(int i = 1; i<9; i++){
		tmp = lidar_distance[i];
		if(distance_lidar>tmp){
			distance_lidar = tmp;
		}
		tmp = lidar_distance[lidar_size-i];
		if(distance_lidar>tmp){
			distance_lidar = tmp;
		}
	}
	
	if(distance_lidar>Y[0]){
		return true;
	}
	return false;

}


void camera2local(){
	lB_x = Y[0];
	lB_y = -X[0];
}

void camera2global(){
	gB_x = r_pose[0]+Y[0]*cos(r_theta)+X[0]*sin(r_theta);
	gB_y = r_pose[1]+Y[0]*sin(r_theta)-X[0]*cos(r_theta);
}

void set_vel(float left_f, float right_f, float left_r, float right_r){
	vel_front_left = left_f;
	vel_front_right = right_f;
	vel_rear_left = left_r;
	vel_rear_right = right_r;
}

void rotate_left(){
	set_vel(-45, 45, -45, 45);
}

void rotate_right(){
	set_vel(45, -45, 45, -45);
}

void rotate_left_long_tick(){
	switching_long++;
	if(switching_long<16){
		set_vel(-45, 45, -45, 45);
	}else if(switching_long<21){
		set_vel(25,25,25,25);
	}else{
		switching_long = 0;
	}
}

void rotate_right_long_tick(){
	switching_long++;
	if(switching_long<16){
		set_vel(45, -45, 45, -45);
	}else if(switching_long<21){
		set_vel(25,25,25,25);
	}else{
		switching_long = 0;
	}
}


void rotate_left_slow(){
	set_vel(-50, 50, -20, 20);
}

void rotate_right_slow(){
	set_vel(50, -50, 20,-20);
}


void revolving_left(){
	set_vel(-15, 50, -50, 50);
}

void revolving_right(){
	set_vel(50, -15, 50, -50);
}

void revolving_left_with_ball(){
	set_vel(-20, 100, -100, 100);
}

void revolving_right_with_ball(){
	set_vel(100, -20, 100, -100);
}

void myStop(){
	set_vel(0,0,0,0);
}

void go(){
	start_going++;
	if(start_going<4){
		set_vel(30,30,30,30);
	}else{
		set_vel(15,15,15,15);
	}
}

void go_with_ball(){
	set_vel(25,25,25,25);
}

void rotate_left_tick(){
	if(switching>0){
		myStop();
		switching++;
		if(switching == 10){
			switching = 0;
		}else if(switching == 5){
			set_vel(20,20,20,20);
		}
	}else{
		rotate_left();
		switching++;
	}
}

void rotate_right_tick(){
	if(switching>0){
		myStop();
		switching++;
		if(switching == 10){
			switching = 0;
		}else if(switching == 5){
			set_vel(20,20,20,20);
		}
	}else{
		rotate_right();
		switching++;
	}

}

void rotate_left_tick_fast(){
	if(switching_fast>0){
		myStop();
		switching_fast++;
		if(switching_fast == 5){
			switching_fast = 0;
		}else if(switching_fast == 2){
			set_vel(20,20,20,20);
		}
	}else{
		rotate_left();
		switching_fast++;
	}
}

void rotate_right_tick_fast(){
	if(switching_fast>0){
		myStop();
		switching_fast++;
		if(switching_fast == 5){
			switching_fast = 0;
		}else if(switching_fast == 2){
			set_vel(20,20,20,20);
		}
	}else{
		rotate_right();
		switching_fast++;
	}

}

void rotate_left_tick_very_fast(){
	if(switching_very_fast>0){
		myStop();
		switching_very_fast++;
		if(switching_very_fast == 3){
			switching_very_fast = 0;
		}else if(switching_very_fast == 1){
			set_vel(20,20,20,20);
		}
	}else{
		rotate_left();
		switching_very_fast++;
	}
}

void rotate_right_tick_very_fast(){
	if(switching_very_fast>0){
		myStop();
		switching_very_fast++;
		if(switching_very_fast == 3){
			switching_very_fast = 0;
		}else if(switching_very_fast == 1){
			set_vel(20,20,20,20);
		}
	}else{
		rotate_right();
		switching_very_fast++;
	}

}

void rotate_left_tick_middle(){
	if(switching_middle>0){
		myStop();
		switching_middle++;
		if(switching_middle == 7){
			switching_middle = 0;
		}else if(switching_middle == 4){
			set_vel(20,20,20,20);
		}
	}else{
		rotate_left();
		switching_middle++;
	}
}

void rotate_right_tick_middle(){
	if(switching_middle>0){
		myStop();
		switching_middle++;
		if(switching_middle == 7){
			switching_middle = 0;
		}else if(switching_middle == 4){
			set_vel(20,20,20,20);
		}
	}else{
		rotate_right();
		switching_middle++;
	}

}

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
	detect_left2 = false;
	detect_right2 = false;
	detect_front2 = false;
	detect_front3 = false;
	map_mutex.lock();

	int count = scan->angle_max / scan->angle_increment;
    lidar_size=count;
	// std::cout << "-------------------------------------------------------"<<std::endl;
    for(int i = 0; i < count; i++)
    {	
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
		if(isinf(lidar_distance[i])){
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

	if(front<0.4){
		detect_front3 = true;
	}

	if(front<0.2){
		detect_front2 = true;
	}
	if(behind < 0.35){
		detect_behind =  true;
	}
	if(left<0.137){
		detect_left = true;
	}
	if(right<0.137){
		detect_right = true;
	}
	if(left<0.25){
		detect_left2 = true;
	}
	if(right<0.25){
		detect_right2 = true;
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

void r_camera2_Callback(const core_msgs::ball_position::ConstPtr& position)
{
		start = true;
		r_size2 = position->size;
        if(r_size2 != 0){
			detect_r2 = true;
			for(int i = 0; i<r_size2; i++){
				X_r2[i] = position->img_x[i];
        		Y_r2[i] = position->img_y[i];
			}

			// std::cout << "receive red ball"<<std::endl;
		}else{
			// std::cout << "red ball not detect"<<std::endl;
			detect_r2 = false;
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

void b_camera2_Callback(const core_msgs::ball_position::ConstPtr& position)
{		
		int max_i = 0;
		start = true;
		b_size2 = position->size;
		if(b_size2 !=0){
			detect_b2 = true;
			for(int i = 0; i<b_size2; i++){
				X2_tmp[i] = position->img_x[i];
        		Y2_tmp[i] = position->img_y[i];
				inverse_dist2[i] = 1/(X2_tmp[i]*X2_tmp[i] + Y2_tmp[i]*Y2_tmp[i]);
			}
			for(int j = 0; j<b_size2; j++){
				for(int i = 0; i<b_size2; i++){
					if(inverse_dist2[max_i]<inverse_dist2[i]){
						max_i = i;
					}
				}
				X2[j] = X2_tmp[max_i];
				Y2[j] = Y2_tmp[max_i];
				inverse_dist2[max_i] = 0;
			}
			// std::cout << "receive blue ball"<<std::endl;
		}else{
			// std::cout << "blue ball not detect"<<std::endl;
			detect_b2 = false;
		}
	
}


int main(int argc, char **argv)
{	
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position_b2", 1000, b_camera_Callback);
	ros::Subscriber sub2 = n.subscribe<core_msgs::ball_position>("/position_r2", 1000, r_camera_Callback);
	ros::Subscriber sub3 = n.subscribe<core_msgs::ball_position>("/position_g2", 1000, g_camera_Callback);
	ros::Subscriber sub1_2 = n.subscribe<core_msgs::ball_position>("/position_b", 1000, b_camera2_Callback);
	ros::Subscriber sub2_2 = n.subscribe<core_msgs::ball_position>("/position_r", 1000, r_camera2_Callback);
	ros::Subscriber sub5 = n.subscribe<core_msgs::robot_position>("/robot_position", 1000, global_Callback);
	ros::Publisher pub_left_front_wheel= n.advertise<std_msgs::Float64>("/model20/left_front_wheel_velocity_controller/command", 10);
	ros::Publisher pub_right_front_wheel= n.advertise<std_msgs::Float64>("/model20/right_front_wheel_velocity_controller/command", 10);
	ros::Publisher pub_left_rear_wheel= n.advertise<std_msgs::Float64>("/model20/left_rear_wheel_velocity_controller/command", 10);
	ros::Publisher pub_right_rear_wheel= n.advertise<std_msgs::Float64>("/model20/right_rear_wheel_velocity_controller/command", 10);

	ros::Publisher sus1 = n.advertise<std_msgs::Float64>("/model20/left_front_wheel_suspension_controller/command", 10);
	ros::Publisher sus2 = n.advertise<std_msgs::Float64>("/model20/right_front_wheel_suspension_controller/command", 10);
	ros::Publisher sus3 = n.advertise<std_msgs::Float64>("/model20/left_rear_wheel_suspension_controller/command", 10);
	ros::Publisher sus4 = n.advertise<std_msgs::Float64>("/model20/right_rear_wheel_suspension_controller/command", 10);

    while(ros::ok){
		
			std_msgs::Float64 left_front_wheel_msg;
			std_msgs::Float64 right_front_wheel_msg;
			std_msgs::Float64 left_rear_wheel_msg;
			std_msgs::Float64 right_rear_wheel_msg;
			std_msgs::Float64 suspension_msg;

			//center of x =0
			if(start && isenabled){
				if(mode == -1){
					if(sub_mode == 0){
						if(detect_g && X_g>0){
							sub_mode = 1;
						}else{
							rotate_left_tick_fast();
							std::cout << "mode : -1, rotate"<<std::endl;
						}
					}else if(sub_mode == 1){
						c++;
						if(c<35){
							set_vel(25,25,25,25);
						}else{
							c = 0;
							sub_mode = 2;
						}
					}else if(sub_mode == 2){
						c ++;
						if(c<10){
							rotate_left();
							std::cout << "mode : -1, rotate"<<std::endl;
						}else{
							c = 0;
							sub_mode = 0;
							mode = 0;
						}
					}
				}
				if(detect_g){
					goal_x = r_pose[0]+Y_g*cos(r_theta)+X_g*sin(r_theta);
					goal_y = r_pose[1]+Y_g*sin(r_theta)-X_g*cos(r_theta);
					set_center();
				}
				if(detect_b || detect_b2){
					if(mode == 0){ // mode 0 : rotate in place for matching the direction to the ball
						if((detect_b && X[0]<0) || (detect_b2 && X2[0]<0)){ // if the ball is at the right side
							if((detect_b && X[0]>-0.008) || (detect_b2 && X2[0]>-0.008)){
								mode = 1;
								myStop();
								c1=0;
								c2=0;
								c = 0;
								start_going = 0;
							}else{
								std::cout << "mode : 0, positive"<<std::endl;
								rotate_left_tick();
								if(!is_pos){
									c++;
									cont_count =0;
								}
								is_pos = true;
								cont_count++;
								if(cont_count>200){
									mode = 5;
									sub_mode = 3;
									myStop();
									c1=0;
									c2=0;
									c = 0;
									start_going = 0;
									cont_count = 0;
								}
							}
						}else if((detect_b && X[0]>0) || (detect_b2 && X2[0]>0)){ // if the ball is at the left side
							if((detect_b && X[0]<0.008) || (detect_b2 && X2[0]<0.008)){
								mode = 1;
								myStop();
								c1=0;
								c2=0;
								c = 0;
								start_going = 0;
							}else{
								std::cout << "mode : 0, negative"<<std::endl;
								rotate_right_tick();
								if(is_pos){
									c++;
									cont_count = 0;
								}
								is_pos = false;
								if(cont_count>200){
									mode = 5;
									sub_mode = 3;
									myStop();
									c1=0;
									c2=0;
									c = 0;
									start_going = 0;
									cont_count = 0;
								}
							}
						}
						if(c > 40){
							mode = 1;
							myStop();
							c1=0;
							c2=0;
							c = 0;
							start_going = 0;
						}
					}else if(mode == 1){ // mode 1 : go straight
						if(detect_b2 && Y2[0]<0.15){
							std::cout << "mode : 1, stop"<<std::endl;
							myStop();
							flag =1;
							mode  = 5;
							c = 0;
							rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
							start_going = 0;
						}else{
							if(flag && Y[0]<1){
								start_going = 0;
								camera2local();
								camera2global();
								if(!far_than_ball()){
									if(is_close()){
										if(breaking<3){
											breaking++;
											set_vel(-20,-20,-20,-20);
										}else if(breaking <25){
											breaking++;
											myStop();
										}else{
											if(is_target_close()){
												std::cout << "distance_target : "<< distance_target<<std::endl;
												if(is_target_close_obstacle){
													mode = 21;
													myStop();
													rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
													breaking = 0;
												}else{
													mode = 20;
													myStop();
													rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
													breaking = 0;
												}
											}else{
												std::cout << "distance_target : "<< distance_target<<std::endl;
												mode = 16;
												myStop();
												rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
												breaking = 0;
											}
										}
									}else{
										if(breaking<3){
											breaking++;
											set_vel(-20,-20,-20,-20);
										}else if(breaking <25){
											breaking++;
											myStop();
										}
										else{
											mode = 0;
											flag = 0;
											myStop();
											breaking = 0;
										}
									}

								}else{
									if(breaking<3){
										breaking++;
										set_vel(-20,-20,-20,-20);
									}else if(breaking <25){
										breaking++;
										myStop();
									}
									else{
										mode = 20;
										sub_mode = 4;
										start_going = 0;
										breaking = 0;
										if(is_close()){
											c = 0;
										}else{
											c = 50;
										}
									}
								}
								
							}else{
								std::cout << "mode : 1, go"<<std::endl;
								go();
								if((detect_b && abs(X[0])>0.06) || (detect_b2 && abs(X2[0]>0.06))){
									c = 0;
									start_going=0;
									myStop();
									mode = 0;
								}
							}
						}
					}

				}
				else if(mode < 2 && mode != -1){ // when the blue ball is not detect
					if(c1<350){
						c1++;
						std::cout << "detect mode, rotate"<<std::endl;
						rotate_left_slow();
						mode = 0;
					}else{
						mode = 17;
						sub_mode = 0;
						c1 = 0;
						rot_left = is_turn_left(r_theta, center_x-r_pose[0], center_y-r_pose[1]);
						if(ball_count>2){
							mode = 9;
							isFinish = true;
						}
					}
				}
				
				if(mode == 17){
					if(sub_mode == 0){
						if(abs(1-(DiffandNorm(center_x, center_y, r_pose[0], r_pose[1],0)*cos(r_theta)+DiffandNorm(center_x, center_y, r_pose[0], r_pose[1], 1)*sin(r_theta)))<0.01){
							sub_mode =1;
							myStop();
							c = 0;
						}else{
							std::cout << "mode 17, go to center(direction matching)"<<std::endl;
							if(rot_left){
								rotate_left_tick_fast();
							}else{
								rotate_right_tick_fast();
							}
						}
					}else if(sub_mode == 1){
						if(distance_btw(r_pose[0], r_pose[1], center_x, center_y)<0.2){
							sub_mode = 0;
							myStop();
							mode = 0;
							c = 0;
							start_going = 0;
						}else{
							std::cout << "mode 17, go to center(distance matching)"<<std::endl;
							go();
							c++;
							if(c>150){
								sub_mode = 0;
								myStop();
								mode = 0;
								c = 0;
								start_going = 0;
							}
						}
					}

				}else if(mode == 21){
					if(sub_mode == 0){
						if(is_matched()){
							sub_mode =1;
							myStop();
							start_going = 0;
						}else{
							std::cout << "mode 21, direction matching"<<std::endl;
							if(rot_left){
								rotate_left_tick_fast();
							}else{
								rotate_right_tick_fast();
							}
						}
					}else if(sub_mode == 1){
						std::cout<<"l_x = "<<target_x-r_pose[0] <<", l_y = "<<target_y-r_pose[1]<<std::endl;
						if(distance_btw(r_pose[0], r_pose[1], target_x, target_y)<0.15 || flag_21 || detect_front3){
							flag_21 = true;
							if(breaking<3){
								breaking++;
								set_vel(-20,-20,-20,-20);
							}else if(breaking <25){
								breaking++;
								myStop();
							}
							else{
								start_going = 0;
								breaking = 0;
								sub_mode = 2;
								myStop();
								rot_left = is_turn_left(r_theta, gB_x-r_pose[0], gB_y-r_pose[1]);
								flag_21 = false;
							}
						}else{
							std::cout << "mode 21, go straight"<<std::endl;
							go();
						}
					}else if(sub_mode == 2){
						if(abs(1-(DiffandNorm(gB_x, gB_y,r_pose[0],r_pose[1], 0)*cos(r_theta)+DiffandNorm(gB_x, gB_y,r_pose[0],r_pose[1], 1)*sin(r_theta)))<0.009){
							sub_mode = 3;
							myStop();
						}else{
							std::cout << "mode 21, rotate"<<std::endl;
							if(rot_left){
								rotate_left_slow();
							}else{
								rotate_right_slow();
							}
						}
					}else if(sub_mode == 3){
						if((detect_b && X[0]<0) || (detect_b2 && X2[0]<0)){ // if the ball is at the right side
							if((detect_b && X[0]> -0.005) || (detect_b2 && X2[0]>-0.01)){
								sub_mode = 4;
								myStop();
							}else{
								std::cout << "mode : 21, positive"<<std::endl;
								rotate_left_tick();
							}
						}else if((detect_b && X[0]>0) || (detect_b2 && X2[0]>0)){ // if the ball is at the left side
							if((detect_b && X[0]<0.005) || (detect_b2 && X2[0]<0.01)){
								sub_mode = 4;
								myStop();
							}else{
								std::cout << "mode : 21, negative"<<std::endl;
								rotate_right_tick();
							}
						}
					}else if(sub_mode == 4){
						if((detect_b && Y[0]<0.15) || (detect_b2 && Y2[0]<0.15)){
							std::cout << "mode : 21, stop"<<std::endl;
							start_going = 0;
							myStop();
							sub_mode = 5;
							c = 0;
						}else{
							std::cout << "mode : 21, go"<<std::endl;
							go();
							if((detect_b && abs(X[0])>0.01 && Y[0]>0.23) || (detect_b2 && abs(X2[0])>0.01 && Y[0]>0.23)){
								sub_mode = 3;
								start_going = 0;
							}
						}
					}else if(sub_mode == 5){
						c++;
						if(c<50){
							std::cout << "mode : 21, go with ball"<<std::endl;
							go_with_ball();
						}else{
							std::cout << "mode : 21, stop"<<std::endl;
							start_going = 0;
							myStop();
							mode = 5;
							sub_mode = 0;
							flag =1;
							rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
							c = 0;
						}
					}					
				}else if(mode == 20){
					if(sub_mode == 0){
						if(is_matched()){
							sub_mode =1;
							myStop();
							start_going = 0;
						}else{
							std::cout << "mode 20, direction matching"<<std::endl;
							if(rot_left){
								rotate_left_tick_fast();
							}else{
								rotate_right_tick_fast();
							}
						}
					}else if(sub_mode == 1){
						std::cout<<"l_x = "<<target_x-r_pose[0] <<", l_y = "<<target_y-r_pose[1]<<std::endl;
						if(distance_btw(r_pose[0], r_pose[1], target_x, target_y)<0.15 || flag_20){
							flag_20 = true;
							if(breaking<3){
								breaking++;
								set_vel(-20,-20,-20,-20);
							}else if(breaking <25){
								breaking++;
								myStop();
							}
							else{
								start_going = 0;
								breaking = 0;
								sub_mode = 2;
								myStop();
								rot_left = is_turn_left(r_theta, gB_x-r_pose[0], gB_y-r_pose[1]);
								flag_20 = false;
							}
						}else{
							std::cout << "mode 20, go straight"<<std::endl;
							go();
						}
					}else if(sub_mode == 2){
						if(abs(1-(DiffandNorm(gB_x, gB_y,r_pose[0],r_pose[1], 0)*cos(r_theta)+DiffandNorm(gB_x, gB_y,r_pose[0],r_pose[1], 1)*sin(r_theta)))<0.009){
							sub_mode = 3;
							myStop();
						}else{
							std::cout << "mode 20, rotate"<<std::endl;
							if(rot_left){
								rotate_left_slow();
							}else{
								rotate_right_slow();
							}
						}
					}else if(sub_mode == 3){
						if((detect_b && X[0]<0) || (detect_b2 && X2[0]<0)){ // if the ball is at the right side
							if((detect_b && X[0]> -0.005) || (detect_b2 && X2[0]>-0.01)){
								sub_mode = 4;
								myStop();
							}else{
								std::cout << "mode : 20, positive"<<std::endl;
								rotate_left_tick();
							}
						}else if((detect_b && X[0]>0) || (detect_b2 && X2[0]>0)){ // if the ball is at the left side
							if((detect_b && X[0]<0.005) || (detect_b2 && X2[0]<0.01)){
								sub_mode = 4;
								myStop();
							}else{
								std::cout << "mode : 20, negative"<<std::endl;
								rotate_right_tick();
							}
						}else{
							start_going = 0;
							myStop();
							mode = 0;
							sub_mode = 0;
							flag =1;
							rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
						}
					}else if(sub_mode == 4){
						if((detect_b && Y[0]<0.15) || (detect_b2 && Y2[0]<0.15)){
							std::cout << "mode : 20, stop"<<std::endl;
							start_going = 0;
							myStop();
							sub_mode = 5;
						}else{
							std::cout << "mode : 20, go"<<std::endl;
							go();
							if((detect_b && abs(X[0])>0.01 && Y[0]>0.23) || (detect_b2 && abs(X2[0])>0.01 && Y[0]>0.23)){
								sub_mode = 3;
								start_going = 0;
							}
						}
					}else if(sub_mode == 5){
						c++;
						if(c<50){
							std::cout << "mode : 20, go with ball"<<std::endl;
							go_with_ball();
						}else{
							std::cout << "mode : 20, stop"<<std::endl;
							start_going = 0;
							myStop();
							mode = 5;
							sub_mode = 0;
							flag =1;
							rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
							c = 0;
						}
					}
				}

				if(mode == 16){
					if(sub_mode == 0){
						if(is_matched()){
							sub_mode =1;
							myStop();
							start_going = 0;
						}else{
							std::cout << "mode 16, direction matching"<<std::endl;
							if(rot_left){
								rotate_left_tick_fast();
							}else{
								rotate_right_tick_fast();
							}
						}
					}else if(sub_mode == 1){
						std::cout<<"l_x = "<<target_x-r_pose[0] <<", l_y = "<<target_y-r_pose[1]<<std::endl;
						if(distance_btw(r_pose[0], r_pose[1], target_x, target_y)<0.12 || flag_16){
							flag_16 = true;
							if(breaking<3){
								breaking++;
								set_vel(-20,-20,-20,-20);
							}else if(breaking <25){
								breaking++;
								myStop();
							}
							else{
								start_going = 0;
								breaking = 0;
								sub_mode = 2;
								myStop();
								rot_left = is_turn_left(r_theta, gB_x-r_pose[0], gB_y-r_pose[1]);
								flag_16 = false;
								mode16_count = 0;
							}
						}else{
							std::cout << "mode 16, go straight"<<std::endl;
							go();
						}
					}else if(sub_mode == 2){
						if(abs(1-(DiffandNorm(gB_x, gB_y,r_pose[0],r_pose[1], 0)*cos(r_theta)+DiffandNorm(gB_x, gB_y,r_pose[0],r_pose[1], 1)*sin(r_theta)))<0.015){
							sub_mode = 3;
							myStop();
						}else{
							std::cout << "mode 16, rotate"<<std::endl;
							if(rot_left){
								rotate_left_slow();
							}else{
								rotate_right_slow();
							}
						}
					}else if(sub_mode == 3){
						if((detect_b && X[0]<0) || (detect_b2 && X2[0]<0)){ // if the ball is at the right side
							if((detect_b && X[0]> -0.007) || (detect_b2 && X2[0]>-0.007)){
								sub_mode = 4;
								myStop();
							}else{
								std::cout << "mode : 16, positive"<<std::endl;
								rotate_left_tick();
							}
						}else if((detect_b && X[0]>0) || (detect_b2 && X2[0]>0)){ // if the ball is at the left side
							if((detect_b && X[0]<0.007) || (detect_b2 && X2[0]<0.007)){
								sub_mode = 4;
								myStop();
							}else{
								std::cout << "mode : 16, negative"<<std::endl;
								rotate_right_tick();
							}
						}
					}else if(sub_mode == 4){
						if((detect_b && Y[0]<0.15) || (detect_b2 && Y2[0]<0.15)){
							std::cout << "mode : 16, stop"<<std::endl;
							start_going = 0;
							myStop();
							mode = 5;
							sub_mode = 0;
							flag =1;
							rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
							c = 0;
						}else{
							std::cout << "mode : 16, go"<<std::endl;
							c++;
							if(c<497){
								go();
								rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
							}else if(c<500){
								if(rot_left){
									rotate_left();
								}else{
									rotate_right();
								}
							}else{
								c = 0;
							}
							if((detect_b && abs(X[0])>0.06) || (detect_b2 && abs(X2[0]>0.06))){
								sub_mode = 3;
								c = 0;
								start_going=0;
								myStop();
							}
						}
					}
				}

				if(mode == 2){ // mode 2 : avoid action - rotate
					if(sub_mode == 0){
						if(detect_b2 && Y2[0]<0.25){
							if(c<12){ // until the red ball is not detected on the camera, rotate.
								std::cout << "mode : 2, direction avoid, red ball"<<std::endl;
								c++;
								if(rev_dir == 0){
									rotate_right();
								}else{
									rotate_left();
								}
							}else{
								c++;
								if(c<30){
									myStop();
								}else if(detect_b2 && abs(X2[0])<0.05 && Y2[0]<0.5){
									mode = 3;
									c = 0;
									sub_mode = 0;
								}else{
									if(c<40){
										set_vel(-20,-20,-20,-20);
									}else{
										mode = 0;
										sub_mode = 0;
										c = 0;
										flag_m2 = false;
									}
								}
							}
						}else{
							if(c<12){ // until the red ball is not detected on the camera, rotate.
								std::cout << "mode : 2, direction avoid, red ball"<<std::endl;
								c++;
								if(rev_dir == 0){
									rotate_right();
								}else{
									rotate_left();
								}
							}else{
								mode = 3;
								c = 0;
								sub_mode = 0;
								myStop();
							}
						}
					}
				}else if(mode == 3){ // mode 3 : avoid action - revolving around
					if(detect_b2 && Y2[0]<0.25){
						std::cout << "mode : 3, go little"<<std::endl;
						set_vel(30,30,30,30);
						if(c == 15){
							flag_m2 = false;
						}
						if(c < 27){
							c++;
						}else{
							if(breaking<1){
								breaking++;
								set_vel(-20,-20,-20,-20);
							}else if(breaking <25){
								breaking++;
								myStop();
							}
							else{
								mode = 0;
								flag = 1;
								myStop();
								breaking = 0;
								c = 0;
								if(store_mode == 16){
									mode = 16;
									rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
								}
								else if(store_mode == 6){
									mode = 5;
									sub_mode = 0;
									rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
								}
							}
						}
					}else{
						std::cout << "mode : 3, revolving"<<std::endl;
						if(rev_dir == 0){
							revolving_left();
						}else{
							revolving_right();
						}
						if(c == 15){
							flag_m2 = false;
						}
						if(c < 35){
							c++;
						}else{
							if(breaking<9){
								breaking++;
								set_vel(-20,-20,-20,-20);
							}else if(breaking <25){
								breaking++;
								myStop();
							}
							else{
								mode = 0;
								flag = 1;
								myStop();
								breaking = 0;
								c = 0;
								if(store_mode == 16){
									mode = 16;
									rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
								}
								else if(store_mode == 6){
								mode = 5;
								sub_mode = 0;
								rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
								}
							}
						}
					}
				}else if(mode == 10){
					if(sub_mode == 0){
						if(detect_b2 && Y2[0]<0.25){
							if(c<12){ // until the red ball is not detected on the camera, rotate.
								std::cout << "mode : 10, direction avoid, pillar"<<std::endl;
								c++;
								if(rev_dir == 0){
									rotate_right();
								}else{
									rotate_left();
								}
							}else{
								c++;
								if(c<30){
									myStop();
								}else if(detect_b2 && abs(X2[0])<0.05 && Y2[0]<0.5){
									c = 0;
									sub_mode = 1;
								}else{
									if(c<40){
										set_vel(-20,-20,-20,-20);
									}else{
										mode = 0;
										sub_mode = 0;
										c = 0;
										flag_m2 = false;
									}
								}
							}

						}else{
							if(c<12){ // until the red ball is not detected on the camera, rotate.
								std::cout << "mode : 10, direction avoid, pillar"<<std::endl;
								c++;
								if(rev_dir == 0){
									rotate_right();
								}else{
									rotate_left();
								}
							}else{
								myStop();
								c = 0;
								sub_mode = 1;
							}
						}
					}else if(sub_mode == 1){
						if(detect_b2 && Y2[0]<0.25){
							std::cout << "mode : 10, go little"<<std::endl;
							set_vel(30,30,30,30);
							if(c == 15){
								flag_m2 = false;
							}
							if(c < 27){
								c++;
							}else{
								if(breaking<1){
									breaking++;
									set_vel(-20,-20,-20,-20);
								}else if(breaking <25){
									breaking++;
									myStop();
								}else{
									c = 0;
									mode = 0;
									sub_mode = 0;
									flag = 1;
									breaking = 0;
									myStop();
									if(store_mode == 16){
										mode = 16;
										rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
									}
									else if(store_mode == 6){
										mode = 5;
										sub_mode = 0;
										rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
									}
								}
							}
						}else{
							std::cout << "mode : 10, go little"<<std::endl;
							if(rev_dir == 0){
								//revolving_left();
								set_vel(25,25,25,25);
							}else{
								//revolving_right();
								set_vel(25,25,25,25);
							}
							if(c == 15){
								flag_m2 = false;
							}
							if(c < 35){
								c++;
							}else{
								if(breaking<9){
									breaking++;
									set_vel(-20,-20,-20,-20);
								}else if(breaking <25){
									breaking++;
									myStop();
								}else{
									c = 0;
									mode = 0;
									sub_mode = 0;
									flag = 1;
									breaking = 0;
									myStop();
									if(store_mode == 16){
										mode = 16;
										rot_left = is_turn_left(r_theta, target_x-r_pose[0], target_y-r_pose[1]);
									}
									else if(store_mode == 6){
									mode = 5;
									sub_mode = 0;
									rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
									}
								}
							}
						}
					}
				}else if(mode == 11){

					c++;
					if(c<50){
						std::cout << "mode : 11, go"<<std::endl;
						go_with_ball();
					}else{
						c = 0;
						mode = 0;
						sub_mode = 0;
						myStop();
						flag_m11 = false;
					}

				}else if(mode == 12){
					c++;
					if(c<18){
						std::cout << "mode : 12, avoid red ball"<<std::endl;
						if(rev_dir){
							rotate_left();
						}else{
							rotate_right();
						}
					}else if(c<37){
						std::cout << "mode : 12, back"<<std::endl;
						set_vel(-20,-20,-20,-20);
					}else{
						c = 0;
						flag_m12 = false;
						mode = 0;
						sub_mode = 0;
						myStop();
					}

				}else if(mode == 6){ //go to the goal.
					if(sub_mode == 0){
						if(detect_g){
							if(Y_g<0.8){
								if(breaking<3){
									breaking++;
									set_vel(-20,-20,-20,-20);
								}else if(breaking <25){
									breaking++;
									myStop();
								}
								else{
									breaking = 0;
									sub_mode = 1;
									myStop();
									start_going = 0;
								}
							}else{
								std::cout << "mode : 6, go from far"<<std::endl;
								go_with_ball();
							}
						}else{
							if(distance_btw(r_pose[0], r_pose[1], goal_x, goal_y)<0.9){
								if(breaking<3){
									breaking++;
									set_vel(-20,-20,-20,-20);
								}else if(breaking <25){
									breaking++;
									myStop();
								}
								else{
									breaking = 0;
									sub_mode = 1;
									myStop();
									start_going = 0;
								}
							}else{
								std::cout << "mode : 6, go from far"<<std::endl;
								go_with_ball();
							}
						}
					}else if(sub_mode == 1){
						if(X_g<0){ // if the ball is at the right side
							if(X_g>-0.008){
								sub_mode = 2;
								myStop();
							}else{
								std::cout << "mode : 6, positive"<<std::endl;
								rotate_left_tick();
							}
						}else{ // if the ball is at the left side
							if(X_g<0.008){
								sub_mode = 2;
								myStop();
							}else{
								std::cout << "mode : 6, negative"<<std::endl;
								rotate_right_tick();
							}
						}
					}else if(sub_mode == 2){
						if(Y_g <0.19){ // if distance is smaller than 0.18, go to mode7(blue ball goal in)
							if(breaking<2){
								breaking++;
								set_vel(-20,-20,-20,-20);
							}else if(breaking <15){
								breaking++;
								myStop();
							}else{
								myStop();
								c = 0;
								mode = 7;
								sub_mode = 0;
								breaking = 0;
								start_going = 0;
							}
						}else{
							if(!detect_g){
								if(c<100){
									c++;
								}else{
									myStop();
									c = 0;
									mode = 7;
									sub_mode = 0;
									start_going = 0;
								}
							}else if(Y_g<0.35){
								if(c<130){
									c++;
									std::cout << "mode : 6, go"<<std::endl;
									go_with_ball();
								}else{
									if(c<135){
										c++;
										std::cout << "mode : 6, back little"<<std::endl;
										set_vel(-20,-20,-20,-20);
									}else{
										if(X_g<0){ // if the ball is at the right side
											if(X_g>-0.002){
												c = 0;
												myStop();
											}else{
												std::cout << "mode : 6, positive"<<std::endl;
												rotate_left_tick_fast();
											}
										}else{ // if the ball is at the left side
											if(X_g<0.002){
												c = 0;
												myStop();
											}else{
												std::cout << "mode : 6, negative"<<std::endl;
												rotate_right_tick_fast();
											}
										}
									}
								}
							}
							else{
								c = 0;
								std::cout << "mode : 6, go"<<std::endl;
								go_with_ball();
							}
						}
					}
				}

				if((((mode < 4 || (mode == 6 && sub_mode !=1 && sub_mode != 2) || mode == 10 || (mode == 16 && sub_mode <2) || mode == 17 || (mode == 18 && sub_mode == 2)) && (((detect_r && Y_r[0]<0.34 && abs(X_r[0])<0.25) || (detect_r2 && Y_r2[0]<0.34 && abs(X_r2[0]<0.25)))|| detect_front || detect_fl || detect_fr)) || (mode == 5 && (detect_fl || detect_fr))) && (!detect_g || (detect_g && Y_g >0.6)) && !flag_m2){ // interrupt if the red ball is in front of the robot without blue ball -> go to mode 2
					sub_mode = 0;
					breaking = 0;
					start_going = 0;
					flag_16 = false;
					flag_20 = false;
					flag_m20 = false;
					flag = 1;
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

						mode = 10;
						flag_m2 = true;
						c = 0;
					}else{
						if(detect_r){
							if(X_r[0]>0){
								rev_dir = 1;
							}else{
								rev_dir = 0;
							}
							mode = 2;
							flag_m2 = true;
							c = 0;
						}else if(detect_r2){
							if(X_r2[0]>0){
								rev_dir = 1;
							}else{
								rev_dir = 0;
							}
							mode = 2;
							flag_m2 = true;
							c = 0;
						}
					}
				}

				if(((detect_r&& abs(X_r[0])<0.05 && Y_r[0]<0.23) || (detect_r2 && abs(X_r2[0])<0.05 && Y_r2[0]<0.23) && !flag_m12) || detect_front2 ){
					mode = 5;
					sub_mode = 3;
					c = 0;
					flag_m2 = false;
					start_going = 0;
					breaking = 0;
					c1=0;
					c2=0;
					cont_count = 0;
					vibration = 0;
					flag = 1;
					flag_m20 = false;
					mode16_count = 0;
					if((detect_r&& abs(X_r[0])<0.05 && Y_r[0]<0.23) || (detect_r2 && abs(X_r2[0])<0.05 && Y_r2[0]<0.23)){
						mode = 12;
						sub_mode = 0;
						flag_m12 = true;
						rev_dir = rand()%2;
					}
				}

				if(((detect_left || detect_right) || (mode == 5 && (detect_left2 || detect_right2))) && !flag_m11 ){
					mode = 11;
					sub_mode = 0;
					c = 0;
					flag_m2 = false;
					start_going = 0;
					breaking = 0;
					c1=0;
					c2=0;
					cont_count = 0;
					vibration = 0;
					flag = 1;
					flag_m20 = false;
					mode16_count = 0;
					flag_21 = 0;
					if(detect_left2){
						rev_dir = 1;
					}else{
						rev_dir = 0;
					}
					flag_m11 = true;
				}

				if(mode == 5){//direction matching
					if(sub_mode == 0){
						std::cout << "mode : 5, go little"<<std::endl;
						if(c<2){
							c++;
							set_vel(10,10,10,10);
						}else{
							c = 0;
							sub_mode = 1;
							myStop();
						}
					}else if(sub_mode == 1){
						std::cout << "mode : 5, direction matching with goal"<<std::endl;
						if(detect_g){
							if(abs(X_g<0.15)){
								sub_mode = 2;
								c = 0;
								myStop();
							}else{
								if(rot_left){
									rotate_left_long_tick();
								}else{
									rotate_right_long_tick();
								}
							}
							
						}else{
							if(abs(1-(DiffandNorm(goal_x, goal_y,r_pose[0],r_pose[1], 0)*cos(r_theta)+DiffandNorm(goal_x, goal_y,r_pose[0],r_pose[1], 1)*sin(r_theta)))<0.08){
								sub_mode = 2;
								c = 0;
								myStop();
							}else{
								if(rot_left){
									rotate_left_long_tick();
								}else{
									rotate_right_long_tick();
								}
							}

						}

					}else if(sub_mode == 2){
						std::cout << "mode : 5, direction matching with goal"<<std::endl;
						if(detect_g){
							if(X_g<0){ // if the ball is at the right side
								if(X_g> -0.01){
									if(detect_b2 && Y2[0]<0.23 && abs(X2[0]<0.05)){
										mode = 6;
										sub_mode = 0;
										myStop();
										c = 0;
									}else{
										sub_mode = 3;
										myStop();
										c = 0;
									}
								}else{
									std::cout << "mode : 5, positive"<<std::endl;
									rotate_left_tick();
								}
							}else{ // if the ball is at the left side
								if(X_g<0.01){
									if(detect_b2 && Y2[0]<0.23 && abs(X2[0]<0.05)){
										mode = 6;
										sub_mode = 0;
										myStop();
										c = 0;
									}else{
										sub_mode = 3;
										myStop();
										c = 0;
									}
								}else{
									std::cout << "mode : 5, negative"<<std::endl;
									rotate_right_tick();
								}
							}
						}else{
							if(abs(1-(DiffandNorm(goal_x, goal_y,r_pose[0],r_pose[1], 0)*cos(r_theta)+DiffandNorm(goal_x, goal_y,r_pose[0],r_pose[1], 1)*sin(r_theta)))<0.005){
								if(detect_b2 && Y2[0]<0.23 && abs(X2[0]<0.05)){
									mode = 6;
									sub_mode = 0;
									myStop();
									c = 0;
								}else{
									sub_mode = 3;
									myStop();
									c = 0;
								}
							}else{
								rot_left = is_turn_left(r_theta, goal_x-r_pose[0], goal_y-r_pose[1]);
								if(rot_left){
									rotate_left_tick();
								}else{
									rotate_right_tick();
								}
							}
						}
					}else if(sub_mode == 3){
						c++;
						if(c<12){
							set_vel(-20,-20,-20,-20);
						}else if(c<24){
							myStop();
						}else{
							mode = 0;
							sub_mode = 0;
							myStop();
							c = 0;
							flag_m2 = false;
						}

					}
				}
				if(mode == 7){ // rotate 180 degree to find another ball
					if(sub_mode == 0){
						std::cout << "mode : 7, back"<<std::endl;
						if(Y_g<0.4){
							c++;
							set_vel(-20,-20,-20,-20);
						}else{
							c = 0;
							myStop();
							sub_mode = 1;
						}
					}else if(sub_mode == 1){
						std::cout << "mode : 7, rotate 180"<<std::endl;
						if(c<40){
							c++;
							if(rev_dir == 0){
								rotate_left();
							}else{
								rotate_right();
							}
						}else{
							c=0;
							myStop();
							sub_mode = 0;
							mode = 0;
							ball_count++;
							if(ball_count >2){
								mode = 9;
								isFinish = true;
							}
						}
					}
				}else if(mode == 9){
					if(c<200){
						std::cout << "finish??"<<std::endl;
						c++;
						if(rev_dir == 0){
							rotate_left_slow();
						}else{
							rotate_right_slow();
						}
						if(detect_b || detect_b2){
							isFinish = false;
						}
						if(!isFinish){
							c = 0;
							isFinish = true;
							mode = 0;
						}
					}else{
						std::cout << "finish!!"<<std::endl;	
						myStop();
					}
				}
			
			std::cout << "isenabled! " << vel_front_left << " " << vel_front_right << std::endl;
			left_front_wheel_msg.data=vel_front_left;   // set left_wheel velocity
			right_front_wheel_msg.data=vel_front_right;  // set right_wheel velocity
			left_rear_wheel_msg.data=vel_rear_left;
			right_rear_wheel_msg.data=vel_rear_right;

			pub_left_front_wheel.publish(left_front_wheel_msg);   // publish left_wheel velocity
			pub_right_front_wheel.publish(right_front_wheel_msg);  // publish right_wheel velocity
			pub_left_rear_wheel.publish(left_rear_wheel_msg);
			pub_right_rear_wheel.publish(right_rear_wheel_msg);
			}
		bool entrance_finished;
		if (!isenabled && n.getParam("/entrance_finished", entrance_finished)) {
			if (entrance_finished) { // First stop
				isenabled = true;
				left_front_wheel_msg.data=0;   // set left_wheel velocity
				right_front_wheel_msg.data=0;  // set right_wheel velocity
				left_rear_wheel_msg.data=0;
				right_rear_wheel_msg.data=0;

				pub_left_front_wheel.publish(left_front_wheel_msg);   // publish left_wheel velocity
				pub_right_front_wheel.publish(right_front_wheel_msg);  // publish right_wheel velocity
				pub_left_rear_wheel.publish(left_rear_wheel_msg);
				pub_right_rear_wheel.publish(right_rear_wheel_msg);
			}
		}
		
		suspension_msg.data = 0;	
		sus1.publish(suspension_msg);
		sus2.publish(suspension_msg);
		sus3.publish(suspension_msg);
		sus4.publish(suspension_msg);
	    ros::Duration(0.025).sleep();
	    ros::spinOnce();
    }

    return 0;
}
