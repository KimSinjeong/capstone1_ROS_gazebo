#include <ros/ros.h>
#include "core_msgs/line_segments.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <std_msgs/Float64.h>

using namespace std;

ros::Publisher pub_right_front_wheel;
ros::Publisher pub_left_front_wheel;
ros::Publisher pub_left_rear_wheel;
ros::Publisher pub_right_rear_wheel;

std_msgs::Float64 right_front;
std_msgs::Float64 left_front;
std_msgs::Float64 left_rear;
std_msgs::Float64 right_rear;

int correct=0;


void decision_center(const core_msgs::line_segments::ConstPtr& msg)
{
    int count; //this is the number of actual data points received
    int array_size = msg->size;

    if (array_size == 0)
        return;

    count = array_size;
    for (int i =0;i < array_size;i++)
    {
        if (isnan(msg->com_x[i]))
        {
            count=i;
            cout << "count = " << count << "    " << msg->com_x[0] << endl;
            break;
        }
    }

   if (count>0){
        double error = msg->com_x[0];
        right_front.data=50;
        left_front.data=50;
        left_rear.data=50;
        right_rear.data=50;

        if (abs(error)<55){
            correct = 0;
        }

        if (abs(error)>80 || correct==1){
            correct =1;
            if (error>0){
                left_front.data = 50; //50
                right_front.data = -48;//40
                left_rear.data=20;//40
                right_rear.data = -20;
            }
            else{
                right_front.data = 50;
                left_front.data = -48;
                right_rear.data = 20;//chaznge
                left_rear.data = -20;
            }
        }
/*
        else if (abs(error)>80 || correct == 2){
            correct =2;
            if (error>0){
                right_front.data = -20;
                left_rear.data=20;
                right_rear.data = -20;
            }
            else{
                left_front.data = -20;
                right_rear.data = 20;
                left_rear.data = -20;
            }
        }
*/

        pub_left_front_wheel.publish(left_front);
        pub_right_front_wheel.publish(right_front);
        pub_left_rear_wheel.publish(left_rear);
        pub_right_rear_wheel.publish(right_rear);

        cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
            << "    " << left_rear.data << endl;
    }



}

int main (int argc, char **argv) 
{
    ros::init (argc, argv, "line_detect_actuation_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/segments", 10, decision_center);

    pub_right_front_wheel = nh.advertise<std_msgs::Float64>("model13/right_front_wheel_velocity_controller/command", 10);
    pub_left_front_wheel = nh.advertise<std_msgs::Float64>("model13/left_front_wheel_velocity_controller/command", 10);
    pub_right_rear_wheel = nh.advertise<std_msgs::Float64>("model13/right_rear_wheel_velocity_controller/command", 10);
    pub_left_rear_wheel = nh.advertise<std_msgs::Float64>("model13/left_rear_wheel_velocity_controller/command", 10);
     
    ros::spin();
    return 0;
}