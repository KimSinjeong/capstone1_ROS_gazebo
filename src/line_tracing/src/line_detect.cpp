#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include "core_msgs/line_segments.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>

using namespace std;
using namespace cv;

ros::Publisher pub;
Mat buffer, image_gray, image_binary;

bool isenabled = true;

int thresh = 100;

int nolinecnt = 0;

int thickness = 1;
int font = FONT_HERSHEY_SIMPLEX;// hand-writing style font
double fontScale = 0.75;

Scalar blue(255, 0, 0), red(0, 0, 255), green(0, 255, 0);

void imagegrabber (const sensor_msgs::ImageConstPtr& msg) {
    if (!isenabled) return;

    try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
    catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
    
    int width = 640;
    int height = buffer.size().height*640/buffer.size().width;
    resize(buffer, buffer, Size(width, height));

    cvtColor(buffer, image_gray, COLOR_BGR2GRAY);
    threshold(image_gray, image_binary, 10, 255, THRESH_BINARY);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    bitwise_not(image_binary, image_binary);

    /// Find contours
    findContours( image_binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    core_msgs::line_segments pmsg;
    pmsg.size = contours.size();
    pmsg.com_x.resize(contours.size());
    pmsg.com_y.resize(contours.size());
    pmsg.mass.resize(contours.size());

    if ((pmsg.size == 0) && (++nolinecnt == 4)) {
        ros::param::set("/entrance_finished", true);
        isenabled = false;
        return;
    }
    if (pmsg.size != 0)
        nolinecnt = 0;

    /// Get the moments, mass centers:
    vector<Moments> mu(contours.size() );
    vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ ) {
        mu[i] = moments( contours[i], false );
        mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
        pmsg.com_x[i] = mc[i].x - width/2;
        pmsg.com_y[i] = height - mc[i].y;
        pmsg.mass[i] = mu[i].m00;
    }

    /// Draw contours
    Mat drawing = Mat(buffer);
    line(drawing, Point(width/2, 0), Point(width/2, height-1), red, 3);

    for( int i = 0; i< contours.size(); i++ )
    {
        printf(" * Contour[%d] - Area (M_00) = %.2f - Dist from center: %.2f \n", i, mu[i].m00, mc[i].x-width/2 );
        line(drawing, Point(width/2, mc[i].y), Point(mc[i].x, mc[i].y), blue, 2);
        putText(drawing, to_string(mc[i].x-width/2).c_str(), Point((width/2 + mc[i].x)/2, mc[i].y-10), font, fontScale, blue, thickness);
        drawContours( drawing, contours, i, green, 2, 8, hierarchy, 0, Point() );
        circle( drawing, mc[i], 4, green, -1, 8, 0 );
    }

    pub.publish(pmsg);
    imshow( "Contours", drawing );
    waitKey(1);
}

int main (int argc, char **argv) {
    ros::init (argc, argv, "line_detect_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    sleep(4);

    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imagegrabber);
    pub = nh.advertise<core_msgs::line_segments>("/segments", 100);

/*
    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();
        bool entrance_finished;
        if (nh.getParam("/entrance_finished", entrance_finished) && isenabled) {
            if (entrance_finished) {
                isenabled = false;
            }
        }
    }
*/
    ros::spin();
    return 0;
}