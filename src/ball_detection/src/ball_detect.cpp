
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <stdio.h>

#define PI 3.14159265

using namespace std;
using namespace cv;

// Setting Thresholds for red and blue part of image.
// Changable to fit your enviroment. If you want to use bgr, it should be different.

int low_h2_r=169, high_h2_r=180;
int low_h_r=0, low_s_r=134, low_v_r=106;
int high_h_r=8, high_s_r=255, high_v_r=255;
int low_h_b=100, low_s_b=126, low_v_b=60;
int high_h_b=121, high_s_b=255, high_v_b=255;

// Initialization of variable for camera calibration paramters.
// You should change this if you changed the size of the image.

float intrinsic_data[9] = {1206.8897719532354, 0.0, 960.5, 0.0, 1206.8897719532354, 540.5, 0.0, 0.0, 1.0};
float distortion_data[5] = {0, 0, 0, 0, 0};

// Initialization of variable for text drawing
String text;
int iMin_tracking_ball_size = 5; // Minimum ball radius size by pixels. If ball is smaller than this, it won't be searched.
float fball_diameter = 0.14 ; // Initialization of variable for dimension of the target(real ball diameter by meter)

// Setting Mat variables for images.
Mat buffer;
Mat result;

// Setting Publishers
ros::Publisher pub;
ros::Publisher pub_markers;

// Declaring functions for image erode and dilaation.
void morphOps(Mat &thresh) {
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));
  morphologyEx(thresh, thresh, MORPH_CLOSE, erodeElement);
  morphologyEx(thresh, thresh, MORPH_OPEN, erodeElement);
}

// Declaration of functions that calculates the ball position from pixel position.
vector<float> pixel2point(Point center, int radius) {
  vector<float> position;
  float x, y, u, v, Xc, Yc, Zc;
  x = center.x;//.x;// .at(0);
  y = center.y;//.y;//
  u = (x-intrinsic_data[2])/intrinsic_data[0];
  v = (y-intrinsic_data[5])/intrinsic_data[4];
  Zc = (intrinsic_data[0]*fball_diameter)/(2*(float)radius) ;
  Xc = u*Zc ;
  Yc = v*Zc ;
  Xc = roundf(Xc * 1000) / 1000;
  Yc = roundf(Yc * 1000) / 1000;
  Zc = roundf(Zc * 1000) / 1000;
  position.push_back(Xc);
  position.push_back(Yc);
  position.push_back(Zc);
  return position;
}

// Changing int variable to string.
string intToString(int n) {
  stringstream s;
  s << n;
  return s.str();
}

// Changing float variable to string.
string floatToString(float f) {
  ostringstream buffer;
  buffer << f;
  return buffer.str();
}

void ball_detect()
{
  // Declare intrinsic and distortions by using the variable declared before.
  Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);

  // Declare another Mat variable to keep the image.
  Mat frame;
  frame = buffer;

  Mat calibrated_frame;
  // Undistort frame images and save to calibrated frame.
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);

  // Defining Mat variables for Threshold images.
  Mat hsv_frame;
  Mat hsv_frame_red;
  Mat hsv_frame_red1;
  Mat hsv_frame_red2;
  Mat hsv_frame_blue;

  visualization_msgs::Marker ball_list;  //declare marker
  ball_list.header.frame_id = "/camera_link";  //set the frame
	ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
	ball_list.ns = "balls";   //name of markers
	ball_list.action = visualization_msgs::Marker::ADD;
	ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
	ball_list.pose.position.y=0;
	ball_list.pose.position.z=0;
	ball_list.pose.orientation.x=0;
	ball_list.pose.orientation.y=0;
	ball_list.pose.orientation.z=0;
	ball_list.pose.orientation.w=1.0;

  ball_list.id = 0; //set the marker id. if you use another markers, then make them use their own unique ids
	ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker

  double radius = 0.10;
  ball_list.scale.x=radius; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
  ball_list.scale.y=radius;
  ball_list.scale.z=radius;

  // Making clone of original image for drawing circles.
  result=calibrated_frame.clone();

  //Change RGB frame to HSV frame
  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

  //Threshold
  inRange(hsv_frame, Scalar(low_h_r,low_s_r,low_v_r), Scalar(high_h_r,high_s_r,high_v_r), hsv_frame_red1);
  inRange(hsv_frame, Scalar(low_h2_r,low_s_r,low_v_r), Scalar(high_h2_r,high_s_r,high_v_r), hsv_frame_red2);
  inRange(hsv_frame, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b), hsv_frame_blue);
  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

  //Blur and erode, dilate
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));
  Mat hsv_frame_red_1, hsv_frame_red_2, hsv_frame_blue_1, hsv_frame_blue_2, hsv_frame_green_1, hsv_frame_green_2;

  morphologyEx(hsv_frame_red, hsv_frame_red_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_red_1, hsv_frame_red_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_red_2, hsv_frame_red, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_blue, hsv_frame_blue_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_blue_1, hsv_frame_blue_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_blue_2, hsv_frame_blue, cv::Size(9, 9), 2, 2);

  //Canny Edge Detection
  int lowThreshold=100;
  int ratio=3;
  int kernel_size=3;

  Mat img_canny_blue;
  Mat img_canny_red;

  Canny(hsv_frame_blue, img_canny_blue, lowThreshold, lowThreshold*ratio, kernel_size);
  Canny(hsv_frame_red, img_canny_red, lowThreshold, lowThreshold*ratio, kernel_size);

  //Finding Contours for blue threshold image
  vector<Vec4i> hierarchy_b;
  vector<vector<Point> > contours_b;
  findContours(img_canny_blue, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_r;
  vector<vector<Point> > contours_r;
  findContours(img_canny_red, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

  // Define variables for contour poly, center of circles, radius of circles
  vector<vector<Point> > contours_poly_b( contours_b.size() );
  vector<Point2f> center_b( contours_b.size() );
  vector<float> radius_b( contours_b.size() );

  vector<vector<Point> > contours_poly_r( contours_r.size() );
  vector<Point2f> center_r( contours_r.size() );
  vector<float> radius_r( contours_r.size() );

  /*Finding blue balls by contours
    Find polygon from contours and find the minimun size enclosing circle of that polygon.
  */

  for( size_t i = 0; i < contours_b.size(); i++ )
  {
    approxPolyDP( contours_b[i], contours_poly_b[i], 1, true );
    minEnclosingCircle( contours_poly_b[i], center_b[i], radius_b[i] );
  }

  for( size_t i = 0; i < contours_r.size(); i++ )
  {
    approxPolyDP( contours_r[i], contours_poly_r[i], 1, true );
    minEnclosingCircle( contours_poly_r[i], center_r[i], radius_r[i] );
  }

  // Declare message variable to publish
  core_msgs::ball_position msg;
  int ball_num = 0;

  // For blue balls
  for (size_t i=0; i<contours_b.size(); i++)
  {
    if(radius_b[i] > iMin_tracking_ball_size) {

      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(255,0,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle center, and the pixel radius of an circle
      float px_b = center_b[i].x;
      float py_b = center_b[i].y;
      float pr_b = radius_b[i];

      // change the pixel value to real world value

      vector<float> ball_pos_b;
      ball_pos_b = pixel2point(center_b[i],radius_b[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image
      float isx = ball_pos_b[0];
      float isy = ball_pos_b[1];
      float isz = ball_pos_b[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_b[i], 2, 1, color_g, 2);
      circle(result, center_b[i], (int)radius_b[i], color, 2, 8, 0);
      ball_num = ball_num + 1;

      geometry_msgs::Point p;
      p.x = px_b;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
      p.y = py_b;
      p.z = 0.1;
      ball_list.points.push_back(p);

      std_msgs::ColorRGBA c;
      c.r = 1.0;  //set the color of the balls. You can set it respectively.
      c.g = 0.0;
      c.b = 0.0;
      c.a = 1.0;
      ball_list.colors.push_back(c);

      //push back variables of real ball position to the message variable
      msg.img_x.push_back(ball_pos_b[0]);
      msg.img_y.push_back(ball_pos_b[2]);
    }
  }

  // do same procedure for red balls

  for (size_t i=0; i<contours_r.size(); i++)
  {
    if(radius_r[i] > iMin_tracking_ball_size)
    {
      Scalar color = Scalar(0,0,255);
      Scalar color_g = Scalar(0,255,0);

      float px_r = center_r[i].x;
      float py_r = center_r[i].y;
      float pr_r = radius_r[i];

      vector<float> ball_pos_r;
      ball_pos_r = pixel2point(center_r[i],radius_r[i]);


      float isx = ball_pos_r[0];
      float isy = ball_pos_r[1];
      float isz = ball_pos_r[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_r[i], 2, 1, color_g, 2);
      circle(result, center_r[i], (int)radius_r[i], color, 2, 8, 0);
      ball_num = ball_num + 1;

      geometry_msgs::Point p;
      p.x = px_r;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
      p.y = py_r;
      p.z = 0.1;
      ball_list.points.push_back(p);

      std_msgs::ColorRGBA c;
      c.r = 0.0;  //set the color of the balls. You can set it respectively.
      c.g = 0.0;
      c.b = 1.0;
      c.a = 1.0;
      ball_list.colors.push_back(c);

      msg.img_x.push_back(ball_pos_r[0]);
      msg.img_y.push_back(ball_pos_r[2]);
    }
  }
  msg.size = ball_num;
  //show what is published at the terminal
  cout<<msg.size<<endl;
  for (int i=0 ; i<ball_num ; i++)
  {
    cout<<msg.img_x[i]<<endl;
    cout<<msg.img_y[i]<<endl;
  }
  pub.publish(msg); //publish a message
  pub_markers.publish(ball_list);
  
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  if(msg->height!=480 || buffer.size().width!=320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640, buffer.size().height*640/buffer.size().width));
  }
  ball_detect();
  imshow("result", result);
  waitKey(10);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "ball_detect_node"); //init ros nodd
  ros::NodeHandle nh; //create node handler
  image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
  pub = nh.advertise<core_msgs::ball_position>("/position", 1); //setting publisher
  pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);
  ros::Rate loop_rate(30);
  //namedWindow("result", WINDOW_NORMAL);
  while(ros::ok()){
     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;
}
