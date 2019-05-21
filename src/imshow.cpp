#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<iostream>
#include<geometry_msgs/Twist.h>
#include<iostream>
using namespace std;
using namespace cv;
int h=0,s=0,v=65;
int hmax=7,smax=125,vmax=252;
int flag=0;

int linearx=0,angularz=0;
float dev=0;

geometry_msgs::Twist mg;

int benchmark=1000;

Mat img,img2,img3;
void call(const sensor_msgs::ImageConstPtr& msg)
{

  cv::namedWindow("W1",CV_WINDOW_NORMAL);
  cv::namedWindow("W2",CV_WINDOW_NORMAL);
  cv::createTrackbar("h","W1",&h,255,NULL);
  cv::createTrackbar("hmax","W1",&hmax,255,NULL);
  cv::createTrackbar("s","W1",&s,255,NULL);
  cv::createTrackbar("smax","W1",&smax,255,NULL);
  cv::createTrackbar("v","W1",&v,255,NULL);
  cv::createTrackbar("vamx","W1",&vmax,255,NULL);
  cv::createTrackbar("benchmark","W2",&benchmark,1000,NULL);
  img=cv_bridge::toCvShare(msg,"bgr8")->image;
  cvtColor(img,img2,COLOR_BGR2HSV);
  inRange(img2,Scalar(h,s,v),Scalar(hmax,smax,vmax),img2);
  imshow("W3",img2);
  for(int i=0;i<img2.rows;i++)
  {
    for(int j=0;j<img2.cols;j++)
    {
      if(img2.at<unsigned char>(i,j)<20)
      img2.at<unsigned char>(i,j)=255;
      else
      img2.at<unsigned char>(i,j)=0;
    }
  }
  img2=img2(Rect(0,img2.rows/2,img2.cols,img2.rows/2));
  img=img(Rect(0,img.rows/2,img.cols,img.rows/2));
  //img3=img2(Rect(0,0,img2.cols,img2.rows/4));
  //imshow("W4",)
  imshow("W1",img2);
  Moments m = moments(img2);
  Point p(m.m10/m.m00, m.m01/m.m00);
  Point cen(img.cols/2,img.rows/2);
  dev=float(int(p.x)-int(cen.x));
  //cout<<int(p.x)<<endl;
  cout<<dev<<endl;
  if(int(p.x)==INT_MIN)
  flag=1;
  else
  flag=0;
  circle(img,p,10,Scalar(0,0,255),-1);
  circle(img,cen,10,Scalar(255,0,0),-1);
  imshow("W2",img);
 if(cv::waitKey(30)=='k')
 std::exit(0);
}

int main(int argc,char** argv)
{
  cout<<INT_MIN;
ros::init(argc,argv,"image_subscriber");
ros::NodeHandle nh;
cv::namedWindow("view",CV_WINDOW_NORMAL);
image_transport::ImageTransport it(nh);
image_transport::Subscriber sub=it.subscribe("/front_camera/image_raw",1,call);
ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/qt_pi/cmd_vel",1);
ros::Rate ob(31);
while(ros::ok())
{
  if(flag==1)
  {
    mg.linear.x=-0.1;
    mg.angular.z=0;
  }
  else
  {
 if((dev>0 && dev<=benchmark)|| (dev<0 && dev>=-benchmark))
 {
 mg.linear.x=0.2;
 mg.angular.z=dev/200;
}
 else
 {
 mg.angular.z=dev/100;
 mg.linear.x=.05;
}
}
 cout<<mg.linear.x<<endl;
 cout<<mg.angular.z<<endl;
 pub.publish(mg);
 ros::spinOnce();
 ob.sleep();
}
}
