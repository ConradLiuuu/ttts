#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "comparision");
  ros::NodeHandle nh;

  Mat img, img_binary, draw, draw1, bgr, bgr1;

  // ros image reansport setting
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_img = it.advertise("image",1);
  image_transport::Publisher pub_img_binary = it.advertise("image_binary",1);
  sensor_msgs::ImagePtr msg_img, msg_binary;

  // opencv settings
  vector<vector<Point> > contours; // Store all contour
  vector<vector<Point> > contourrrr;  // Store contour which area > 50
  float rad;
  Point2f cen;
  Point2f mc;

  // Read image
  std::string path = "/home/lab606a/findcontour/left_sample8.jpg";
  img = imread(path);

  draw = img.clone();
  draw1 = img.clone();
  bgr = img.clone();
  bgr1 = img.clone();
  

  // Convert color space and find contour
  cvtColor(img, img_binary, CV_BGR2HSV);
  inRange(img_binary, Scalar(10, 135, 250), Scalar(30, 255, 255), img_binary);
  findContours(img_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  // Draw contour
  for(int i = 0; i< contours.size(); i++){
    //Scalar color = Scalar( 255, 255, 0 );
    double area = contourArea(contours[i]);
    cout << "area = " << area << endl;
    if (area > 30) {
      contourrrr.push_back(contours[i]);
    }
  }

  vector<vector<Point>> contours_poly(contourrrr.size());
  vector<Point2f> center(contourrrr.size());
  vector<float> radius(contourrrr.size());

  // Find min enclosing circle
  for(int i = 0; i < contourrrr.size(); i++){
    approxPolyDP( Mat(contourrrr[i]), contours_poly[i], 3, true );
    minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
    cout << "center by minEnclosingCircle = " << center[i] << endl;
    cout << "radius by minEnclosingCircle = " << radius[i] << endl;
    rad = radius[i];
    cen = center[i];
  }

  vector<Moments> mu(contourrrr.size());

  for(int i = 0; i< contourrrr.size(); i++){
    mu[i] = moments(contourrrr[i], false);
  }
  for(int i = 0; i< contourrrr.size(); i++){
    mc = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
    Scalar color = Scalar( 255, 0, 0 );
    circle( draw, mc, rad, color, 1, 8, 0 );
    circle( bgr, mc, 1, color, -1, 8, 0 );  
    cout << "center by moment= " << mc << endl;
  }

  // Draw red circle by minEnclosing Circle
  Scalar colors = Scalar(0, 0, 255);
  circle(draw, cen, rad, colors, 1, 8, 0);
  //circle(bgr, cen, 1, colors, -1, 8, 0); //Draw center of object

  // Draw red circle by moments
  Scalar color = Scalar(255, 0, 0);
  circle(draw, mc, rad, color, 1, 8, 0);
  //circle(bgr, mc, 1, color, -1, 8, 0); //Draw center of object


  // Show image using ros
  msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw).toImageMsg();
  msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_binary).toImageMsg();

  while(ros::ok()){
    pub_img.publish(msg_img);
    pub_img_binary.publish(msg_binary);
  }

/*
  // Show image using openCV
  namedWindow("comparision", 1);
  imshow("comparision", draw);
  waitKey(0);
*/
  return 0;
}
