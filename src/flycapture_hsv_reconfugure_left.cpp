#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "FlyCapture2.h"

using namespace FlyCapture2;
//using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flycapture_HSV_reconfigure_left");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_img = it.advertise("origin_left",1);
  image_transport::Publisher pub_bin = it.advertise("binary_left",1);
  //image_transport::Publisher pub_canny = it.advertise("canny",1);
  ros::Rate rate(10);
  sensor_msgs::ImagePtr msg_origin, msg_binary ,msg_canny;
  int H_min, H_max,S_min, S_max,V_min, V_max;

  Error error;
  Camera camera;
  CameraInfo camInfo;
  BusManager busMgr;
  PGRGuid guid_L;

  //unsigned int SerialNumber = 17491067; // right
  unsigned int SerialNumber = 17491073; // left

  // Connect camera
  busMgr.GetCameraFromSerialNumber(SerialNumber, &guid_L);
  error = camera.Connect(&guid_L);
  if (error != PGRERROR_OK){
    ROS_INFO("Failed to connect to camera");
    return false;
  }

  // Get camera info
  error = camera.GetCameraInfo(&camInfo);
  if (error != PGRERROR_OK){
    ROS_INFO("Failed to get camera info");
    return false;
  }
  cout << "Vendor name :" << camInfo.vendorName << endl;
  cout << "Model name :" << camInfo.modelName << endl;
  cout << "Serial number :" << camInfo.serialNumber << endl;

  // Start capture image
  error = camera.StartCapture();
  if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error != PGRERROR_OK){
    ROS_INFO("Failed to start image capture");
    return false;
  }

  // capture loop

  Image rawImage;
  Image bgrImage;

  unsigned int rowBytes;
  cv::Mat img, img_hsv, img_canny;
  cv::cuda::GpuMat uimg_src, uimg_dst;

  cv::cuda::GpuMat hsv, shsv[3], thresc[3], temp, thres;

  cv::Mat threshold = cv::Mat::zeros(1536, 2048, CV_8UC3);

  Error err;
  char key = 0;

  //while (key != 'q'){
  while (ros::ok()){
    // Get image
    //Image rawImage;
    Error err = camera.RetrieveBuffer(&rawImage);

    if (err != PGRERROR_OK){
      ROS_INFO("Captrue error");
      continue;
    }
    
    // Convert to bgr
    //Image bgrImage;
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);

    // Conver to OpenCV Mat
    rowBytes = (double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
    img = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(), rowBytes);
/*
    ROS_INFO("rows = %d", bgrImage.GetRows());
    ROS_INFO("cols = %d", bgrImage.GetCols());
*/
    //cv::resize(img, img, cv::Size(), 0.315, 0.315);

    img_hsv = img.clone();
    //uimg_src.upload(img);
    //cv::cuda::cvtColor(uimg_src, uimg_dst, CV_BGR2HSV_FULL);
    //cv::cuda::cvtColor(uimg_src, uimg_dst, CV_BGR2HSV);
    //uimg_dst.download(img_hsv);
    cv::cvtColor(img_hsv, img_hsv, CV_BGR2HSV_FULL);
/*
    cv::cuda::cvtColor(uimg_src, hsv, CV_BGR2HSV_FULL);
    cv::cuda::split(hsv, shsv);
*/
    nh.getParam("/dynamic_HSV_server/H_min_L",H_min);
    nh.getParam("/dynamic_HSV_server/H_max_L",H_max);
    nh.getParam("/dynamic_HSV_server/S_min_L",S_min);
    nh.getParam("/dynamic_HSV_server/S_max_L",S_max);
    nh.getParam("/dynamic_HSV_server/V_min_L",V_min);
    nh.getParam("/dynamic_HSV_server/V_max_L",V_max);

    cv::inRange(img_hsv, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), threshold);
    /*
    cv::cuda::threshold(shsv[0], thresc[0], H_min, H_max, CV_THRESH_BINARY);
    cv::cuda::threshold(shsv[1], thresc[1], S_min, S_max, CV_THRESH_BINARY);
    cv::cuda::threshold(shsv[2], thresc[2], V_min, V_max, CV_THRESH_BINARY);

    cv::cuda::bitwise_and(thresc[0], thresc[1], temp);
    cv::cuda::bitwise_and(temp, thresc[2], thres);
    thres.download(threshold);
*/

    //cv::Canny(threshold, img_canny, 50, 240, 3);


    msg_origin = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    msg_binary = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold).toImageMsg();
    //msg_canny = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_canny).toImageMsg();
    
    pub_img.publish(msg_origin);
    pub_bin.publish(msg_binary);
    //pub_canny.publish(msg_canny);
    ros::spinOnce();
    rate.sleep();
/*
    cv::namedWindow("img", 0);
    cv::imshow("img", img);

    cv::namedWindow("canny", 0);
    cv::imshow("canny", img_canny);
    key = cv::waitKey(1);
*/
  }

  //error = camera.StopCapture();
  camera.Disconnect();

  return 0;
}
