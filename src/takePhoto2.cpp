#include "FlyCapture2.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <thread>

using namespace FlyCapture2;
using namespace std;

cv::Mat img_L, img_R;

std::string path = "/home/lab606a/pic/";
std::string fileName_L;
std::string baseName_L = "left_sample";
std::string fileName_R;
std::string baseName_R = "right_sample";
std::string num;
int serial = 1;

void callback(const std_msgs::Bool::ConstPtr& msg);
void func();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "takePhoto2");
  ros::NodeHandle nh;
  ros::Rate rate(60);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_left = it.advertise("left_camera",1);
  image_transport::Publisher pub_right = it.advertise("right_camera",1);
  sensor_msgs::ImagePtr msg_left, msg_right;

  BusManager busMgr;

  // Left eye 
  Error error_L;
  Camera camera_L;
  CameraInfo camInfo_L;
  PGRGuid guid_L;

  // Right eye
  Error error_R;
  Camera camera_R;
  CameraInfo camInfo_R;
  PGRGuid guid_R;

  // Connect camera
  busMgr.GetCameraFromIndex(1, &guid_L);
  error_L = camera_L.Connect(&guid_L);
  if (error_L != PGRERROR_OK){
    //ROS_INFO("Failed to connect to left camera");
    return false;
  }

  busMgr.GetCameraFromIndex(0, &guid_R);
  error_R = camera_R.Connect(&guid_R);
  if (error_R != PGRERROR_OK){
    //ROS_INFO("Failed to connect to right camera");
    return false;
  }

  // Get camera info
  error_L = camera_L.GetCameraInfo(&camInfo_L);
  if (error_L != PGRERROR_OK){
    //ROS_INFO("Failed to get left camera info");
    return false;
  }
  cout << "Vendor name_L :" << camInfo_L.vendorName << endl;
  cout << "Model name_L :" << camInfo_L.modelName << endl;
  cout << "Serial number_L :" << camInfo_L.serialNumber << endl;

  error_R = camera_R.GetCameraInfo(&camInfo_R);
  if (error_R != PGRERROR_OK){
    //ROS_INFO("Failed to get right camera info");
    return false;
  }
  cout << "Vendor name_R :" << camInfo_R.vendorName << endl;
  cout << "Model name_R :" << camInfo_R.modelName << endl;
  cout << "Serial number_R :" << camInfo_R.serialNumber << endl;

  // Start capture image
  error_L = camera_L.StartCapture();
  if ( error_L == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    //ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error_L != PGRERROR_OK){
    //ROS_INFO("Failed to start image capture");
    return false;
  }

  error_R = camera_R.StartCapture();
  if ( error_R == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ){
    //ROS_INFO("Bandwidth exceeded");
    return false;
  }
  else if (error_R != PGRERROR_OK){
    //ROS_INFO("Failed to start image capture");
    return false;
  }

  // capture loop

  Image rawImage_L;
  Image bgrImage_L;
  Image rawImage_R;
  Image bgrImage_R;

  unsigned int rowBytes_L;
  unsigned int rowBytes_R;

  Error err_L;
  Error err_R;

  thread t1(func);

  //char key = 0;
  //char save = 0;

  //while (key != 'q'){
  while (ros::ok()){
    // Get image
    Error err_L = camera_L.RetrieveBuffer(&rawImage_L);
    Error err_R = camera_R.RetrieveBuffer(&rawImage_R);

    if (err_L != PGRERROR_OK){
      //ROS_INFO("Left camera captrue error");
      continue;
    }
    if (err_R != PGRERROR_OK){
      //ROS_INFO("Right camera captrue error");
      continue;
    }

    // Convert to bgr
    rawImage_L.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage_L);
    rawImage_R.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage_R);

    // Conver to OpenCV Mat
    rowBytes_L = (double)bgrImage_L.GetReceivedDataSize() / (double)bgrImage_L.GetRows();
    img_R = cv::Mat(bgrImage_L.GetRows(), bgrImage_L.GetCols(), CV_8UC3, bgrImage_L.GetData(), rowBytes_L);
    msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_R).toImageMsg();
    rowBytes_R = (double)bgrImage_R.GetReceivedDataSize() / (double)bgrImage_R.GetRows();
    img_L = cv::Mat(bgrImage_R.GetRows(), bgrImage_R.GetCols(), CV_8UC3, bgrImage_R.GetData(), rowBytes_R);
    msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_L).toImageMsg();

    pub_left.publish(msg_left);
    pub_right.publish(msg_right);

    rate.sleep();
    ros::spinOnce();
    /*
    cv::namedWindow("image left", 0);
    cv::imshow("image left", img_L);
    cv::namedWindow("image right", 0);
    cv::imshow("image right", img_R);

    if (save == 's'){
      num = std::to_string(serial);
      fileName_L = path + baseName_L + num + ".jpg";
      fileName_R = path + baseName_R + num + ".jpg";
      cv::imwrite(fileName_L, img_L, compression_params);
      cout << "Save left camera image:" << fileName_L << endl;
      cv::imwrite(fileName_R, img_R, compression_params);
      cout << "Save right camera image:" << fileName_R << endl;
      serial += 1;
    }
    
    key = cv::waitKey(1);
    save = cv::waitKey(1);
    */
  }

  t1.join();

  camera_L.Disconnect();

  return 0;
}

void callback(const std_msgs::Bool::ConstPtr& msg){
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  if (msg->data == true){
    num = std::to_string(serial);
    fileName_L = path + baseName_L + num + ".jpg";
    fileName_R = path + baseName_R + num + ".jpg";
    cv::imwrite(fileName_L, img_L, compression_params);
    cout << "Save left camera image:" << fileName_L << endl;
    cv::imwrite(fileName_R, img_R, compression_params);
    cout << "Save right camera image:" << fileName_R << endl;
    serial += 1;
  }
}

void func(){
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("takephoto", 1, callback);
  ros::spin();
}
