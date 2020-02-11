#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ttts/HSVConfig.h>
/*
void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level)
{

  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);

}*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_HSV_server");
  ROS_INFO("Dynamic reconfigure HSV server is opened.");

  // Create dynamic reconfigure server
  dynamic_reconfigure::Server<ttts::HSVConfig> server;
  // define calback function, bind calback function and server
  //dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;

  //f = boost::bind(&callback, _1, _2);
  //server.setCallback(f);

  //ROS_INFO("");
  ros::spin();

  return 0;
}

