#include "ros/ros.h"
#include "sensor_msgs/Image.h"

ros::Time lastFrameTime;
ros::Publisher debug_pub;


void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // Image data is 14 bits stored in 16 bits
  uint32_t width = msg->width;
  uint32_t height = msg->height;
  std::vector<uint8_t> data = msg->data;
  uint16_t *c_data = (uint16_t*)&data[0];

  ROS_INFO("SIZE: %d", msg->data.size());

  // Calculate time since last frame
  // Since we use the frame timestamps instead of the system time,
  // this will play nice with bag replay! e.g when we replay at half speed
  ros::Time timestamp = msg->header.stamp;
  ros::Duration sinceLastFrame = timestamp - lastFrameTime;
  lastFrameTime = timestamp;
  float deltaMs = sinceLastFrame.toSec() * 1000;

  // ROS_INFO("Image width/height: %d / %d", width, height);
  // ROS_INFO("ms since last frame: %0.2f", deltaMs);

  // PING PUT YOUR CODE HERE!
  // to rebuild and re-run:
    // cd ~/catkin_ws
    // catkin build thermal_processing
    // rosrun thermal_processsing node


  // this is just a little demo of publishing a debug image
  // feel free to remove this
  // sensor_msgs::Image imageOriginal = *msg;
  // sensor_msgs::Image imageCopy = imageOriginal;
  // for (int i = 0; i < imageCopy.data.size(); i++) {
  //   imageCopy.data[i] = 255 - imageCopy.data[i];
  // }
  // debug_pub.publish(imageCopy);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thermal_processing_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/thermal/image_raw", 1000, imageCallback);
  debug_pub = n.advertise<sensor_msgs::Image>("/thermal/debug", 1000);

  ros::spin();

  return 0;
}
