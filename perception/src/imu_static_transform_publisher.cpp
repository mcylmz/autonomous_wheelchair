#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Eigen>
#include <cmath>

/**
 * Calculates roll and pitch from the IMU linear acceleration.
*/

#define NUM_SAMPLES 500

sensor_msgs::Imu::_linear_acceleration_type linear_accel;

void accel_cb(const sensor_msgs::ImuConstPtr& accel) {
  linear_accel = accel->linear_acceleration;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "imu_static_transform_publisher");
  ros::NodeHandle node_handle;
  if(argc != 7)
  {
    ROS_ERROR("Invalid number of parameters\nusage: imu_static_transform_publisher base_frame_name camera_frame_name x y z yaw");
    return -1;
  }
  if(strcmp(argv[1], argv[2])==0)
  {
    ROS_ERROR("Camera frame name and base frame name cannot be same");
    return -1;

  }
  std::string base_frame_name = argv[1];
  std::string camera_frame_name = argv[2];

  std::cout << base_frame_name << ", " << camera_frame_name << std::endl;
  
  ros::Subscriber accel = node_handle.subscribe ("/camera/accel/sample", 1, accel_cb);

  ros::Rate r(250); // Hz
  Eigen::Vector3f axis(0, 0, 0);
  for (size_t i = 0; i < NUM_SAMPLES; i++)
  {
    ros::spinOnce();
    axis += Eigen::Vector3f(linear_accel.x, linear_accel.y, linear_accel.z);
    r.sleep();
  }
  axis /= axis.norm();

  accel.shutdown();

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = base_frame_name;
  static_transformStamped.child_frame_id = camera_frame_name;
  static_transformStamped.transform.translation.x = atof(argv[3]);
  static_transformStamped.transform.translation.y = atof(argv[4]);
  static_transformStamped.transform.translation.z = atof(argv[5]);

  tf2::Quaternion quat;
  float roll = -(atan2f(axis.y(), axis.x()) + M_PI_2);
  float pitch = -(atan2f(axis.y(), axis.z()) + M_PI_2);
  float yaw = atof(argv[6]);
  quat.setRPY(roll, pitch, yaw);

  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
  ROS_INFO("Spinning until killed publishing %s to %s", camera_frame_name.c_str(), base_frame_name.c_str());
  ros::spin();
  return 0;
};
