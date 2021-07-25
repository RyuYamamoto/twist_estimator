#ifndef _TWIST_ESTIMATOR_
#define _TWIST_ESTIMATOR_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double eps = 0.0001;

class TwistEstimator
{
public:
  TwistEstimator();
  ~TwistEstimator() = default;

private:
  void poseStampedCallback(const geometry_msgs::PoseStamped msg);
  double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
  geometry_msgs::Vector3 convertToRPY(const geometry_msgs::Quaternion quaternion);

private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};

  ros::Subscriber pose_stamped_subscriber_;
  ros::Publisher twist_publisher_;

  std::string frame_id_;
};

#endif
