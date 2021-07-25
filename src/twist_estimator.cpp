#include <twist_estimator/twist_estimator.h>

TwistEstimator::TwistEstimator()
{
  pnh_.param<std::string>("frame_id", frame_id_, "base_link");

  pose_stamped_subscriber_ = pnh_.subscribe("pose", 10, &TwistEstimator::poseStampedCallback, this);
  twist_publisher_ = pnh_.advertise<geometry_msgs::TwistStamped>("twist", 10);
}

double TwistEstimator::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (M_PI <= diff_rad)
    diff_rad -= 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad += 2 * M_PI;
  return diff_rad;
}

geometry_msgs::Vector3 TwistEstimator::convertToRPY(const geometry_msgs::Quaternion quaternion)
{
  tf2::Quaternion tf2_quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf2::Matrix3x3 mat(tf2_quat);
  geometry_msgs::Vector3 rpy;
  mat.getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

void TwistEstimator::poseStampedCallback(const geometry_msgs::PoseStamped msg)
{
  static geometry_msgs::PoseStamped prev_pose_stamped = msg;

  const double dt = (msg.header.stamp - prev_pose_stamped.header.stamp).toSec();

  geometry_msgs::Vector3 rpy = convertToRPY(msg.pose.orientation);
  geometry_msgs::Vector3 prev_rpy = convertToRPY(prev_pose_stamped.pose.orientation);

  const double diff_x = msg.pose.position.x - prev_pose_stamped.pose.position.x;
  const double diff_y = msg.pose.position.y - prev_pose_stamped.pose.position.y;
  const double diff_z = msg.pose.position.z - prev_pose_stamped.pose.position.z;

  geometry_msgs::TwistStamped twist;
  twist.header = msg.header;
  twist.header.frame_id = frame_id_;
  twist.twist.linear.x =
    std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2) + std::pow(diff_z, 2)) / dt;
  twist.twist.linear.y = 0.0;
  twist.twist.linear.z = 0.0;
  twist.twist.angular.x = calcDiffForRadian(rpy.x, prev_rpy.x) / dt;
  twist.twist.angular.x = calcDiffForRadian(rpy.y, prev_rpy.y) / dt;
  twist.twist.angular.x = calcDiffForRadian(rpy.z, prev_rpy.z) / dt;

  twist_publisher_.publish(twist);

  prev_pose_stamped = msg;
}
