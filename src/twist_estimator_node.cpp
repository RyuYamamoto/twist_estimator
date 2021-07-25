#include <twist_estimator/twist_estimator.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "twist_estimator_node");

	TwistEstimator twist_estimator;

	ros::spin();

	return 0;
}
