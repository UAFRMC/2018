// odom.h

// Date Created: October 29, 2017

#ifndef ODOM_H_INCLUDED
#define ODOM_H_INCLUDED

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

class Odom {
	public:
		Odom(double wheelbase, double meters_per_tick);
		void updateOdom(int16_t ticks_left, int16_t ticks_right);
		nav_msgs::Odometry odom_msg_;
	private:
		double wheelbase_, meters_per_tick_;

		// Time we last updated the odometry (for velocity estimation)
		ros::Time time_last_update_;

		// Robot state information
		double pose_x_, pose_y_, pose_yaw_;
		double velocity_x_, velocity_yaw_;
		Eigen::MatrixXd pose_covariance_; // Pose covariance is integrated, so need to store previous value
};

#endif // ODOM_H_INCLUDED