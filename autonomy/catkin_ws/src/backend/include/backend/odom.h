// odom.h

// Date Created: October 29, 2017

#ifndef ODOM_H_INCLUDED
#define ODOM_H_INCLUDED

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

class Odom {
	public:
		Odom(double wheelbase, double meters_per_tick);
		void updateOdom(uint8_t ticks_left, uint8_t ticks_right);
		double fixWrap256(unsigned char diff);
		double getWheelbase();
		nav_msgs::Odometry odom_msg_;
	private:
		ros::NodeHandle pr_nh_; // Private Node Handle for Getting Params

		double wheelbase_, meters_per_tick_;
		uint8_t prev_ticks_left_, prev_ticks_right_;
		std::string odom_frame_, base_frame_;

		// Time we last updated the odometry (for velocity estimation)
		ros::Time time_last_update_;

		// Robot state information
		double pose_x_, pose_y_, pose_yaw_;
		double velocity_x_, velocity_yaw_;
		Eigen::MatrixXd pose_covariance_; // Pose covariance is integrated, so need to store previous value

		// Odom tf publishing 
		bool publish_tf_;
		tf2_ros::TransformBroadcaster tf_broadcaster_;
		geometry_msgs::TransformStamped tf_odom_;

};

#endif // ODOM_H_INCLUDED