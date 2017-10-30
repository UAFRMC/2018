// odom.cpp

// Date Created: October 29, 2017

#include <backend/odom.h>
#include <cmath>
#define _USE_MATH_DEFINES

Odom::Odom(double wheelbase, double meters_per_tick) 
	: wheelbase_(wheelbase), meters_per_tick_(meters_per_tick), pose_x_(0), pose_y_(0), pose_yaw_(0),
	velocity_x_(0), velocity_yaw_(0)
{
	// Make sure covariance arrays are zeroed out
	for(int i=0; i<9; ++i) {
		pose_covariance_[i] = 0;
		velocity_covariance_[i] = 0;
	}

	// Initialize ROS Message
	odom_msg_.header.frame_id = "odom";

	// Initialize last odometry update time. Assuming we enter the update loop before the robot moves,
	//     this should be ok.
	time_last_update_ = ros::Time::now();
}

void Odom::updateOdom(int16_t ticks_left, int16_t ticks_right) {
	// Update the wheel odometry. This code is inspired by libcreate and create_autonomy

	// Get time difference between updates
	double delta_t = (ros::Time::now() - time_last_update_).toSec();
	time_last_update_ = ros::Time::now();

	// Compute the distance travelled by each wheel
	double left_wheel_dist = ticks_left*meters_per_tick_;
	double right_wheel_dist = ticks_right*meters_per_tick_;
	double delta_dist = (left_wheel_dist + right_wheel_dist)/2.0;

	// Compute the change in yaw of the robot
	double wheel_dist_diff = right_wheel_dist - left_wheel_dist;
	double delta_yaw = wheel_dist_diff/wheelbase_;

	// Compute the change in position of the robot
	double delta_x, delta_y;
	const double EPSILON = 0.0001;
	if(fabs(wheel_dist_diff) < EPSILON) { // Moving Straight
		delta_x = delta_dist*cos(pose_yaw_);
		delta_y = delta_dist*sin(pose_yaw_);
	}
	else { // Turning
		double turn_radius = (wheelbase_/2.0)*(left_wheel_dist + right_wheel_dist)/wheel_dist_diff;
		delta_x = turn_radius*(sin(pose_yaw_ + delta_yaw) - sin(pose_yaw_));
		delta_y = -turn_radius*(cos(pose_yaw_ + delta_yaw) - cos(pose_yaw_));
	}
	
	// Estimate the velocity of the robot
	if(delta_t > EPSILON) {
		velocity_x_ = delta_dist/delta_t;
		velocity_yaw_ = delta_yaw/delta_t;
	}
	else {
		velocity_x_ = 0;
		velocity_yaw_ = 0;
	}

	// Update Pose
	pose_x_ += delta_x;
	pose_y_ += delta_y;
	pose_yaw_ += delta_yaw;
	// Normalize the yaw angle
	while (pose_yaw_ < -M_PI) pose_yaw_ += 2*M_PI;
	while (pose_yaw_ > M_PI) pose_yaw_ -= 2*M_PI;

	// Update Covariances
	// Ref: "Introduction to Autonomous Mobile Robots" (Siegwart 2004, page 189)
	double kr = 1.0;
	double kl = 1.0;
	double cos_yaw_and_half_delta = cos(pose_yaw_ + (delta_yaw/2.0));
	double sin_yaw_and_half_delta = sin(pose_yaw_ + (delta_yaw/2.0));
	double dist_over_two_wb = delta_dist/(2*wheelbase_);




	// ***** Move the data into the ROS Message ***** //
	// Populate Position Info
	tf2::Quaternion quat;
	quat.setEuler(pose_yaw_,0,0);
	odom_msg_.pose.pose.position.x = pose_x_;
	odom_msg_.pose.pose.position.y = pose_y_;
	odom_msg_.pose.pose.orientation = tf2::toMsg(quat);

	// Populate Velocity Info
	odom_msg_.twist.twist.linear.x = velocity_x_;
	odom_msg_.twist.twist.angular.z = velocity_yaw_;

	// Update covariances
	odom_msg_.pose.covariance[0] = pose_covariance_[0];
	odom_msg_.pose.covariance[1] = pose_covariance_[1];
	odom_msg_.pose.covariance[5] = pose_covariance_[2];
	odom_msg_.pose.covariance[6] = pose_covariance_[3];
	odom_msg_.pose.covariance[7] = pose_covariance_[4];
	odom_msg_.pose.covariance[11] = pose_covariance_[5];
	odom_msg_.pose.covariance[30] = pose_covariance_[6];
	odom_msg_.pose.covariance[31] = pose_covariance_[7];
	odom_msg_.pose.covariance[35] = pose_covariance_[8];
	odom_msg_.twist.covariance[0] = velocity_covariance_[0];
	odom_msg_.twist.covariance[1] = velocity_covariance_[1];
	odom_msg_.twist.covariance[5] = velocity_covariance_[2];
	odom_msg_.twist.covariance[6] = velocity_covariance_[3];
	odom_msg_.twist.covariance[7] = velocity_covariance_[4];
	odom_msg_.twist.covariance[11] = velocity_covariance_[5];
	odom_msg_.twist.covariance[30] = velocity_covariance_[6];
	odom_msg_.twist.covariance[31] = velocity_covariance_[7];
	odom_msg_.twist.covariance[35] = velocity_covariance_[8];	
}