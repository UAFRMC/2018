// odom.cpp

// Date Created: October 29, 2017

#include <backend/odom.h>
#include <cmath>
#define _USE_MATH_DEFINES

Odom::Odom(double wheelbase, double meters_per_tick) 
	: pr_nh_("~"), wheelbase_(wheelbase), meters_per_tick_(meters_per_tick), prev_ticks_left_(0), prev_ticks_right_(0),
	pose_x_(0), pose_y_(0), pose_yaw_(0), velocity_x_(0), velocity_yaw_(0), pose_covariance_(3,3)
{
	// Get Parameters
	pr_nh_.param<std::string>("base_frame", base_frame_, "base_footprint");
	pr_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
	pr_nh_.param<bool>("publish_tf", publish_tf_, true);

	// Initialize ROS Message
	odom_msg_.header.frame_id = odom_frame_;

	// Initialize odom tf
	tf_odom_.header.frame_id = odom_frame_;
	tf_odom_.child_frame_id = base_frame_;

	pose_covariance_ = Eigen::MatrixXd::Constant(3,3,0.0); // Initialize pose covariance to all zeros

	// Initialize last odometry update time. Assuming we enter the update loop before the robot moves,
	//     this should be ok.
	time_last_update_ = ros::Time::now();
}

void Odom::updateOdom(uint8_t ticks_left, uint8_t ticks_right) {
	// Update the wheel odometry. This code is inspired by libcreate and create_autonomy

	//***** Estimate Pose and Velocity ***** //
	// Get time difference between updates
	double delta_t = (ros::Time::now() - time_last_update_).toSec();
	time_last_update_ = ros::Time::now();

	double delta_ticks_left = fixWrap256(ticks_left - prev_ticks_left_);
	double delta_ticks_right = fixWrap256(ticks_right - prev_ticks_right_);

	prev_ticks_left_ = ticks_left;
	prev_ticks_right_ = ticks_right;

	// Compute the distance travelled by each wheel
	double left_wheel_dist = delta_ticks_left*meters_per_tick_;
	double right_wheel_dist = delta_ticks_right*meters_per_tick_;
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

	// Transfer pose and velocity info to odom_msg_
	// Populate Position Info
	tf2::Quaternion quat;
	quat.setEuler(pose_yaw_,0,0);
	odom_msg_.pose.pose.position.x = pose_x_;
	odom_msg_.pose.pose.position.y = pose_y_;
	odom_msg_.pose.pose.orientation = tf2::toMsg(quat);

	// Populate Velocity Info
	odom_msg_.twist.twist.linear.x = velocity_x_;
	odom_msg_.twist.twist.angular.z = velocity_yaw_;

	// ********** //


	// ***** Update Covariances for Pose and Velocity ***** //
	// Ref: "Introduction to Autonomous Mobile Robots" (Siegwart 2011, Page 270)

	// kr and kl are error constants representing the nondeterministic parameters of the motor drive
	//     and the wheel-floor interaction. 
	double kr = 1.0;
	double kl = 1.0;
	double cos_yaw_and_half_delta = cos(pose_yaw_ + (delta_yaw/2.0));
	double sin_yaw_and_half_delta = sin(pose_yaw_ + (delta_yaw/2.0));
	double dist_over_two_wb = delta_dist/(2*wheelbase_);

	// Motion increment covariance matrix
	Eigen::MatrixXd inc_covar(2, 2);
	inc_covar(0,0) = kr*fabs(right_wheel_dist);
	inc_covar(0,1) = 0.0;
	inc_covar(1,0) = 0.0;
	inc_covar(1,1) = kl*fabs(left_wheel_dist);

	// Velocity Jacobian
	Eigen::MatrixXd Finc(3,2);
	Finc(0,0) = (cos_yaw_and_half_delta/2.0) - (dist_over_two_wb*sin_yaw_and_half_delta);
	Finc(0,1) = (cos_yaw_and_half_delta/2.0) + (dist_over_two_wb*sin_yaw_and_half_delta);
	Finc(1,0) = (sin_yaw_and_half_delta/2.0) + (dist_over_two_wb*cos_yaw_and_half_delta);
	Finc(1,1) = (sin_yaw_and_half_delta/2.0) - (dist_over_two_wb*cos_yaw_and_half_delta);
	Finc(2,0) = (1.0/wheelbase_);
	Finc(2,1) = (-1.0/wheelbase_);
	Eigen::MatrixXd FincT = Finc.transpose();

	// Position Jacobian
	Eigen::MatrixXd Fp(3,3);
	Fp(0,0) = 1.0;
	Fp(0,1) = 0.0;
	Fp(0,2) = (-delta_dist)*sin_yaw_and_half_delta;
	Fp(1,0) = 0.0;
	Fp(1,1) = 1.0;
	Fp(1,2) = delta_dist*cos_yaw_and_half_delta;
	Fp(2,0) = 0.0;
	Fp(2,1) = 0.0;
	Fp(2,2) = 1.0;
	Eigen::MatrixXd FpT = Fp.transpose();

	// Compute matrices
	Eigen::MatrixXd vel_covar = Finc*inc_covar*FincT;
	pose_covariance_ = Fp*pose_covariance_*FpT + vel_covar;

	// Transfer covariance matrices to odom message
	odom_msg_.pose.covariance[0] = pose_covariance_(0,0);
	odom_msg_.pose.covariance[1] = pose_covariance_(0,1);
	odom_msg_.pose.covariance[5] = pose_covariance_(0,2);
	odom_msg_.pose.covariance[6] = pose_covariance_(1,0);
	odom_msg_.pose.covariance[7] = pose_covariance_(1,1);
	odom_msg_.pose.covariance[11] = pose_covariance_(1,2);
	odom_msg_.pose.covariance[30] = pose_covariance_(2,0);
	odom_msg_.pose.covariance[31] = pose_covariance_(2,1);
	odom_msg_.pose.covariance[35] = pose_covariance_(2,2);
	odom_msg_.twist.covariance[0] = vel_covar(0,0);
	odom_msg_.twist.covariance[1] = vel_covar(0,1);
	odom_msg_.twist.covariance[5] = vel_covar(0,2);
	odom_msg_.twist.covariance[6] = vel_covar(1,0);
	odom_msg_.twist.covariance[7] = vel_covar(1,1);
	odom_msg_.twist.covariance[11] = vel_covar(1,2);
	odom_msg_.twist.covariance[30] = vel_covar(2,0);
	odom_msg_.twist.covariance[31] = vel_covar(2,1);
	odom_msg_.twist.covariance[35] = vel_covar(2,2);

	if(publish_tf_) {
		tf_odom_.header.stamp = ros::Time::now();
		tf_odom_.transform.translation.x = pose_x_;
		tf_odom_.transform.translation.y = pose_y_;
		tf_odom_.transform.rotation = tf2::toMsg(quat);
		tf_broadcaster_.sendTransform(tf_odom_);
	}
}

double Odom::fixWrap256(unsigned char diff) {
	if(diff>128) return diff-256;
	else return diff;
}

double Odom::getWheelbase() {
	return wheelbase_;
}