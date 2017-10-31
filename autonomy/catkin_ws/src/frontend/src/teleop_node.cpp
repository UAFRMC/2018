#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class telop_node
{
public:
	telop_node(ros::NodeHandle nh, ros::NodeHandle pr_nh);
private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh_;
	ros::NodeHandle pr_nh_;

	int linear_axis,angular_axis;
	double linear_scale,angular_scale;

	ros::Publisher velocity_pub;
	ros::Subscriber joy_sub;

};

telop_node::telop_node(ros::NodeHandle nh, ros::NodeHandle pr_nh) 
	: nh_(nh),pr_nh_(pr_nh),linear_axis(1),angular_axis(0),linear_scale(1),angular_scale(1)
{
	pr_nh_.param("axis_linear",linear_axis,linear_axis);
	pr_nh_.param("axis_angular",angular_axis,angular_axis);
	pr_nh_.param("scale_linear",linear_scale,linear_scale);
	pr_nh_.param("scale_linear",angular_scale,angular_scale);

	velocity_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
	joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy",1,&telop_node::joyCallback,this);
}

void telop_node::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	twist.angular.z = angular_scale*joy->axes[angular_axis];
	twist.linear.x = linear_scale*joy->axes[linear_axis];
	velocity_pub.publish(twist);
}

int main(int argc, char ** argv)
{
	// Setup the ROS node
	ros::init(argc,argv,"telop_AuroraRobotics");
	ros::NodeHandle nh;
	ros::NodeHandle pr_nh("~"); // Private node handle to get params
	telop_node driver(nh, pr_nh);

	ros::spin();
}
