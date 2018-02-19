class FrontendROS {
	constructor(ros_master_ip) {
		// Connect to the ROS Master
		this.ros = new ROSLIB.Ros({
			url : 'ws://' + ros_master_ip +':9090'
		});

		// Setup node callbacks
		this.ros.on('connection', function() {
			console.log('Connected to websocket Server');
		});

		this.ros.on('error', function(error) {
			console.log('Error connecting to websocket server', error);
		});

		this.ros.on('close', function() {
			console.log('Connection to websocket server closed.');
		});

		// Create Publishers and Subscribers
		this.cmd_vel_pub = new ROSLIB.Topic({
			ros : this.ros,
			name : 'cmd_vel',
			messageType : 'geometry_msgs/Twist'
		});

	}

	publishDriveCommands(command) {
		clearInterval(this.interval);
		this.interval = setInterval( function() {
			var twist = new ROSLIB.Message ({
				linear : {
					x : 0.0,
					y : 0.0,
				 	z : 0.0
				},
				angular : {
					x : 0.0,
					y : 0.0,
					z : 0.0
				}
			});

			if(command === "forward") {
				twist.linear.x = 0.25;
				twist.angular.z = 0.0;
			}
			else if(command === "backward") {
				twist.linear.x = -0.25;
				twist.angular.z = 0.0;
			}
			else if(command === "left") {
				twist.linear.x = 0.0;
				twist.angular.z = -2.0;
			}
			else if(command === "right") {
				twist.linear.x = 0.0;
				twist.angular.z = 2.0;
			}
			else if(command === "stop") {
				twist.linear.x = 0.0;
				twist.angular.z = 0.0;				
			}

			frontend_ros.cmd_vel_pub.publish(twist);			
		}, 100);
	}
}

var frontend_ros;

function initialize_ros(ip) {
	frontend_ros = new FrontendROS(ip);
}

initialize_ros("10.10.10.101")

