// Set URL of rosbridge server
var ros = new ROSLIB.Ros({
	url : 'ws://localhost:9090'
});

// Set up command velocity publisher
var cmd_vel_pub = new ROSLIB.Topic({
	ros : ros,
	name : 'cmd_vel',
	messageType : 'geometry_msgs/Twist'
});

function publishDriveCommands() {
	var twist = new ROSLIB.message ({
		linear : {
			x : 0.1,
			y : 0.0,
		 	z : 0.0
		},
		angular : {
			x : 0.0,
			y : 0.0,
			z : 2.0
		}
	});

	cmd_vel_pub.publish(twist);
}

ros.on('connection', function() {
	console.log('Connected to UDP Server');
});

ros.on('error', function(error) {
	console.log('Error connecting to UDP server', error);
});

ros.on('close', function() {
	console.log('Connection to UDP server closed.');
});