#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
class RandomWalk {
public:
 // Construst a new RandomWalk object and hook up this ROS node
 // to the simulated robot's velocity control and laser topics
 RandomWalk(ros::NodeHandle& nh) :
   fsm(FSM_MOVE_FORWARD),
   rotateStartTime(ros::Time::now()),
   rotateDuration(0.f) {
 // Initialize random time generator
 srand(time(NULL));
 
 
 // Advertise a new publisher for the simulated robot's velocity command topic
 // (the second argument indicates that if multiple command messages are in
 // the queue to be sent, only the last command will be sent)
 commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);//changed
 
 
 // Subscribe to the simulated robot's laser scan topic and tell ROS to call
 // this->commandCallback() whenever a new message is published on that topic
 laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback, this);
 odomSub = nh.subscribe("odom",1, &RandomWalk::odometryCallback, this);
 odomCombinedSub = nh.subscribe("robot_pose_ekf/odom_combined", 1, &RandomWalk::odCombineCallback,this); 
 };
 
 
 // Send a velocity command
 void move(double linearVelMPS, double angularVelRadPS) {
 geometry_msgs::Twist msg; // The default constructor will set all commands to 0
 msg.linear.x = linearVelMPS;
 msg.angular.z = angularVelRadPS;
 commandPub.publish(msg);
 };
 
 // =================== [ Project Four Functions ] ==========================
 void translate(double d){
	 distence = d;	   
	 fsm = FSM_MOVE_FORWARD;
 }
 void rotate_rel(double a){
 	angle = a;
	fsm = FSM_ROTATE;
 }

 // this processes the incoming odom mesages
 void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  //ROS_INFO("%s", msg->header.frame_id.c_str());
  //ROS_INFO("%f", msg->twist.twist.linear.x);
  ROS_INFO_STREAM("with just odom, x: " << msg->pose.pose.position.x);
  ROS_INFO_STREAM("with just odom, w: " << msg->pose.pose.orientation.z);
  odom_x = msg->pose.pose.position.x;
}

// this to process filter odom messages
void odCombineCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  ROS_INFO_STREAM("with filtered odom, x: " << msg->pose.pose.position.x);
  ROS_INFO_STREAM("with filtered odom, w: " << msg->pose.pose.orientation.z);
  ROS_INFO_STREAM("TEST");
}
 
 // Process the incoming laser scan message
 void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (fsm == FSM_MOVE_FORWARD) {
	unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->\
				angle_min) / msg->angle_increment);
 	unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
	float closestRange = msg->ranges[minIndex];
 	for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
 	 if (msg->ranges[currIndex] < closestRange) {
 	   closestRange = msg->ranges[currIndex];
 	 	}
	}

	ROS_INFO_STREAM("Range: " << closestRange);

	ros::Time start = ros::Time::now();
	double stuff = (distence / FORWARD_SPEED_MPS);
	ROS_INFO_STREAM(" distence / 1m speed: " << stuff);
	ROS_INFO_STREAM(ros::Time::now());
	while( start + ros::Duration(stuff) > ros::Time::now()) {
		//ros::Time nowww = ros::Time::now();
		move(FORWARD_SPEED_MPS,0);
	}
	ROS_INFO_STREAM("done: " << ros::Time::now());
	//ROS_INFO_STREAM(odom_x);
	fsm = FSM_STILL;	 
    }
  else if (fsm == FSM_STILL) {
	  move(0, 0);
	  /*unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->\
				angle_min) / msg->angle_increment);
    	  unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
	  float closestRange = msg->ranges[minIndex];
 	  for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
 	   if (msg->ranges[currIndex] < closestRange) {
 	     closestRange = msg->ranges[currIndex];
 	   	}
	  }
	ROS_INFO_STREAM("Range: " << closestRange);*/

  }
  else if (fsm == FSM_ROTATE){
	ros::Time start = ros::Time::now();
	double stuff = ( angle / ROTATE_SPEED_RADPS);
	ROS_INFO_STREAM("rad angle: " << angle);
	ROS_INFO_STREAM("rotate rad speed: " <<ROTATE_SPEED_RADPS);
	ROS_INFO_STREAM("rad angle / rotate rad speed: " << stuff);

	ROS_INFO_STREAM(ros::Time::now());
	while( start + ros::Duration(stuff) > ros::Time::now()) {
		//ros::Time nowww = ros::Time::now();
		move(0,ROTATE_SPEED_RADPS);
	}
	ROS_INFO_STREAM("done: " << ros::Time::now());
	fsm = FSM_STILL;
  }

 }; // this ends the sensor section
 
 
 
 // Main FSM loop for ensuring that ROS messages are
 // processed in a timely manner, and also for sending
 // velocity controls to the simulated robot based on the FSM state
 void spin() {
   ros::Rate rate(10); // Specify the FSM loop rate in Hz
 while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
  // TODO: Either call:
  // - move(0, ROTATE_SPEED_RADPS); // Rotate right
  // or
  // - move(FORWARD_SPEED_MPS, 0); // Move foward
  // depending on the FSM state; also change the FSM state when appropriate
 /////////////////////// ANSWER CODE BEGIN //////////////////
  if (!flag) {
 	//translate(5.0);
	rotate_rel(.5);
	flag = true;
 }
 /////////////////////// ANSWER CODE END ///////////////////
 ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
  rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
  }
 };  


 enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE, FSM_STILL};
 
 // Tunable parameters
 // TODO: tune parameters as you see fit
 constexpr static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
 constexpr static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
 constexpr static float PROXIMITY_RANGE_M_DOSOMTHINBOUTIT = 1.0;
 constexpr static float PROXIMITY_RANGE_M = 3.0; // Should be smaller than sensor_msgs::LaserScan::range_max
 constexpr static double FORWARD_SPEED_MPS = 1.0;
 constexpr static double ROTATE_SPEED_RADPS = M_PI/4; //45 deg per sec, but 0.5 radian
 bool flag = false;
 double distence = 3;
 double angle = 3.14;
 double odom_x = 14.5555;



protected:
 ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
 ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
 ros::Subscriber odomSub;
 ros::Subscriber odomCombinedSub;
 enum FSM fsm; // Finite state machine for the random walk algorithm
 ros::Time rotateStartTime; // Start time of the rotation
 ros::Duration rotateDuration; // Duration of the rotation
};


int main(int argc, char **argv) {
 ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
 //ros::init(argc, argv, "odom_sub_node");
 ros::Subscriber odomSub;
 ros::Subscriber odomCombinedSub;
 ros::NodeHandle n;
 //ros::NodeHandle nh;
 RandomWalk walker(n); // Create new random walk object
 walker.spin(); // Execute FSM loop
 return 0;
};

