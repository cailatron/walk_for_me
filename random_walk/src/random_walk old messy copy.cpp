#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
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
 commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
 
 
 // Subscribe to the simulated robot's laser scan topic and tell ROS to call
 // this->commandCallback() whenever a new message is published on that topic
 laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback, this);
 };
 
 
 // Send a velocity command
 void move(double linearVelMPS, double angularVelRadPS) {
 geometry_msgs::Twist msg; // The default constructor will set all commands to 0
 msg.linear.x = linearVelMPS;
 msg.angular.z = angularVelRadPS;
 commandPub.publish(msg);
 };
 


 // Add two (math) vectors together and return the combined angle
 float vector_addition_angle(float a_angle, float a_mag, float b_angle, float b_mag){
     float a_x = a_mag*cos(a_angle);
     float a_y = a_mag*sin(a_angle);
     float b_x = b_mag*cos(b_angle);
     float b_y = b_mag*sin(b_angle);

     float combine_x = a_x + b_x;
     float combine_y = a_y + b_y;

     //float combine_mag = sqrt ( (combine_x^2) + (combine_y^2) );
     float combine_angle = atan( combine_y / combine_x );
     return combine_angle;
 }

 // Add two (math) vectors together and return the combined magnitued
 float vector_addition_angle(float a_angle, float a_mag, float b_angle, float b_mag){
     float a_x = a_mag*cos(a_angle);
     float a_y = a_mag*sin(a_angle);
     float b_x = b_mag*cos(b_angle);
     float b_y = b_mag*sin(b_angle);

     float combine_x = a_x + b_x;
     float combine_y = a_y + b_y;

     float combine_mag = sqrt ( (combine_x^2) + (combine_y^2) );
     //float combine_angle = atan( combine_y / combine_x );
     return combine_mag;
 }


 
 // Process the incoming laser scan message
 void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (fsm == FSM_MOVE_FORWARD) {
     // Compute the average range value between MIN_SCAN_ANGLE and MAX_SCAN_ANGLE
 //
 // NOTE: ideally, the following loop should have additional checks to ensure
 // that indices are not out of bounds, by computing:
 //
 // - currAngle = msg->angle_min + msg->angle_increment*currIndex
 //
 // and then ensuring that currAngle <= msg->angle_max
 unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
 unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
 float closestRange = msg->ranges[minIndex];
 for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
  if (msg->ranges[currIndex] < closestRange) {
    closestRange = msg->ranges[currIndex];
  }
 }
 ROS_INFO_STREAM("Range: " << closestRange);
 
// make a vector a with vaules of zero
 float vect_a [2] = { 0.0, 0.0}; 
// make vecore b, does not need vaues yet
 float vect_b[2];
// make vector c to use with the attractive force
 float vect_c[2];

 for(unsigned int q = 0; q < maxIndex; q++){
   if( msg->ranges[q] <= PROXIMITY_RANGE_M){
   
   // get the angle at this range[q] by multiplying theangle ingramint by q
   float angle = msg->angle_increment * q;
   
   // save that angle to a vector b
   vect_b[0] = angle;

   // do the arithmatic at this range for vector magnitude
   if( d_safe + e_error <= msg->ranges[q] <= PROXIMITY_RANGE_M){ // NEED TO SET UP D_SAFE & E_ERROR & THE OTHER CONSTENTS	   
     float magnitude = ( a_alpha / ( (msg->ranges[q] - d_safe) ) ^2 );
   // save it to vector b aswell
     vect_b[1] = magnitude;
   }	   
   if else( msg->ranges[q] < d_safe + e_error){
   // same psuedo-code as up there, but different arithmatic 
    float magnitude = ( a_alpha / (e_error^2) );
   // save it to vector b aswell
     vect_b[1] = magnitude;
   }


   // do the arthimatic for vector a = vector a + vector b
            // this is actually slightly a bigger section than i thought but it needs to be done
	    // NEED TO ADD SOMETHING THAT CAN DO COS & SIN & TAN MATHAMATICS
   // at the very begining, i think im going to put this 'if a = 0' thing
   if ( vect_a[0] == 0.0 && vect_a[1] == 0.0){
     vect_a[0] = vect_b[0];
     vect_a[1] = vect_b[1];
   } // no need for math, just set a to b and go on making the next b 
   else {
     /* i turned all of this into funtions but im keeping these notes just in case
      * float a_x = vect_a[1]*cos(vect_a[0]);
     float a_y = vect_a[1]*sin(vect_a[0]);
     float b_x = vect_b[1]*cos(vect_b[0]);
     float b_y = vect_b[1]*sin(vect_b[0]);

     float combine_x = a_x + b_x;
     float combine_y = a_y + b_y;

     float combine_mag = sqrt ( (combine_x^2) + (combine_y^2) );
     float combine_angle = tan inverse( combine_y / combine_x);
	
     //now there is a note about some calculators only giving the inverse tan
     // to me -90 to 90, so we might need to finiangle and add 180 to the angle
     */

     // now we set these combined things to vector a to be combined with new vector b in a sec
     vect_a[0] = vector_addition_angle(vect_a[0], vect_a[1], vect_b[0], vect_b[1]);
     vect_a[1] = vector_addition_mag(vect_a[0], vect_a[1], vect_b[0], vect_b[1]);
   }

   } // closes the 'if in the radius we would like to look' if statment 
  } // closes the for loop for all of the q ranges
 

// now, we could just continue to do the maths here and just pass 'to move or not to move' to the other section
//
// [ ] somewhere previously, we need to pass in the globle positions for x and y
  float goal_x = ; // TO DO
  float goal_y = : // TO DO
  
//[old notes] can we then get the globle orientation of the robit?
//[old notes]  ?? and then calculate at what angle is the goal to the robit, so we can keep the vector addition relitive to robit

// using globle xy of goal and xy of robit, we can find the distence to goal
  float robit_x = msg.pose.pose.position.x
  float robit_y = msg.pose.pose.position.y

  float x_away = goal_x - robit_x;
  float y_away = goal_x - robit_x;

  float to_goal_angle = atan2(away_x,away_y); //i was sent this line
  float to_goal_distence = sqrt( (x_away^2) + (y_away^2) );

  // and then use that distence to calculate atractive force
  float to_goal_mag = a_alpha *( (/*something goal*/ - /*something robit*/)^2)

// save that angle and magnitue to vector c
  vect_c[0] = to_goal_angle;
  vect_c[1] = to_goal_mag;

// for simplisity sake, do we want to add vector a + c = d? or keep with a = a + c
// i think we are going with a = a + c here 
// it is at this point that i remeber that coding works with functions and i make adding vectors together into a function
//      and then call the add vectors functions on a and c
   vect_a[0] = vector_addition_angle(vect_a[0], vect_a[1], vect_b[0], vect_b[1]);
   vect_a[1] = vector_addition_mag(vect_a[0], vect_a[1], vect_b[0], vect_b[1]);


// [old notes]
// see i keep thinking that the robot works by moving this amount of degrees and then 1 meter forward
// i think the robit actualy works by moving at an agular speed and a velocity
// so now i need to convert my angle and magnitude into a speed and a speed (i think)
// i would have liked to see objects, thing, turn to a specific direction, move slightly, think, turn to another direction, ~~
// because now i think i need to turn speed for a time, and i have to caluctate the turn speed and the time




// now to convert that vector of angle and magnitude into speeds i guess?




 
 // TODO: if range is smaller than PROXIMITY_RANGE_M, update fsm and rotateStartTime,
 // and also choose a reasonable rotateDuration (keeping in mind of the value
 // of ROTATE_SPEED_RADPS)
 //
 // HINT: you can obtain the current time by calling:
 //
 // - ros::Time::now()
 //
 // HINT: you can set a ros::Duration by calling:
 //
 // - ros::Duration(DURATION_IN_SECONDS_FLOATING_POINT)
 //
 // HINT: you can generate a random number between 0 and 99 by calling:
 //
 // - rand() % 100
 //
 // see http://www.cplusplus.com/reference/clibrary/cstdlib/rand/ for more details
 /////////////////////// ANSWER CODE BEGIN ///////////////////

 /* old stuffs ==============================================================
   ros::Time start = ros::Time::now();
   while(ros::Time::now() - start < ros::Duration( rand()%10 )){
 	geometry_msgs::Twist move;

	move.linear.x = 1.1;
	move.angular.z = 0;
	
	//commandPub.publish(move);

	ros::spinOnce();
	//rate.sleep();
 };*/




 /////////////////////// ANSWER CODE END ///////////////////
   }
 }; // this ends the sensor section
 
 
 
 // Main FSM loop for ensuring that ROS messages are
 // processed in a timely manner, and also for sending
 // velocity controls to the simulated robot based on the FSM state
 void spin() {
   ros::Rate rate(10); // Specify the FSM loop rate in Hz
 while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
  // TODO: Either call:
  //
  // - move(0, ROTATE_SPEED_RADPS); // Rotate right
  //
  // or
  //
  // - move(FORWARD_SPEED_MPS, 0); // Move foward
  //
  // depending on the FSM state; also change the FSM state when appropriate
 /////////////////////// ANSWER CODE BEGIN //////////////////
 
 // see now we are at the point where we just call a move function


 // move(FORWARD_SPEED_MPS, 0);
 // move(0,ROTATE_SPEED_RADPS);


 /////////////////////// ANSWER CODE END ///////////////////

 ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
  rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
  }
 }; // this ends the move section
 
 enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
 
 // Tunable parameters
 // TODO: tune parameters as you see fit
 constexpr static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
 constexpr static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
 constexpr static float PROXIMITY_RANGE_M_DOSOMTHINBOUTIT = 1.0;
 constexpr static float PROXIMITY_RANGE_M = 3.0; // Should be smaller than sensor_msgs::LaserScan::range_max
 constexpr static double FORWARD_SPEED_MPS = 0.3;
 constexpr static double ROTATE_SPEED_RADPS = M_PI/2;



protected:
 ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
 ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
 enum FSM fsm; // Finite state machine for the random walk algorithm
 ros::Time rotateStartTime; // Start time of the rotation
 ros::Duration rotateDuration; // Duration of the rotation
};


int main(int argc, char **argv) {
 ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
 ros::NodeHandle n;
 RandomWalk walker(n); // Create new random walk object
 walker.spin(); // Execute FSM loop
 return 0;
};

