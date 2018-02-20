

#include <ros/ros.h>

#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <math.h> 
#include <stdio.h>

#define SPEED_CONST 0.25
#define PI 3.14159265358
#define LIMIT 10.0
#define SLOW 0.01


/**
* Our class to control the robot
* It has members to store the robot pose, and
* methods to control the robot by publishing data
*/
class Turtlebot
{
public:
  Turtlebot();
 

  /**
   * This function should command the robot to reach the goal
   * It should compute the commands to the robot by knowing the current position
   * and the goal position
   */
  void command(double goal_x, double goal_y);

  

private:

  
  ros::NodeHandle nh_;
  

  //Publisher and subscribers
  ros::Publisher vel_pub_;
 

  //Transform listerner to obtain the transform between the world frame (odom) and the robot frame (base_link)
  tf::TransformListener listener;


  //!Publish the command to the turtlebot
  void publish(double angular_vel, double linear_vel);

  

};

Turtlebot::Turtlebot()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_control");
  Turtlebot robot;
  ros::NodeHandle n;

  if(argc<3)
  {
	std::cout << "Insuficient number of parameters" << std::endl;
	std::cout << "Usage: c1 Goal_coordinate_x Goal_coordinate_y" << std::endl;
	return 0;
  }

  double xGoal=5.0, yGoal=3.0;

  xGoal=std::atof(argv[1]);
  yGoal=std::atof(argv[2]);

  ros::Rate loop_rate(20);

  //Control loop
  while (ros::ok())
  {
   
  
    robot.command(xGoal,yGoal);

    
    loop_rate.sleep();
   
  }


  return 0;

}


void Turtlebot::command(double gx, double gy)  
{

	double linear_vel=0.0;
	double angular_vel=0.0;
	double margin = 1.0;
	

	//Transform the goal to the local frame
	geometry_msgs::PointStamped goal;
	geometry_msgs::PointStamped base_goal;
	
  	goal.header.frame_id = "odom";

  	//we'll just use the most recent transform available for our simple example
  	goal.header.stamp = ros::Time();

  	//just an arbitrary point in space
  	goal.point.x = gx;
  	goal.point.y = gy;
  	goal.point.z = 0.0;

	try{
	    	listener.transformPoint("base_link", goal, base_goal);

	    	ROS_INFO("goal: (%.2f, %.2f. %.2f) -----> base_goal: (%.2f, %.2f, %.2f) at time %.2f",
		goal.point.x, goal.point.y, goal.point.z,
		base_goal.point.x, base_goal.point.y, base_goal.point.z, base_goal.header.stamp.toSec());

  	}catch(tf::TransformException& ex){
    		ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
		return;
  	}
	
	/**
	* This should be completed. You should use a proportional controller
	* The linear velocity should be proportional to the distance to the goal
	* The angular velocity should be proportional to the difference in the angle towards
	* the goal and the current angle of the robot
	*/
	
	double x = base_goal.point.x;
	double y = base_goal.point.y;
	double d = sqrt((base_goal.point.x*base_goal.point.x)+(base_goal.point.y*base_goal.point.y));
	double theta = atan2(y, x); 
	ROS_INFO("Angulo %.2f, dist %.2f", theta*(180/PI), d);
	ROS_INFO ("ANGLE LIMIT+: %.2f", (LIMIT*(PI/180)));
	ROS_INFO ("ANGLE LIMIT-: %.2f", (-LIMIT*(PI/180)));
	ROS_INFO ("THETA: %.2f", theta);
	ROS_INFO ("THETA: %.2f", theta);

	if (d < LIMIT/100) {
		angular_vel = 0;
		linear_vel = 0;
		ROS_INFO ("DISTANCIA 0");
	} else {		
		if (theta > (LIMIT*(PI/180)) || theta < (-LIMIT*(PI/180))) {
			angular_vel = theta * SPEED_CONST;
			linear_vel = 0;
			
		} else {
			linear_vel = SPEED_CONST * d;
			angular_vel = 0;
		}
	}
   	publish(angular_vel,linear_vel);    


  return;
}



//Publish the control values to the turtlebot
void Turtlebot::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    vel_pub_.publish(vel);    


  return;
}





