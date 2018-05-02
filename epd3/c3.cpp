#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/LaserScan.h>

#include <stdio.h>

#include <math.h>
#include <vector>
#include <fstream>

#define SPEED_CONST 0.25
#define SPEED_MAX 2
#define SPEED_MIN 0.2
#define PI 3.14159265358
#define LIMIT 10.0
#define OBSTACLE_DIST 1.0

// Representation (RVIZ)
#include <visualization_msgs/Marker.h>

struct point {
	float x;
	float y;
};

/**
* Our class to control the robot
* It has members to store the robot pose, and
* methods to control the robot by publishing data
*/
class Turtlebot
{
public:
  Turtlebot();
 
  /*
   * This function should command the robot to reach the goal
   * It should compute the commands to the robot by knowing the current position
   * and the goal position.
   * This function will return true only if the goal has been reached.
   */
  bool command(double goal_x, double goal_y);

private:

  
  ros::NodeHandle nh_;
  
  //2D robot pose
  double x,y,theta;
  // Scan
  sensor_msgs::LaserScan data_scan;
  ros::Subscriber kinect_sub_;
  
  //Transform listerner to obtain the transform between the world frame (odom) and the robot frame (base_link)
  tf::TransformListener listener;

  //Publisher and subscribers
  ros::Publisher vel_pub_;
 
  //!Publish the command to the turtlebot
  void publish(double angular_vel, double linear_vel);
  
  //!Callback for kinect
  void receiveKinect(const sensor_msgs::LaserScan & laser_kinect);

  // Punto leido por el laser despues de ser convertido desde angulo
  point puntos[];
  float obstacle_angle;
  bool obstacle = false;
  //Una variable que si es 0 esta evitando (girando), si es 1, avanza 1 metro, si es 2, ya no hay obstaculo y vuelve al recorrido
  int status = 2;
  // Punto el el sistema de referencia global en el que el robot empezo el estatus 1
  geometry_msgs::PointStamped statusOnePosition;

};

Turtlebot::Turtlebot()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  kinect_sub_ = nh_.subscribe("/scan",1,&Turtlebot::receiveKinect,this);

}

bool Turtlebot::command(double gx, double gy)  
{

	double linear_vel=0.0;
	double angular_vel=0.0;
	bool ret_val = false;

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
		return ret_val;
  	}
	
	/**
	* This should be completed. You should use a proportional controller
	* The linear velocity should be proportional to the distance to the goal
	* The angular velocity should be proportional to the difference in the angle towards
	* the goal and the current angle of the robot. You should check if you reached the goal
	*/
	
	double x = base_goal.point.x;
	double y = base_goal.point.y;
	double d = sqrt((base_goal.point.x*base_goal.point.x)+(base_goal.point.y*base_goal.point.y));
	double theta = atan2(y, x); 

	ROS_INFO("Angulo %.2f, dist %.2f", theta*(180/PI), d);
	//ROS_INFO ("ANGLE LIMIT+: %.2f", (LIMIT*(PI/180)));
	//ROS_INFO ("ANGLE LIMIT-: %.2f", (-LIMIT*(PI/180)));
	ROS_INFO ("THETA: %.2f", theta);
	ROS_INFO ("DISTANCIA: %.2f", d);
	//ROS_INFO ("WAYPOINT: x=%.2f, y=%.2f", gx, gy);
	//ROS_INFO ("OBSTACLE: %d", obstacle);
	
	
	// Si hay un obstaculo cerca, se para
	for (int i=0; i<data_scan.ranges.size(); i++) {
		if (data_scan.ranges[i] < OBSTACLE_DIST) {
			status = 0;
		} else {
			status = 2;
		}
	}
	
	// Si el robot esta esquivando un objeto, ignora el objetivo
	if (status!=1 || status!=0) {
		// Si esta desorientado, rota hacia el objetivo	
		if (theta > (LIMIT*(PI/180)) || theta < (-LIMIT*(PI/180))) {
			status = 2;			
		} else { // SI ya esta orientado, se mueve en linea recta
			status = 3;
		}
	}

	// con un switch, una variable que si es 0 esta evitando (girando), si es 1, avanza 1 metro, si es 2, ya no hay obstaculo y vuelve al recorrido
	switch (status) {
		case 0: // Reorientarse para evitar obstaculo
		{
			angular_vel = (theta+PI/4) * SPEED_CONST;
			linear_vel = 0.0;
			// Si ya esta orientado
			if (((theta+PI/4) < (LIMIT*(PI/180))) && ((theta+PI/4) > (-LIMIT*(PI/180)))) {
				status = 1;
				// statusOnePosition = 0,0 del robot
				geometry_msgs::PointStamped zero;
				zero.point.x = 0.0;
				zero.point.y = 0.0;
				zero.point.z = 0.0;
				listener.transformPoint("base_link", zero, statusOnePosition);
				ROS_INFO ("Status 1 position: [%.2f, %.2f]", statusOnePosition.point.x, statusOnePosition.point.y);
			}
			ROS_INFO ("STATUS 0");
		}
		break;
		case 1: // Moverse 1 metro para evitar obstaculo
		{
			// comprobar si ha recorrido 1 metro: si robot(0,0) - statusOnePosition > 1
			geometry_msgs::PointStamped statusOnePositionRobot;
			listener.transformPoint("base_link", statusOnePosition, statusOnePositionRobot);
			float traveledDistance = sqrt((statusOnePositionRobot.point.x*statusOnePositionRobot.point.x)				+(statusOnePositionRobot.point.y*statusOnePositionRobot.point.y));
			if (traveledDistance < 1.0) {
				linear_vel = SPEED_CONST;
				angular_vel = 0;
			} else {
				status = 2;
			}
			ROS_INFO ("STATUS 1");
		}
		break;
		case 2: // Reorientarse hacia el objetivo
		{
			angular_vel = theta * SPEED_CONST;
			linear_vel = 0.0;
			ROS_INFO ("STATUS 2");
		}
		break;
		case 3:  // Moverse hacia el objetivo
		{
			linear_vel = SPEED_CONST;
			angular_vel = 0.0;
			ROS_INFO ("STATUS 3");
		}
		break;
		default:
			break;
	}

	// SI ya ha llegado al objetivo
	if (d < LIMIT/40) {
		angular_vel = 0;
		linear_vel = 0;
		ret_val = true;
		//ROS_INFO ("DISTANCIA 0");
	}	

        publish(angular_vel,linear_vel);
  	return ret_val;	
}



//Publish the command to the turtlebot
void Turtlebot::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

	//std::cout << "Velocidades: " << vel.linear.x << ", " << vel.angular.z << std::endl;

    vel_pub_.publish(vel);    


  return;
}

//Callback for robot position
void Turtlebot::receiveKinect(const sensor_msgs::LaserScan& msg)
{
	data_scan=msg;
	// Different variables used to detect obstacles
	// Hay que hacer una transformacion de lo que se obtiene del scan a puntos para pasarselos al command y que evite los obstculos
	for (int i=0; i<data_scan.ranges.size(); i++) {
		puntos[i].x = data_scan.ranges[i] * cos(data_scan.angle_min + i * data_scan.angle_increment);
		puntos[i].y = data_scan.ranges[i] * sin(data_scan.angle_min + i * data_scan.angle_increment);
	}

}

void visualizePlan(const std::vector<geometry_msgs::Pose> &plan, ros::Publisher &marker_pub );

std::vector<geometry_msgs::Pose> loadPlan(const char *filename);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_control");
  Turtlebot robot;
  ros::NodeHandle n;


  if(argc<2)
  {
	std::cout << "Insufficient number of parameters" << std::endl;
	std::cout << "Usage: robot_control <filename>" << std::endl;
	return 0;
  }

  std::vector<geometry_msgs::Pose> plan = loadPlan(argv[1]);
  unsigned int cont_wp = 0;
	ROS_INFO("Tamanyo del plan %d", plan.size());
  ros::Rate loop_rate(20);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 4);

  /**
  * Complete the code to follow the path,
  * calling adequately to the command function
  */
	
  bool notArrived = false;
  while (ros::ok())
  {
    ros::spinOnce();
    
    visualizePlan(plan, marker_pub );

	//aqui hay que pasarle a command el goal al que queremos ir, y cuando estemos a una distancia de ese punto, pasar a siguiente
	
	//while (notArrived && cont_wp < plan.size()) {
	//notArrived = false;
	notArrived = robot.command(plan[cont_wp].position.x, plan[cont_wp].position.y);
	if (notArrived && cont_wp < plan.size()) {
		cont_wp+=1;
		getchar();
	}
	

    
    loop_rate.sleep();
  }

  return 0;

}

std::vector<geometry_msgs::Pose> loadPlan(const char *filename) {
  std::vector<geometry_msgs::Pose> plan;
  double x,y;
  
  std::ifstream is(filename);
  
  while (is.good()) {
    is >> x;
    if (is.good()) {
      is >> y;
      geometry_msgs::Pose curr_way;
      curr_way.position.x = x;
      curr_way.position.y = y;
      plan.push_back(curr_way);
      ROS_INFO("Loaded waypoint (%f, %f).", x , y);
    }
  }
  ROS_INFO("Plan loaded successfully.");
  return plan;
}


void visualizePlan(const std::vector< geometry_msgs::Pose >& plan, ros::Publisher &marker_pub )
{
  ros::NodeHandle n;
  
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  for (unsigned int i = 0; i < plan.size(); i++) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/odom";  // This is the default fixed frame in order to show the move of the robot
    marker.header.stamp = ros::Time::now();
  
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "plan";
    marker.id = i;
  
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
  
    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
  
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = plan[i].position.x;
    marker.pose.position.y = plan[i].position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
  
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
  
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
  
    marker.lifetime = ros::Duration(); // Eternal marker
  
    // Publish the marker
    marker_pub.publish(marker);
  }
}
