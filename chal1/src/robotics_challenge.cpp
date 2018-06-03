#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <string>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
//JAC
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
// Costmap
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include"GlobalPlanner.h"

// Representation (RVIZ)
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>
#include <math.h>
#include <vector>
#include <fstream>

#define SPEED_CONST 0.3
#define SPEED_MAX 2
#define SPEED_MIN 0.2
#define PI 3.14159265358
#define LIMIT 10.0
#define OBSTACLE_DIST 0.65

struct point {
	float x;
	float y;
};
// estas dos tienen que ser una variable global porque una variable de la clase se llena de basura sin motivo
int status = 2; 
geometry_msgs::PointStamped statusOnePosition;
double dodgetheta = 0.0;
geometry_msgs::PointStamped stored_dodge_point;
double thetadiff = 0.0;

tf::TransformListener *tf_ptr=NULL;
ros::Publisher *map_pose_pub_ptr=NULL;

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
  //Una variable que si es 0 esta evitando (girando), si es 1, avanza 1 metro, si es 2, ya no hay obstaculo y vuelve al recorrido
  //int status;
  // Punto el el sistema de referencia global en el que el robot empezo el estatus 1
  //geometry_msgs::PointStamped statusOnePosition;

};

Turtlebot::Turtlebot()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  kinect_sub_ = nh_.subscribe("/scan",1,&Turtlebot::receiveKinect,this);
}

bool Turtlebot::command(double gx, double gy)  
{

	//RETOMAR AQUI: Coordenadas en MAp frame, pasar a odom que es lo que usa el robot
	double linear_vel=0.0;
	double angular_vel=0.0;
	bool ret_val = false;

	//Transform the goal to the local frame
	geometry_msgs::PointStamped goal;
	geometry_msgs::PointStamped base_goal;	
	
  	goal.header.frame_id = "map";

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
    		ROS_INFO("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
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

	ROS_INFO ("THETA: %.2f", theta*180/PI);
	ROS_INFO ("DISTANCIA: %.2f", d);
	ROS_INFO("STATUS PRE LOGIC: %d", status);
	
	// Si hay un obstaculo cerca, se para
	for (int i=0; i<data_scan.ranges.size(); i++) {
		if ((data_scan.ranges[i] < OBSTACLE_DIST) && (thetadiff <PI/24 && thetadiff > -PI/24) && status!=0) {
			status = 0;
			// Si es la primera vez que se detecta un obstaculo
			// RETOMAR AQUI: O BIEN NO RESETEAMOS DODGETHETA O SIEMPRE ESQUIVAMOS EN LA MISMA DIRECCION
			if (dodgetheta==0) {
				//dodgetheta += (theta>0) ? PI/12 : -PI/12;
				dodgetheta -= PI/12;
			} else {
			// Si ya se estaba esquivando pero hace falta esquivar mas
				dodgetheta += (dodgetheta>0) ? PI/12 : -PI/12;
				//dodgetheta += PI/12;	
			}
			// Convertimos dodgetheta a dodgepoint
			geometry_msgs::PointStamped dodge_point;
			dodge_point.header.frame_id = "base_link";
			dodge_point.header.stamp = ros::Time();
			dodge_point.point.x = OBSTACLE_DIST * cos(dodgetheta);
			dodge_point.point.y = OBSTACLE_DIST * sin(dodgetheta);
			dodge_point.point.z = 0.0;
			try {
				listener.transformPoint("odom", dodge_point, stored_dodge_point); 
			}catch(tf::TransformException& ex){
		    		ROS_INFO("WARNING: Received an exception trying to transform dodgepoint to storeddodgepoint: %s", ex.what());
				return ret_val;
			}
			ROS_INFO("OBSTACLE DETECTED: DODGETHETA=%.2f", dodgetheta*180/PI);
			ROS_INFO("STORED DODGEPOINT: [x=%.2f, y=%.2f]", stored_dodge_point.point.x, stored_dodge_point.point.y);
			ROS_INFO("DODGEPOINT [x=%.2f, y=%.2f]", dodge_point.point.x, dodge_point.point.y);
		}
	}
	ROS_INFO("DODGETHETA: %.2f", dodgetheta*(180/PI));
	ROS_INFO("THETADIFF: %.2f", thetadiff*(180/PI));
	// Si el robot esta esquivando un objeto, ignora el objetivo
	if (status > 1 && status < 4) {
		// Si esta desorientado, rota hacia el objetivo	
		if (theta > (LIMIT*(PI/180)) || theta < (-LIMIT*(PI/180))) {
			status = 2;			
		} else { // SI ya esta orientado, se mueve en linea recta
			status = 3;
		}
	}

	// con un switch,
	//una variable que si es 0 esta evitando (girando), si es 1, avanza 1 metro, si es 2, ya no hay obstaculo y vuelve al recorrido
	switch (status) {
		case 0: // Reorientarse para evitar obstaculo
		{	
			// Calculamos lo que queda para llegar a dodgetheta
			geometry_msgs::PointStamped base_dodge_point;
			stored_dodge_point.header.frame_id = "odom";
			stored_dodge_point.header.stamp = ros::Time();
			// Convertimos el dodgepoint guardado a base_link
			try {
				listener.transformPoint("base_link", stored_dodge_point, base_dodge_point); 
			}catch(tf::TransformException& ex){
		    		ROS_INFO("WARNING: Received an exception trying to transform dodgepoint to basedodgepoint: %s", ex.what());
				return ret_val;
			}			
			ROS_INFO("STORED DODGEPOINT: [x=%.2f, y=%.2f]", stored_dodge_point.point.x, stored_dodge_point.point.y);
			ROS_INFO("BASE_DODGEPOINT [x=%.2f, y=%.2f]", base_dodge_point.point.x, base_dodge_point.point.y);
			// Calculamos lo que queda para orientarse a base_dodgepoint			
			thetadiff = atan2(base_dodge_point.point.y, base_dodge_point.point.x);  
			angular_vel = (thetadiff<0) ? -SPEED_CONST : SPEED_CONST;
			linear_vel = 0.0;
			// Si ya esta orientado
			if ((thetadiff < PI/24) && (thetadiff > -PI/24)) {
				ROS_INFO("ORIENTED TO 1");
				status = 1;
				// statusOnePosition = guardamos el 0,0 del robot (base_link) para usarlo luego
				geometry_msgs::PointStamped zero;
				zero.header.frame_id = "base_link";
				zero.header.stamp = ros::Time();
				zero.point.x = 0.0;
				zero.point.y = 0.0;
				zero.point.z = 0.0;
				try {
					listener.transformPoint("odom", zero, statusOnePosition); 
				}catch(tf::TransformException& ex){
			    		ROS_INFO("WARNING: Received an exception trying to transform zero to statusOnePosition: %s", ex.what());
					return ret_val;
			  	} 
				ROS_INFO ("Current robot position: [%.2f, %.2f]", statusOnePosition.point.x, statusOnePosition.point.y);
			}
			ROS_INFO ("STATUS 0: Degrees to dodgetheta: %.2f", thetadiff*180/PI);
		}
		break;
		case 1: // Moverse 1 metro para evitar obstaculo
		{
			// comprobar si ha recorrido 1 metro: si robot(0,0) - statusOnePosition > 1
			geometry_msgs::PointStamped statusOnePositionRobot;
			geometry_msgs::PointStamped zero;
			zero.header.frame_id = "base_link";
			zero.header.stamp = ros::Time();
			zero.point.x = 0.0;
			zero.point.y = 0.0;
			zero.point.z = 0.0;
			//statusOnePositionRobot.header.frame_id="base_link";
			try {
				listener.transformPoint("odom", zero, statusOnePositionRobot);
			} catch(tf::TransformException& ex){
				ROS_INFO("Received an exception trying to transform zero to statusOnePositionRobot: %s", ex.what());
				return ret_val;
		  	} 
			ROS_INFO("STATUSONEPOSITION: x=%.2f, y=%.2f", statusOnePosition.point.x, statusOnePosition.point.y);
			ROS_INFO("STATUSONEPOSITIONROBOT: x=%.2f, y=%.2f", statusOnePositionRobot.point.x, statusOnePositionRobot.point.y);
			float traveledDistance = sqrt(
				pow((statusOnePosition.point.x-statusOnePositionRobot.point.x), 2) +
				pow((statusOnePosition.point.y-statusOnePositionRobot.point.y), 2)
			);
			if (traveledDistance < OBSTACLE_DIST - 0.15) {
				linear_vel = SPEED_CONST;
				angular_vel = 0;
			} else {
				status = 2;
			}
			ROS_INFO ("STATUS 1: traveled distance=%.2f", traveledDistance);
		}
		break;
		case 2: // Reorientarse hacia el objetivo
		{	
			dodgetheta = 0;
			angular_vel = (theta<=0) ? -SPEED_CONST : SPEED_CONST;
			linear_vel = 0.0;
			ROS_INFO ("STATUS 2");
		}
		break;
		case 3:  // Moverse hacia el objetivo
		{
			dodgetheta = 0;
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
	}	
	ROS_INFO("STATUS AFTER LOGIC: %d", status);
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

void publishPath(ros::Publisher& path_pub, std::vector<geometry_msgs::PoseStamped>& plan )
{
	visualization_msgs::Marker marker;
  	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "path";
	marker.type = 4;
	marker.id = 0;
	marker.action = 0;

	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b =0;
	marker.color.a = 1;
	marker.scale.x = 0.05;
	for (unsigned i = 0; i< plan.size(); i++) {
		geometry_msgs::Point p;
		p.x = plan[i].pose.position.x;
		p.y =plan[i].pose.position.y;
		p.z = 0;
		marker.points.push_back(p);
	}
	path_pub.publish(marker);	


}


void odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	geometry_msgs::PoseStamped poseIn,poseOut;
	poseIn.pose = odom->pose.pose;
	poseIn.header = odom->header;	
	poseIn.header.stamp = ros::Time(0);
	try{
		tf_ptr->transformPose("map",poseIn,poseOut);
		poseOut.header.stamp = odom->header.stamp;
		map_pose_pub_ptr->publish(poseOut);	
	}catch(std::exception &e) {
		ROS_ERROR("%s",e.what());
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotics_challenge");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	double start_x, start_y;
	double goal_x, goal_y;

	tf::TransformListener tf(ros::Duration(10));
	tf_ptr = &tf;

  	costmap_2d::Costmap2DROS cost_map("cost_map", tf);

	// (start_x, start_y) are the coordinates of the initial position of the robot in MAP frame
	pn.param<double>("start_x",start_x,3.46);
	pn.param<double>("start_y",start_y,4.62);
	
	// (goal_x, goal_y) are the coordinates of the goal in MAP frame
	pn.param<double>("goal_x",goal_x,6.0);
	pn.param<double>("goal_y",goal_y,9.0);
	
	ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("/path", 1);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1, odomReceived);
	ros::Publisher map_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/map_pose", 1);
	map_pose_pub_ptr = &map_pose_pub;

	// Start position
  	geometry_msgs::PoseStamped start;
  	start.pose.position.x = start_x;
  	start.pose.position.y = start_y;
  	start.pose.position.z = 0.0;
  	start.header.stamp = ros::Time::now();
  
  	// Goal Position
  	geometry_msgs::PoseStamped goal;
  	goal.pose.position.x = goal_x;
  	goal.pose.position.y = goal_y;
  	goal.pose.position.z = 0.0;
  	goal.header.stamp = ros::Time::now();

	// Opcional por si se quiere planificar un camino
	std::vector<geometry_msgs::PoseStamped> plan;
 	unsigned int cont_wp = 0;
	ROS_INFO("Tamanyo del plan %zu", plan.size());
  	global_planner::GlobalPlanner planner("turtle_planner", &cost_map); 
  	planner.makePlan(start, goal, plan);	

	publishPath(path_pub,plan);
	ros::Rate loop_rate(15);

  	/**
  	* Complete the code to follow the path
  	*/
	Turtlebot robot;
	bool notArrived = false;
  	while (ros::ok())
  	{
	//aqui hay que pasarle a command el goal al que queremos ir, y cuando estemos a una distancia de ese punto, pasar a siguiente
	
	//notArrived = command(plan[cont_wp].position.x, plan[cont_wp].position.y);
	notArrived = robot.command(goal.pose.position.x, goal.pose.position.y);
	if (notArrived && cont_wp < plan.size()) {
		cont_wp+=1;
		getchar();
	}

	 ros::spinOnce();
       	 loop_rate.sleep();
  	}


	return 0;
}
