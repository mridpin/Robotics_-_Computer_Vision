#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


void sendTransform(void){

	static tf::TransformBroadcaster br;

	//Name of the coordinate frame	
	std::string frame_name = "frame_1";

	//Represents the full transformation
  	tf::Transform transform;

	//Translation
	transform.setOrigin( tf::Vector3(5.0, 8.0, 0.0) );

	//Rotation
	tf::Quaternion q; //TF uses quaternions, but we can create them from roll,pitch and yaw (line below)
  	q.setRPY(0, 0, 45.0*M_PI/180.0);  //Roll, pitch and yaw angles
  	transform.setRotation(q);

	//Send the transformation between "world" and "frame_1" with a time stamp
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", frame_name));
 
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "epd1_c1");

  	ros::NodeHandle node;
  
  	ros::Rate loop_rate(10);

  	while (ros::ok())
  	{

		sendTransform();
    		ros::spinOnce();

    		loop_rate.sleep();
  	}

  	return 0;
};

