#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

const int NO_OF_STEPS = 14;
const float STAIR_HEIGHT = 2.7;
const float STEP_HEIGHT = STAIR_HEIGHT / NO_OF_STEPS;
const float STEP_WIDTH = 1.2;
const float STEP_ROTATION = 360/NO_OF_STEPS;
const float ORIGIN = STEP_WIDTH * (2/3);

void sendTransform(void) {

	static tf::TransformBroadcaster br;

	//Name of the coordinate frame	
	//std::ostringstream frame_name;
	//std::ostringstream frame_name_new;

	//Represents the full transformation
	tf::Transform transform[NO_OF_STEPS];

	for (int i = 0; i < NO_OF_STEPS; i++) {
		//Translation
		transform[i].setOrigin(tf::Vector3(STEP_WIDTH, 0, STEP_HEIGHT));

		//Rotation
		tf::Quaternion q; //TF uses quaternions, but we can create them from roll,pitch and yaw (line below)
		q.setRPY(0, 0, STEP_ROTATION * M_PI / 180.0);  //Roll, pitch and yaw angles
		transform[i].setRotation(q);

	}
    // (STEP_ROTATION * i) * M_PI / 180.0
	//Send the transformation between "world" and "frame_0" with a time stamp
	br.sendTransform(
			tf::StampedTransform(transform[0], ros::Time::now(), "world",
					"escalon_0"));

	for (int i = 1; i < NO_OF_STEPS; i++) {
		std::ostringstream frame_name;
		std::ostringstream frame_name_new;

		frame_name << "escalon_" << i - 1;
		frame_name_new << "escalon_" << i;
		br.sendTransform(
				tf::StampedTransform(transform[i], ros::Time::now(),
						frame_name.str(), frame_name_new.str()));

	}

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "epd1_c2");

	ros::NodeHandle node;

	ros::Rate loop_rate(10);

	while (ros::ok()) {

		sendTransform();
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
;

