

#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include "student.h"

sensor_msgs::CameraInfo cameraCalibration;

void callback(const sensor_msgs::Image::ConstPtr& colorMsg, const sensor_msgs::Image::ConstPtr& depthMsg)
{
    cv_bridge::CvImageConstPtr bColor, bDepth;
    cv::Mat color, depth;
    cv::Mat out_frame;
    

    // Convertidor de mensaje ROS a imagen OpenCV por medio de la funci�n cv_bridge
    try
    {
        bColor = cv_bridge::toCvShare(colorMsg, sensor_msgs::image_encodings::BGR8);
	//The depth image comes as a image of 16 bit pixels, encoded as unsigned ints, and with the depth in millimeters
        bDepth = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
	bColor->image.copyTo(color);
	//bDepth->image.copyTo(depth);
	bDepth->image.convertTo(depth,CV_32F,1.0/1000.0); //Convert from int to float, and from millimeters to meters
	
	// Procesamiento de im�gen  y mapa de profundidad

	processImageAndDepth(color, depth, out_frame);

	//As an example, we compute the 3D pose of the point (200,300) on the image
	//Draw a circle at that position
	cv::circle(color,cv::Point(200,300),10,cv::Scalar(0,0,255),3);
	//Transform that pixel to world coordinates using the depth from the Kinect
	//The order of storage is (row, col), that is(300,200), but the coordinates are (col, row), that is (200,300)
	transfromToWorldCoordinates(200,300,depth.at<float>(300,200),cameraCalibration);

	
	
	// Muestra imagen mono
	cv::imshow("Image", color);
	cv::imshow("Depth", depth);
	cv::imshow("out", out_frame);
	cv::waitKey(5);
}


void getCalibration(const sensor_msgs::CameraInfo::ConstPtr& calib)
{

	cameraCalibration = *calib;

}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "epd4_c1");




  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle n;




  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_disp(n, "/camera/depth_registered/image_raw", 1);

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image, sub_disp);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber calibration = n.subscribe("/camera/rgb/camera_info", 1, getCalibration); //subscribes to the Kinect video frames


    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    ros::spin();

  return 0;
}
