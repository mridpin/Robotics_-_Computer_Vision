

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/CameraInfo.h>



void searchTemplate(cv::Mat &in, cv::Mat &temp, cv::Mat &out)
{

	double min, max;
	cv::Point pointMin, pointMax;

	//Create similarity matrix between the template, temp, and the input image
	cv::matchTemplate(in,temp,out,CV_TM_CCORR_NORMED);

	//Look for the maxima in the similarity matrix
	cv::minMaxLoc(out,&min,&max,&pointMin,&pointMax);

	//Once we have the maxima located, we copy the input image in the output parameter
	in.copyTo(out);

	//Draw a circle on the output image at the location of the maxima, and a rectangle with the dimensions of the template temp
	cv::circle(out,pointMax,10,cv::Scalar(0,0,255),3);
    	cv::rectangle(out,pointMax,cv::Point(pointMax.x+temp.cols,pointMax.y+temp.rows),cv::Scalar(0,0,255),3);

}





/**
* OpenCV image coordinate frame: u corresponds to the column (and the X axis), v corresponds to the rows
* of the image (and the Y axis)

cv::Vec3f transfromToWorldCoordinates(float u, float v, float depth, sensor_msgs::CameraInfo calib)
{
	float X, Y, Z;
	cv::Vec3f point3D;

	//We apply the formulae

	std::cout << "u: " << u << " v: " << v << " d: " <<  depth << std::endl;	

	X=(u-calib.K[2])*depth/calib.K[0];
	Y=(v-calib.K[5])*depth/calib.K[4];
	Z=depth;

	std::cout << "X: " << X << " Y: " << Y << " Z: " <<  Z << std::endl;

	point3D[0]=X;
	point3D[1]=Y;
	point3D[2]=Z;
	
	return point3D;

}
*/



void processImageAndDepth(cv::Mat &img, cv::Mat &depth, cv::Mat &out)
{

	//out = depth;

	cv::threshold(depth,out,1.5,255.0,cv::THRESH_BINARY_INV);

	
}






