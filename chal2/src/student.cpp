

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/CameraInfo.h>

// rgb(117, 255, 1)
void processImage(cv::Mat &in, cv::Mat &out)
{
	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;

	cv::Mat o1[] = { b, g, r, alpha };

	//in is a color image. We split it into the 3 colors (in order BGR) and the alpha channel:
	cv::split(in, o1);

	//From now on, we process just one of the channels. For instance, o[2] is the red channel

	// Process red from the book
	cv::threshold(o1[2], r, 120.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o1[1], g, 80.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o1[0], b, 120.0, 255.0, cv::THRESH_BINARY);
	cv::Mat nob;
	cv::Mat nog;
	cv::Mat nobg;
	cv::bitwise_not(b, nob);
	cv::bitwise_not(g, nog);
	cv::bitwise_and(nob, nog, nobg);
	cv::bitwise_and(r, nobg, out);

	// Process teal parts from book
	cv::Mat o2[] = { b, g, r, alpha };
	cv::split(in, o2);
	cv::threshold(o2[2], r, 80.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o2[1], g, 80.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o2[0], b, 80.0, 255.0, cv::THRESH_BINARY);
	cv::Mat nor;
	cv::Mat gb;
	cv::bitwise_not(r, nor);
	cv::bitwise_and(g, b, gb);
	cv::Mat teal;
	cv::bitwise_and(nor, gb, teal);
	cv::erode (teal, teal, cv::Mat());	
	cv::dilate(teal, teal, cv::Mat());
	cv::dilate(teal, teal, cv::Mat());
	cv::dilate(teal, teal, cv::Mat());
	cv::dilate(teal, teal, cv::Mat());

	cv::erode (out, out, cv::Mat());
	cv::erode (out, out, cv::Mat());
	cv::dilate(out, out, cv::Mat());
	cv::dilate(out, out, cv::Mat());

	cv::bitwise_or(out, teal, out);
	
}

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
*/
/*
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


/**
*/
void processImageAndDepth(cv::Mat &img, cv::Mat &depth, cv::Mat &out)
{	
	// Process depth by setting a minimun and maximum threshold and ANDing them
	//cv::Mat threshold_close;
	//depth.copyTo(threshold_close);
	//cv::threshold(depth,threshold_close,0.75,255.0,cv::THRESH_BINARY);
	cv::Mat threshold_far;
	depth.copyTo(threshold_far);
	cv::threshold(depth,threshold_far,1.40,255.0,cv::THRESH_BINARY_INV);
	//cv::bitwise_and(threshold_close, threshold_far, out);
	out = threshold_far;
	out.convertTo(out, CV_8U); // Convertir out compatible con img
	// Process color
	cv::Mat colorFilter;
	processImage(img, colorFilter);
	
	// Combine both
	cv::bitwise_and(colorFilter, out, out);
}






