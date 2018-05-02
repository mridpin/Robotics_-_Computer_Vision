


#include <stdio.h>
#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "student.h"
      
 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edp2_c3"); 

 

  cv::Mat input_image = cv::imread(argv[1],0); //Read an image from file. The 0 as second argument reads it as a grayscale image
  cv::Mat output_image;

  cv::imshow("input_image", input_image); //Show a window with the input image

  processImage_c3(input_image,output_image); //This is the function to be filled

  cv::imshow("output_image", output_image); //Show a window with the output image


  cv::waitKey(); //Wait for a key

  return 0;
}
