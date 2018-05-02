

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


//This is the function to be filled in C3
void processImage_c3(cv::Mat &in, cv::Mat &out)
{
	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;

	cv::Mat o[] = { b, g, r, alpha };

	//in is a color image. We split it into the 3 colors (in order BGR) and the alpha channel:
	cv::split(in, o);

//	//From now on, we process just one of the channels. For instance, o[2] is the red channel
	cv::threshold(o[2], r, 110.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[1], g, 50.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[0], b, 50.0, 255.0, cv::THRESH_BINARY);
	cv::Mat gb;
	cv::Mat nob;
	cv::Mat nog;
	cv::bitwise_not(b, nob);
	cv::bitwise_not(g, nog);
	cv::Mat aux;
	cv::bitwise_and(nob, nog, aux);
	cv::bitwise_and(aux, r, out);
	cv::erode (out, out, cv::Mat());
	cv::dilate(out, out, cv::Mat());
	
}


//This is the function to be filled in C4
void processImageColor_c4(cv::Mat &in, cv::Mat &out)
{

	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;

	cv::Mat o[] = { b, g, r, alpha };

	//in is a color image. We split it into the 3 colors (in order BGR) and the alpha channel:
	cv::split(in, o);

//	//From now on, we process just one of the channels. For instance, o[2] is the red channel
	cv::threshold(o[2], r, 15.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[1], g, 5.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[0], b, 10.0, 255.0, cv::THRESH_BINARY);
	//cv::Mat gb;
	cv::Mat nob;
	//cv::Mat nog;
	cv::bitwise_not(b, nob);
	cv::Mat aux;
	cv::bitwise_and(r, g, aux);
	cv::bitwise_and(aux, nob, out);
	cv::erode (out, out, cv::Mat());
	cv::erode (out, out, cv::Mat());
	cv::dilate(out, out, cv::Mat());
	cv::dilate(out, out, cv::Mat());

	
}
