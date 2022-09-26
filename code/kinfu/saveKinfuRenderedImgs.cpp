#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <iostream>

using namespace std;
using namespace sensor_msgs;
using namespace cv;



int m_counter = 0, m_queueSize = 1000;
string m_saveImgPath; 


void callbackImgs(const ImageConstPtr &image){

	cv_bridge::CvImagePtr cvImagePtr;

	cvImagePtr = cv_bridge::toCvCopy(image, image->encoding);

	imwrite(m_saveImgPath + "image_" + to_string(m_counter) + ".png", cvImagePtr->image);

	cout << "saved Image to: " << m_saveImgPath << endl;

	m_counter++;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "saveKinfuRenderedImgs");
	ros::NodeHandle nodeHandle("~");

	if(!nodeHandle.getParam("saveImgPath", m_saveImgPath)){
		ROS_ERROR("Missing save dir for kinfu rendered Imgs");
		return -1;
	}

	ros::Subscriber sub = nodeHandle.subscribe("/kinfu/output/rendered_image", m_queueSize, callbackImgs);

	ros::spin();

	return 0;
}
