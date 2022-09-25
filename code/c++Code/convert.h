#include <Eigen/Dense>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ccalib.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

using namespace sensor_msgs;
using namespace cv;
using namespace std;

/************************VARIABLES************************/
/*********************************************************/

struct img {
    double timestamp;
    int index = -1;
    bool pos = false;
    double timeDiff = 0;
    string fileName;
};

/*******************FUNCTIONS*******************/
/**
 * @brief eliminated all depth values above 2000 mm to have better visability
 * in output .png file
 * 
 * @param depth input depth image
 * @param out output depth image
 */
void cutDepthValues(Mat &in, Mat &out){
    out = cv::Mat::zeros(in.rows, in.cols, CV_16UC1);
    for (int v = 0; v < in.rows; v++){
        for (int u = 0; u < in.cols; u++){
            uint16_t d = in.at<uint16_t>(v, u);
            if(d > 4096) out.at<uint16_t>(v, u) = 0;
            else out.at<uint16_t>(v, u) = d >> 4;
        }        
    }
}

/**
 * @brief align the depth image to the rgb image. first use the transformation calculated
 * from icp. afterwards align the imag using the extrinsic transformaiton calculated from
 * calibration
 * 
 * @param depthImg input source depth image
 * @param depthImg1 input target depth image
 * @param depthInfo depth info used for pointcloud calculation
 * @param rgbInfo rgb info used for depth image alignment
 * @param img img struct elemnt for interpolating the calculated synchronization transformation
 * @param extrinsicTransform the extrinsic transformation calculated from calibration
 */
void alignToColor(Mat &depthImg, Mat depthImg1, CameraInfo depthInfo, CameraInfo rgbInfo, img img,
                    Mat extrinsicTransform){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn1 (new pcl::PointCloud<pcl::PointXYZ>());
    
    for(int v = 0; v < depthImg.rows; v++){
        for(int u = 0; u < depthImg.cols; u++){
            uint16_t d = depthImg.at<uint16_t>(v, u);
            if(d != 0){
                pcl::PointXYZ point((u - depthInfo.K[2]) * d / depthInfo.K[0],
                                    (v - depthInfo.K[5]) * d / depthInfo.K[4],
                                    d);
                cloudIn->push_back(point);
            }
            uint16_t d1 = depthImg1.at<uint16_t>(v, u);
            if(d1 != 0){
                pcl::PointXYZ point1((u - depthInfo.K[2]) * d1 / depthInfo.K[0],
                                    (v - depthInfo.K[5]) * d1 / depthInfo.K[4],
                                    d1);
                cloudIn1->push_back(point1);
            }
        }
    }


    //caculate icp transformation
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudIn1);
    pcl::PointCloud<pcl::PointXYZ> finalIcp;
    icp.align(finalIcp);
    Eigen::Matrix4f icpTransform = icp.getFinalTransformation();
    
    cout << "Bevore -- icp: " << icpTransform << "; icp has converged: " << icp.hasConverged() << endl;
    
    //calculate transformation to synchronize camera
    Eigen::Transform<float, 3, Eigen::Affine> trafo(icpTransform);
    float_t x, y, z, roll, pitch , yaw;
    pcl::getTranslationAndEulerAngles(trafo, x, y, z, roll, pitch, yaw);
    x /= 1000.0; y /= 1000.0; z /= 1000.0;
    
    roll = roll / 37 * ((int)round(img.timeDiff/10.0));
    pitch = pitch / 37 * ((int)round(img.timeDiff/10.0));
    yaw = yaw / 37 * ((int)round(img.timeDiff/10.0));
    
    x = x / 37 * ((int)round(img.timeDiff/10.0));
    y = y / 37 * ((int)round(img.timeDiff/10.0));
    z = z / 37 * ((int)round(img.timeDiff/10.0));


    Eigen::Transform<float, 3, Eigen::Affine> transformIcp;

    pcl::getTransformation(x, y, z, roll, pitch, yaw, transformIcp);

    Mat icpMat(4, 4, CV_32FC1, transformIcp.data());
    Mat icpMatTranspose = icpMat.t();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout << "icpMat: " << icpMatTranspose << endl;
    cout << "extrinsicMat: " << extrinsicTransform << endl;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    cv::Mat depthMat = Mat::zeros(3, 3, CV_32FC1);
    depthMat.at<float>(0, 0) = depthInfo.K[0]; depthMat.at<float>(0, 1) = depthInfo.K[1]; depthMat.at<float>(0, 2) = depthInfo.K[2];
    depthMat.at<float>(1, 0) = depthInfo.K[3]; depthMat.at<float>(1, 1) = depthInfo.K[4]; depthMat.at<float>(1, 2) = depthInfo.K[5];
    depthMat.at<float>(2, 0) = depthInfo.K[6]; depthMat.at<float>(2, 1) = depthInfo.K[7]; depthMat.at<float>(2, 2) = depthInfo.K[8];

    cv::Mat distCoeff = Mat::zeros(1, 5, CV_32FC1);
    for(int i = 0; i < rgbInfo.D.size(); i++) distCoeff.at<float>(0, i) = depthInfo.D.at(i);
       
    cv::Mat rgbMat = Mat::zeros(3, 3, CV_32FC1);
    rgbMat.at<float>(0, 0) = rgbInfo.K[0]; rgbMat.at<float>(0, 1) = rgbInfo.K[1]; rgbMat.at<float>(0, 2) = rgbInfo.K[2];
    rgbMat.at<float>(1, 0) = rgbInfo.K[3]; rgbMat.at<float>(1, 1) = rgbInfo.K[4]; rgbMat.at<float>(1, 2) = rgbInfo.K[5];
    rgbMat.at<float>(2, 0) = rgbInfo.K[6]; rgbMat.at<float>(2, 1) = rgbInfo.K[7]; rgbMat.at<float>(2, 2) = rgbInfo.K[8];
    
    //transform according to synchronisation
    Mat syncDepth = Mat::zeros(depthImg.rows, depthImg.cols, CV_16UC1);
    cv::rgbd::registerDepth(depthMat, depthMat, distCoeff, icpMatTranspose, depthImg, cv::Size(depthImg.cols, depthImg.rows), syncDepth, true);

    //register image with open3D transformation matrix
    Mat finalDepth = Mat::zeros(depthImg.rows, depthImg.cols, CV_16UC1);
    cv::rgbd::registerDepth(depthMat, rgbMat, distCoeff, extrinsicTransform, syncDepth, cv::Size(depthImg.cols, depthImg.rows), finalDepth, true);

    depthImg = finalDepth;
}


/**
 * @brief prepare the ros image and camera info topics. the topics are put into a rosbag
 * in the main function later on
 * 
 * @param depthImg 
 * @param depthInfo
 * @param newDepthInfo 
 * @param cvDepthImg 
 * @param rgbImg 
 * @param rgbInfo 
 * @param cvRgbImg 
 * @param img 
 * @param seqCounter 
 */
void assignTopics(Image &depthImg, CameraInfo depthInfo, CameraInfo &newDepthInfo, Mat cvDepthImg,
                        Image &rgbImg, CameraInfo &rgbInfo, Mat cvRgbImg,
                        img img, double seqCounter){

    //images
    cv_bridge::CvImage imgBridge;
        //depth
    imgBridge = cv_bridge::CvImage(depthImg.header, "16UC1", cvDepthImg);
    imgBridge.toImageMsg(depthImg);
 
    depthImg.header.stamp = ros::Time(img.timestamp/10000.0);
    depthImg.header.seq = seqCounter;
    depthImg.header.frame_id = "thermal_image_view";

        //rgb
    imgBridge = cv_bridge::CvImage(depthImg.header, "rgb8", cvRgbImg);
    imgBridge.toImageMsg(rgbImg);

    rgbImg.header.stamp = ros::Time(img.timestamp/10000.0);
    rgbImg.header.seq = seqCounter;
    rgbImg.header.frame_id = "thermal_image_view";

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //camera info
        //depth
    newDepthInfo.header.stamp = ros::Time(img.timestamp/10000.0);
    newDepthInfo.header.seq = seqCounter;
    newDepthInfo.header.frame_id = "thermal_image_view";

    newDepthInfo.height = depthImg.height; newDepthInfo.width = depthImg.width;
    newDepthInfo.distortion_model = "plumb_bob";
    for(int i = 0; i < rgbInfo.D.size(); i++) newDepthInfo.D.push_back(rgbInfo.D.at(i));
    for(int i = 0; i < 9; i++) newDepthInfo.K[i] = rgbInfo.K[i];
    for(int i = 0; i < 9; i++) newDepthInfo.R[i] = rgbInfo.R[i];
    for(int i = 0; i < 12; i++) newDepthInfo.P[i] = rgbInfo.P[i];

        //rgb
    rgbInfo.header.stamp = ros::Time(img.timestamp/10000.0);
    rgbInfo.header.seq = seqCounter;
    rgbInfo.header.frame_id = "thermal_image_view";
}
/***********************************************/
