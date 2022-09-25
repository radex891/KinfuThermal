#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/algorithm/string.hpp>

#include <string>
#include <vector>

#include "../include/convert.h"

using namespace std;
using namespace sensor_msgs;
using namespace cv;
using namespace boost::filesystem;

/*****************VARIABLES*****************/
string m_depthInfoPath, m_depthPath;

Eigen::Matrix4f m_icpTransform = Eigen::Matrix4f::Identity();
vector<string> m_depthPaths;
/*******************************************/


/************************FUNCTIONS************************/
/**
 * @brief eliminated all depth values above 2000 mm to have better visability
 * in output .png file
 * 
 * @param depth depth image
 */
void cutDepthValues(Mat &depth){
    for(int v = 0; v < depth.rows; v++){
        for(int u = 0; u < depth.cols; u++){
            uint16_t d = depth.at<uint16_t>(v, u);
            if(d > 2000) depth.at<uint16_t>(v, u) = 0;
        }
    } 
}

/**
 * @brief use the final icp transformation on the depth image
 * 
 * @param depth0 depth image to transform
 * @param depthInfo camera info used fro transformation
 * @param i split the complete transformaiton into the i-th part up to six(i = 0 --> i = 6)
 */
void transform(Mat depth0, CameraInfo depthInfo, int i){

    Eigen::Transform<float, 3, Eigen::Affine> trafo(m_icpTransform);
    float_t x, y, z, roll, pitch , yaw;
    pcl::getTranslationAndEulerAngles(trafo, x, y, z, roll, pitch, yaw);
    x /= 1000.0; y /= 1000.0; z /= 1000.0;
    
    roll = roll / 6 * i;
    pitch = pitch / 6 * i;
    yaw = yaw / 6 * i;
    
    x = x / 6 * i;
    y = y / 6 * i;
    z = z / 6 * i;

    Eigen::Transform<float, 3, Eigen::Affine> transformIcp;

    pcl::getTransformation(x, y, z, roll, pitch, yaw, transformIcp);

    Mat icpMat(4, 4, CV_32FC1, transformIcp.data());
    Mat icpMatTranspose = icpMat.t();

    Mat depthMat = Mat::zeros(3, 3, CV_32FC1);
    depthMat.at<float>(0, 0) = depthInfo.K[0]; depthMat.at<float>(0, 1) = depthInfo.K[1]; depthMat.at<float>(0, 2) = depthInfo.K[2];
    depthMat.at<float>(1, 0) = depthInfo.K[3]; depthMat.at<float>(1, 1) = depthInfo.K[4]; depthMat.at<float>(1, 2) = depthInfo.K[5];
    depthMat.at<float>(2, 0) = depthInfo.K[6]; depthMat.at<float>(2, 1) = depthInfo.K[7]; depthMat.at<float>(2, 2) = depthInfo.K[8];

    cv::Mat distCoeff = Mat::zeros(1, 5, CV_32FC1);
    for(int i = 0; i < depthInfo.D.size(); i++) distCoeff.at<float>(0, i) = depthInfo.D.at(i);

    Mat newDepth = Mat::zeros(depth0.rows, depth0.cols, CV_16UC1);
    cv::rgbd::registerDepth(depthMat, depthMat, distCoeff, icpMat, depth0, cv::Size(depth0.cols, depth0.rows), newDepth, true);

    cutDepthValues(newDepth);
    double min, max;
    cv::minMaxIdx(newDepth, &min, &max);
    Mat adjDepthImg;
    cv::convertScaleAbs(newDepth, adjDepthImg, 255/max);
    imwrite(m_depthPath + "/../rotMat_" + to_string(i) + ".png", adjDepthImg);
}

/**
 * @brief calculate the transformation from one depth image to another. To do this, create two point cloudds
 *  and use calculate the icp transformation between them
 * 
 * @param depthImg starting depth image
 * @param depthImg1 target depth image
 * @param depthInfo use the camera info to calculate the pointclouds
 */
void calculateTransform(Mat depthImg, Mat depthImg1, CameraInfo depthInfo){
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


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudIn1);
    pcl::PointCloud<pcl::PointXYZ> finalIcp;
    icp.align(finalIcp);
    Eigen::Matrix4f icpTransform = icp.getFinalTransformation();

    cout << "Before -- icp: " << icpTransform << "; icp has converged: " << icp.hasConverged() << endl;
    m_icpTransform = m_icpTransform * icpTransform;
}

/**
 * @brief Read the camera info from file to ros cameraInfo topic
 * 
 * @param info the ros camera info topic
 * @param filePath path to the camear info file
 */
void getInfo(CameraInfo &info, string filePath){
    std::ifstream file(filePath.c_str());
    string line; int i = 0;
    while(std::getline(file, line)){
        vector<string> content;
        line.erase(line.size()-1, line.size());
        boost::split(content, line, boost::is_any_of(" "));

        vector<string>::iterator it = remove_if(content.begin(), content.end(), mem_fun_ref(&string::empty));
        content.erase(it, content.end());

        for(int j = 1; j < content.size(); j++){
            int ind = content.at(j).find(",");
            if(ind > -1) content.at(j).erase(ind, content.at(j).length());
         
            boost::trim_right(content.at(j));
            boost::trim_left(content.at(j));
        }

        if(i == 0) info.header.frame_id = content.at(1);

        if(i == 1) info.height = stoi(content.at(1));

        if(i == 2) info.width = stoi(content.at(1));

        if(i == 3){
            for(int j = 1; j < content.size(); j++){
                info.D.push_back(stof(content.at(j)));
            }
        }

        if(i == 4){
            for(int j = 1; j < content.size(); j++){
                info.K[j-1] = stof(content.at(j));
            }
        }

        if(i == 5){
            for(int j = 1; j < content.size(); j++){
                info.R[j-1] = stof(content.at(j));
            }
        }

        if(i == 6){
            for(int j = 1; j < content.size(); j++){
                info.P[j-1] = stof(content.at(j));
            }
        }

        i++;
    }
}

/**
 * @brief read the image from file to opencv mat
 * 
 * @param depth0 
 * @param depth1 
 * @param i read the i-th image
 */
void getImgs(Mat &depth0, Mat &depth1, int &i){
    depth0 = cv::imread(m_depthPaths.at(i), CV_16UC1);
    depth1 = cv::imread(m_depthPaths.at(i+1), CV_16UC1);
}

/**
 * @brief Get the Img Paths and put the into one vector
 */
void getImgPaths(){
    vector<path> paths;
    copy(directory_iterator(m_depthPath), directory_iterator(), back_inserter(paths));
    sort(paths.begin(), paths.end());

    for(path p: paths) m_depthPaths.push_back(p.c_str());
}

/**
 * @brief read user input
 * 
 * @param nodeHandle reference to ros nodeHandle
 * 
 * @return true if required user input is successfull, otherwise false
 */
bool readUserInput(ros::NodeHandle &nodeHandle){
    if(!nodeHandle.getParam("depthImgPath", m_depthPath)){
        ROS_ERROR("Missing Path for depth img");
        return false;
    }

    if(!nodeHandle.getParam("depthInfoPath", m_depthInfoPath)){
        ROS_ERROR("Missing Path for depth info");
        return false;
    }

    return true;
}
/*********************************************************/
int main(int argc, char *argv[]){
    ros::init(argc, argv, "depthSyncTransform");

    ros::NodeHandle depthSyncTransform("~");

    if(!readUserInput(depthSyncTransform)) return -1;

    CameraInfo depthInfo;
    Mat depthImg, depthImg1;

    getInfo(depthInfo, m_depthInfoPath);
    getImgPaths();

    for (int i = 100; (i+1) < 110; i++){
        Mat depth0, depth1;
        getImgs(depth0, depth1, i);
        calculateTransform(depth0, depth1, depthInfo);

        cout << "\n\n" << m_icpTransform << endl;
    }

    cout << m_icpTransform << endl;

    Mat depth0 = imread(m_depthPaths.at(100), CV_16UC1);
    Mat depth1 = imread(m_depthPaths.at(110), CV_16UC1);

    for (int i = 0; i < 7; i++)transform(depth0, depthInfo, i);

    cutDepthValues(depth0);
    cutDepthValues(depth1);

    double min, max;
    cv::minMaxIdx(depth0, &min, &max);
    Mat odepth0;
    cv::convertScaleAbs(depth0, odepth0, 255/max);
    imwrite(m_depthPath + "/../original.png", odepth0);

    cv::minMaxIdx(depth1, &min, &max);
    Mat odepth1;
    cv::convertScaleAbs(depth1, odepth1, 255/max);
    imwrite(m_depthPath + "/../original1.png", odepth1);

    return 0;
}
