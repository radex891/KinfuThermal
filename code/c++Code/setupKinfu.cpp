#include <ros/ros.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <fstream> 

#include "../include/convert.h"

using namespace std;
using namespace sensor_msgs;
using namespace cv;
using namespace boost::filesystem;

/*****************VARIABLES*****************/
string m_rgbDir = "/undistorted_optris_images/", m_rgbImgPath;
string m_depthDir = "/rs_depth_images/", m_depthImgPath;

string m_rgbInfo = "/csv_camera_info/optris.txt", m_rgbInfoPath;
string m_depthInfo = "/csv_camera_info/realsense.txt", m_depthInfoPath;

string m_extrinsicTransform = "/csv_camera_info/transformation_matrix.txt", m_extrinsicTransformPath;

string m_bagPath, m_imagePath = "/image_";

const int m_queueSize = 1000;
/*******************************************/

/************************FUNCTIONS************************/
/**
 * @brief read the extrinsic transformation from file to opencv mat
 * 
 * @param transform the opencv extrinsic transform mat
 */
void readExtrinsicTransform(Mat &transform){
    std::ifstream file(m_extrinsicTransformPath.c_str());
    string line;
    while(std::getline(file, line)){
        vector<string> content;
        line.erase(line.size()-1, line.size());
        boost::split(content, line, boost::is_any_of(" "));

        vector<string>::iterator it = remove_if(content.begin(), content.end(), mem_fun_ref(&string::empty));
        content.erase(it, content.end());

        for(int i = 1; i < content.size(); i++){
            int ind = content.at(i).find(",");
            if(ind > -1)content.at(i).erase(ind, content.at(i).length());
            
            boost::trim_right(content.at(i));
            boost::trim_left(content.at(i));
        }

        transform.at<float>(0, 0) = stof(content.at(1));  transform.at<float>(0, 1) = stof(content.at(2)); transform.at<float>(0, 2) = stof(content.at(3));
        transform.at<float>(0, 3) = -stof(content.at(4));

        transform.at<float>(1, 0) = stof(content.at(5));  transform.at<float>(1, 1) = stof(content.at(6));  transform.at<float>(1, 2) = stof(content.at(7));
        transform.at<float>(1, 3) = -stof(content.at(8));

        transform.at<float>(2, 0) = stof(content.at(9));  transform.at<float>(2, 1) = stof(content.at(10)); transform.at<float>(2, 2) = stof(content.at(11));
        transform.at<float>(2, 3) = -stof(content.at(12));
        
        transform.at<float>(3, 0) = stof(content.at(13)); transform.at<float>(3, 1) = stof(content.at(14)); transform.at<float>(3, 2) = stof(content.at(15));
        transform.at<float>(3, 3) = stof(content.at(16));
    }
}

/**
 * @brief Read the camera info from file to ros cameraInfo topic
 * 
 * @param info the ros camera info topic
 * @param filePath path to the camear info file
 */
void parseInfo(CameraInfo &info, string filePath){
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
 * @brief read camera infos for depth and rgb camear
 * 
 * @param depthInfo 
 * @param rgbInfo 
 */
void readCameraInfos(CameraInfo &depthInfo, CameraInfo &rgbInfo){
    parseInfo(rgbInfo, m_rgbInfoPath);
    parseInfo(depthInfo, m_depthInfoPath);
}

/**
 * @brief parse depth image flie names
 * 
 * @param content vector for all filenames 
 * @param fileName the complete path to the file
 * @return img the img struct
 */
img parseImgFileName(vector<string> content, string fileName){
    img img;

    img.fileName = fileName;

    for(int i = 0 ; i < content.size(); i++){
        if(i == 0){
            img.timestamp = stod(content.at(i));
            continue;
        }

        if(content.at(i) == "index"){
            img.index = stoi(content.at(i+1));
            i++;
            continue;
        }

        if(content.at(i) == "pos" || content.at(i) == "neg"){
            img.timeDiff = stod(content.at(i+1));
            if(content.at(i) == "pos") img.pos = true;
            i++;
            continue;
        }
    }
    return img;
}


/**
 * @brief split the file name according to all underbars
 * 
 * @param depthImgs vector of all depth image file names
 * @param rgbImgs vector of all rgb image file names
 */
void readImgs(vector<img> &depthImgs, vector<img> &rgbImgs){
    //rgb
    vector<path> rgbPaths;
    copy(directory_iterator(m_rgbImgPath), directory_iterator(), back_inserter(rgbPaths));
    sort(rgbPaths.begin(), rgbPaths.end());

    for(path p: rgbPaths){
        string filename = p.filename().string();
        int ind = filename.find(".jpg");
        filename.erase(ind, filename.length());

        vector<string> content;
        boost::split(content, filename, boost::is_any_of("_"));

        img img = parseImgFileName(content, p.filename().string());
        cout << filename << endl;
        rgbImgs.push_back(img);
    }

    //depth
    vector<path> depthPaths;
    copy(directory_iterator(m_depthImgPath), directory_iterator(), back_inserter(depthPaths));
    sort(depthPaths.begin(), depthPaths.end());

    for(path p: depthPaths){
        string filename = p.filename().string();
        int ind = filename.find(".png");
        filename.erase(ind, filename.length());

        vector<string> content;
        boost::split(content, filename, boost::is_any_of("_"));

        img img = parseImgFileName(content, p.filename().string());
        cout << filename << endl;
        depthImgs.push_back(img);
    }

}

/**
 * @brief find eht corresponding depth image to each rgb image, based on the index
 * 
 * @param depth0 put the each image from its file to opencv mat for icp transform depth0
 * @param depth1 put the each image from its file to opencv mat for icp transform depth1
 * @param rgb the struct element for rgb
 * @param depthImgs vector of all img struct element for depth imags
 * @param rgbImg the opencv rgb image
 * @param j the j-th depth image
 * 
 * @return true if it has found the an depth image with the corresponding rgb image with
 * the same index, otherwise false
 */
bool foundImages(Mat &depth0, Mat &depth1, Mat &rgb, vector<img> depthImgs, img rgbImg, int &j){
    while(depthImgs.at(j).index != rgbImg.index){
        j++;
        if(j >= depthImgs.size() || (j+1) >=depthImgs.size()) return false;
    }

    if(j == 0 && depthImgs.at(j).pos == false) return false;
    else if(j == depthImgs.size() && depthImgs.at(j-1).pos == true) return false;

    depth0 = cv::imread(m_depthImgPath + depthImgs.at(j).fileName, CV_16UC1);

    if(depthImgs.at(j).pos) depth1 = cv::imread(m_depthImgPath + depthImgs.at(j+1).fileName, CV_16UC1);
    else depth1 = cv::imread(m_depthImgPath + depthImgs.at(j-1).fileName, CV_16UC1);

    rgb = cv::imread(m_rgbImgPath + rgbImg.fileName);

    cout << "\n\nrgbIndex: " << rgbImg.index << " depthIndex: " << depthImgs.at(j).index << endl;

    return true;
}

/**
 * @brief read user input
 * 
 * @param nodeHandle reference to ros nodeHandle
 * 
 * @return true if required user input is successfull, otherwise false
 */
bool readUserInput(ros::NodeHandle &nodeHandle){
    string dataDir;
    if(!nodeHandle.getParam("dataDir", dataDir)){
        ROS_ERROR("Missing data Dir");
        return false;
    }

    m_rgbImgPath = dataDir + m_rgbDir;
    m_depthImgPath = dataDir + m_depthDir;
    m_rgbInfoPath = dataDir + m_rgbInfo;
    m_depthInfoPath = dataDir + m_depthInfo;

    m_extrinsicTransformPath = dataDir + m_extrinsicTransform;

    m_imagePath = dataDir + m_imagePath;
    m_bagPath = dataDir + "/out.bag";

    return true;
}
/*********************************************************/

/***************MAIN***************/
int main(int argc, char** argv){
    ros::init(argc, argv, "setupKinfu");

    ros::NodeHandle setupKinfu("~");
    ROS_INFO("Setup Kinfu Node: Ready to deliver");
    ROS_INFO("");

    if(!readUserInput(setupKinfu)) return -1;

    ros::Publisher depthInfoPub = setupKinfu.advertise<CameraInfo>("/camera/depth/info", m_queueSize);
    ros::Publisher depthImgPub = setupKinfu.advertise<Image>("/camera/depth/img", m_queueSize);
    ros::Publisher rgbImgPub = setupKinfu.advertise<Image>("/optris/img", m_queueSize);

    vector<img> depthImgs;    CameraInfo depthInfo;
    vector<img> rgbImgs;   CameraInfo rgbInfo;

    readCameraInfos(depthInfo, rgbInfo);
    readImgs(depthImgs, rgbImgs);

    Mat extrinsicTransform = Mat::zeros(4, 4, CV_32FC1);

    readExtrinsicTransform(extrinsicTransform);

    rosbag::Bag outputBag;
    outputBag.open(m_bagPath, rosbag::bagmode::Write);
    int j = 0, image = 0; //for depth
    for(int i = 0; i < rgbImgs.size(); i++){ //for rgb

        if(rgbImgs.at(i).index > -1){
            Mat depth0, depth1, rgb;

            if(foundImages(depth0, depth1, rgb, depthImgs, rgbImgs.at(i), j)){
                ROS_INFO("Embedd Message: %d", image);

                alignToColor(depth0, depth1, depthInfo, rgbInfo, depthImgs.at(j),
                                extrinsicTransform);
                
                Mat newImg; cutDepthValues(depth0, newImg);
                Mat greyRGB; cv::cvtColor(newImg, greyRGB, CV_GRAY2RGB);
                Mat combined; cv::addWeighted(greyRGB, 0.5, rgb, 0.5, 0, combined, 0);
                imwrite(m_imagePath + to_string(image) + ".png", combined);

                Image depthImg, rgbImg; CameraInfo newDepthInfo;
                assignTopics(depthImg, depthInfo, newDepthInfo, depth0,
                                rgbImg, rgbInfo, rgb,
                                rgbImgs.at(i), image);

                depthInfoPub.publish(newDepthInfo);
                depthImgPub.publish(depthImg);
                rgbImgPub.publish(rgbImg);

                outputBag.write("/camera/depth/info", newDepthInfo.header.stamp, newDepthInfo);
                outputBag.write("/camera/depth/img", depthImg.header.stamp, depthImg);
                outputBag.write("/optris/img", rgbImg.header.stamp, rgbImg);

                image++;
            }
        }
    }
    outputBag.close();

    return 0;
}
/**********************************/
