#pragma once

#include <fstream>
#include <vector>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// 视觉里程计部分

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
};

struct FRAME
{
    cv::Mat rgb, depth;
    cv::Mat desp;
    vector<cv::KeyPoint> kp;
};

struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
};

class ParameterReader
{
public:
    ParameterReader( string filename = "/home/yasaburo3/project/RGBD/parameter.txt" ){
        ifstream fin( filename.c_str() );
        if (!fin){
            cerr << "parameter file does not exist." << endl;
            return;
        }
        while(!fin.eof()){
            string str;
            getline( fin, str );
            if ( str[0] == '#' ){
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }

    string getData( string key ){
        map<string, string>::iterator iter = data.find(key);
        if ( iter == data.end() )
        {
            cerr << "Parameter name " << key << " not found!" << endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }

private:
    map<string, string> data;
};

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );
void computeKeyPointAndDesp( FRAME& frame );
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera, bool& goodMatchFlag );
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera );

