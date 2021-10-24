#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

#include "../include/slamBase.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int64.h"


#include <librealsense2/rs.hpp>


static const std::string RGB_WINDOW = "rgb window";
static const std::string DEPTH_WINDOW = "depth window";
static const std::string USER_WINDOW = "user window";
enum stateMachine { findingFirstPoint, findingSecondPoint, getResult, Quit };

FRAME readFrame( cv_bridge::CvImagePtr rgb_ptr, cv_bridge::CvImagePtr depth_ptr );
double normofTransform( cv::Mat rvec, cv::Mat tvec );
CAMERA_INTRINSIC_PARAMETERS getDefaultCamera( ParameterReader& pd );
void callback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg, ParameterReader& pd, ros::Publisher& pose_pub, int* ps );
void kb_callback(std_msgs::Int64ConstPtr input, int* ps);

/*******************   MAIN   ************************/

int main(int argc, char** argv){
    ParameterReader pd("/home/yasaburo3/project/rs_ws/src/MotionSolver/param/parameter.txt");
    static int stateFlag = stateMachine::findingFirstPoint;
    int *ps = &stateFlag;

    // ROS部分
    ros::init(argc, argv, "motion_solver");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> sync(rgb_sub, depth_sub, 10);
    
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_odometry", 1);
    ros::Subscriber kb_sub = nh.subscribe<std_msgs::Int64>("/key_pressed", 100, boost::bind(&kb_callback, _1, ps));
    sync.registerCallback(boost::bind(&callback, _1, _2, pd, pose_pub, ps));

    ros::spin();

    return 0;
}

void kb_callback(std_msgs::Int64ConstPtr input, int* ps)
{
    // cout << "dataGet" << input->data << endl;
    if(input->data == 101 || input->data == 69){
        
        if(*ps == stateMachine::findingFirstPoint)
            *ps = stateMachine::findingSecondPoint;
        else if(*ps == stateMachine::findingSecondPoint)
            *ps = stateMachine::getResult;
        else if(*ps == stateMachine::getResult)
            *ps = stateMachine::findingFirstPoint;
    }
    else if(input->data == 113 || input->data == 81){
        *ps = stateMachine::Quit;
    }
    // cout << "CHANGE" << *ps << endl;
}

void callback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg, ParameterReader& pd, ros::Publisher& pose_pub, int* ps )
{   
    static bool initializeFlag = 1;          // 该标志位为1时表示第一次进入回调，执行初始化
    
    static FRAME lastFrame, currFrame;

    cv_bridge::CvImagePtr rgb_ptr;
    cv_bridge::CvImagePtr depth_ptr;
    rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);

    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera(pd);
    double max_norm = atof( pd.getData("max_norm").c_str() );

    static Eigen::Matrix3d RR = Eigen::Matrix3d::Identity();
    static cv::Mat TT = (cv::Mat_<double>(3,1) << 0.0, 0.0, 0.0 );

    static pcl::visualization::CloudViewer viewer("viewer");
    PointCloud::Ptr cloud = image2PointCloud(rgb_ptr->image, depth_ptr->image, camera);
    viewer.showCloud(cloud);
    
    // // Initialization
    // if(initializeFlag)
    // {
    //     lastFrame = readFrame(rgb_ptr, depth_ptr);
    //     computeKeyPointAndDesp(lastFrame);
    //     initializeFlag = 0;
    //     return;
    // }
    // currFrame = readFrame(rgb_ptr, depth_ptr);
    // computeKeyPointAndDesp(currFrame);

    // bool goodMatchFlag = true;
    // RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera, goodMatchFlag );
    // if( goodMatchFlag == false ){
    //     // lastFrame = currFrame;          // 如果匹配失败，那就不应该更新帧了，因为这个帧要留给后面的算
    //     // 但问题是，如果运动速度过快导致前后两帧接不上那就难搞了，所以可能需要一个机制来更新
    //     return;
    // }
    // double norm = normofTransform( result.rvec, result.tvec );
    // // cout << "norm = " << endl;
    // if( norm >= max_norm ){
    //     // lastFrame = currFrame;
    //     return;
    // }
    // Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );


    // // 发布里程计计算出的位姿
    // geometry_msgs::PoseStamped poseOutput;
    // poseOutput.header.stamp = ros::Time::now();
    // TT = TT + result.tvec;
    // poseOutput.pose.position.x = TT.at<double>(0, 0);
    // poseOutput.pose.position.y = TT.at<double>(0, 1);
    // poseOutput.pose.position.z = TT.at<double>(0, 2);

    // cv::Mat R;
    // cv::Rodrigues( result.rvec, R );
    // Eigen::Matrix3d r;
    // cv::cv2eigen( R, r );
    // RR = RR * r;

    // Eigen::Quaterniond Quater(RR);

    // poseOutput.pose.orientation.w = Quater.w();
    // poseOutput.pose.orientation.x = Quater.x();
    // poseOutput.pose.orientation.y = Quater.y();
    // poseOutput.pose.orientation.z = Quater.z();

    // poseOutput.header.frame_id = "camera_link";

    // pose_pub.publish(poseOutput);



    // // cout << "T=" << T.matrix() << endl;

    // cv::Mat UserImg = currFrame.rgb;
    // auto UserImg_W = UserImg.cols;
    // auto UserImg_H = UserImg.rows;
    // int ROI_size = 10;
    // cv::Rect ROI( UserImg_W/2 - ROI_size, UserImg_H/2 - ROI_size , 2*ROI_size, 2*ROI_size);
    // cv::rectangle( UserImg, ROI, cv::Scalar(255,0,0), 1, cv::LINE_8, 0 );

    // // 计算ROI中的平均深度
    // uint16_t ROI_depth = 0;
    // uint16_t ROI_valid_size = 0;
    
    // ROI_depth = currFrame.depth.at< uint16_t>(UserImg_W/2, UserImg_H/2);
    // if(ROI_depth == 0) return;
    // cout << "depth of ROI is: " << ROI_depth << endl;
    

    // cv::Point3f tmpPoint;
    // static cv::Point3f startPoint, secondPoint;
    // static int lastPointFlag = stateMachine::findingFirstPoint;
    // double measureResult = 0;
    // static Eigen::Vector3d secondPointEigen, TransferedPoint;
    // static Eigen::Matrix3d startRR, secondRR;
    // static cv::Mat startTT, secondTT;
    // if(*ps == stateMachine::findingSecondPoint && lastPointFlag == stateMachine::findingFirstPoint){
    //     tmpPoint.x = UserImg_W/2;
    //     tmpPoint.y = UserImg_H/2;
    //     tmpPoint.z = ROI_depth;
    //     startPoint = point2dTo3d(tmpPoint, camera);
    //     startRR = RR;
    //     startTT = TT;
    //     cout << "first point is " << startPoint << endl;
    // }
    // else if(*ps == stateMachine::getResult && lastPointFlag == stateMachine::findingSecondPoint){
    //     tmpPoint.x = UserImg_W/2;
    //     tmpPoint.y = UserImg_H/2;
    //     tmpPoint.z = ROI_depth;
    //     secondPoint = point2dTo3d(tmpPoint, camera);

    //     cout << "second point is " << secondPoint << endl;

    //     secondPointEigen[0, 0] = secondPoint.x;
    //     secondPointEigen[0, 1] = secondPoint.y;
    //     secondPointEigen[0, 2] = secondPoint.z;

    //     secondRR = RR;
    //     secondTT = TT;
    //     TransferedPoint = secondPointEigen.transpose() * secondRR.inverse() * startRR;
        
    //     TransferedPoint[0, 0] -= secondTT.at<double>(0,0);
    //     TransferedPoint[0, 1] -= secondTT.at<double>(0,1);
    //     TransferedPoint[0, 2] -= secondTT.at<double>(0,2);
    //     TransferedPoint[0, 0] += startTT.at<double>(0,0);
    //     TransferedPoint[0, 1] += startTT.at<double>(0,1);
    //     TransferedPoint[0, 2] += startTT.at<double>(0,2);
    //     cout << "transfered point is " << TransferedPoint.matrix() << endl;
        
    //     measureResult += (startPoint.x - TransferedPoint[0, 0])*(startPoint.x - TransferedPoint[0, 0]);
    //     measureResult += (startPoint.y - TransferedPoint[0, 1])*(startPoint.y - TransferedPoint[0, 1]);
    //     measureResult += (startPoint.z - TransferedPoint[0, 2])*(startPoint.z - TransferedPoint[0, 2]);
    //     measureResult = sqrt(measureResult);
    //     cout << "result is " << (measureResult*100) << " cm " << endl;

    // }
    

    // lastPointFlag = *ps;

    // string words;   
    // static string resultLenth;
    // switch(*ps){
    //     case stateMachine::findingFirstPoint:
    //         words = "Press E to create start point for measurement.";
    //         break;

    //     case stateMachine::findingSecondPoint:
    //         words = "Press E to select second point for measurement.";

    //         break;

    //     case stateMachine::getResult:
            
    //         if(measureResult != 0){
    //             resultLenth = to_string(measureResult*100);
    //         }
    //         words = "Result of Measurement is: " + resultLenth + "cm";
    //         break;
        
    //     default:
    //         break;
    // }

    // // cout << "StateFlag" << *ps << endl;
    // cv::putText(UserImg, words, cv::Point(0,50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    

    // cv::imshow("User", UserImg);

    // cv::waitKey(1);


    
            


    // lastFrame = currFrame;



    // cv::imshow(RGB_WINDOW, rgb_ptr->image);
    // cv::imshow(DEPTH_WINDOW, depth_ptr->image);
    // cv::waitKey(3);
}


FRAME readFrame( cv_bridge::CvImagePtr rgb_ptr, cv_bridge::CvImagePtr depth_ptr ){
    FRAME f;
    f.rgb = rgb_ptr->image;
    f.depth = depth_ptr->image;
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec ){
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+fabs(cv::norm(tvec));
}

CAMERA_INTRINSIC_PARAMETERS getDefaultCamera( ParameterReader& pd ){
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = stof(pd.getData("camera.cx").c_str());
    camera.cy = stof(pd.getData("camera.cy").c_str());
    camera.fx = stof(pd.getData("camera.fx").c_str());
    camera.fy = stof(pd.getData("camera.fy").c_str());
    camera.scale = stof(pd.getData("camera.scale").c_str());

    return camera;
}