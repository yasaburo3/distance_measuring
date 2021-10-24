#include "../include/slamBase.h"

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera ){
    PointCloud::Ptr cloud ( new PointCloud );

    for(int m = 0; m < depth.rows; m++)
        for(int n = 0; n < depth.cols; n++){
            ushort d = depth.ptr<ushort>(m)[n];
            if(d == 0)
                continue;
            
            PointT p;
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            cloud->points.push_back( p );
        }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

// 从(u,v,d)得到三维点坐标
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera ){
    cv::Point3f p;
    p.z = double(point.z) / camera.scale;
    p.x = (point.x - camera.cx) * p.z / camera.fx;
    p.y = (point.y - camera.cy) * p.z / camera.fy;
    
    return p;
}

void computeKeyPointAndDesp( FRAME& frame ){
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect( frame.rgb, frame.kp );
    descriptor->compute( frame.rgb, frame.kp, frame.desp );
}

RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera, bool& goodMatchFlag ){

    static ParameterReader pd;
    vector< cv::DMatch > matches;
    cv::Ptr< cv::DescriptorMatcher > matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->match(frame1.desp, frame2.desp, matches);
    // cout << "find total " << matches.size() << "matches." << endl;

    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
    int min_good_match = atoi( pd.getData("min_good_match").c_str() );

    for( size_t i = 0; i < matches.size(); i++ ){
        if( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for( size_t i = 0; i < matches.size(); i++ ){
        if( matches[i].distance < good_match_threshold * minDis )
            goodMatches.push_back(matches[i]);
    }

    // cout << "good matches: " << goodMatches.size() << endl;
    if( goodMatches.size() < min_good_match ){
        RESULT_OF_PNP bad_result;
        goodMatchFlag = false;
        // cerr << "Bad Match: Discard!" << endl;
        return bad_result;
    }

    // 特征匹配效果显示
    // cv::Mat img_match;
    // cv::Mat img_good_match;
    // cv::drawMatches(frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, matches, img_match);
    // cv::drawMatches(frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, goodMatches, img_good_match);
    // cv::imshow("all matches", img_match);
    // cv::imshow("good matches", img_good_match);
    // cv::waitKey(1);

    vector<cv::Point3f> pts_obj;
    vector<cv::Point2f> pts_img;

    for(size_t i = 0; i < goodMatches.size(); i++){
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if(d == 0)
            continue;
        pts_img.push_back(frame2.kp[goodMatches[i].trainIdx].pt);

        cv::Point3f pt( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // cout << "solution of PNP:" << endl;
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec;
    // cv::solvePnP( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false);
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false,100,8.0);
    RESULT_OF_PNP result;
    result.rvec = rvec;
    result.tvec = tvec;
    // cout << "rvec:\n" << rvec << endl;
    // cout << "tvec:\n" << tvec << endl;

    return result;
}

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);             /////

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(0,1);
    T(2,3) = tvec.at<double>(0,2);
    return T;
}

PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ){
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
}

