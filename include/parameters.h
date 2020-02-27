#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "cloud_msgs/cloud_info.h"
#include <sensor_msgs/NavSatFix.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <tic_toc.h>

#include <fstream>
#include <math.h>

typedef pcl::PointXYZI  PointType;

typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::VectorXd VXD;
typedef Eigen::MatrixXd MXD;
typedef Eigen::Quaterniond Q4D;

using namespace std;

namespace parameter {
    
// const double G0 = 9.7803267714;//gravity   
const double G0 = 9.81;//gravity   
const double deg = M_PI/180.0;//degree      
const double rad = 180.0/M_PI;//radian 
const double dph = deg/3600.0;//degree per hour 
const double dpsh = deg/sqrt(3600.0);//degree per square-root hour 
const double mg = G0/1000.0;//mili-gravity force
const double ug = mg/1000.0;//micro-gravity force 
const double mgpsHz = mg/sqrt(1.0);//mili-gravity force per second 
const double ugpsHz = ug/sqrt(1.0);//micro-gravity force per second 
const double Re = 6378137.0;  		///< WGS84 Equatorial radius in meters
const double Rp = 6356752.31425; 
const double Ef = 1.0 / 298.257223563;
const double Wie = 7.2921151467e-5;
const double Ee = 0.0818191908425;
const double EeEe = Ee*Ee;

const bool loopClosureEnableFlag = true;
const double mappingProcessInterval = 0.3;
const float ang_res_x = 0.2;
const float ang_res_y = 2.0;
const float ang_bottom = 15.0+0.1;
const int groundScanInd = 5;
const int systemDelay = 0;
const float sensorMountAngle = 0.0;
const float segmentTheta = 1.0472;
const int segmentValidPointNum = 5;
const int segmentValidLineNum = 3;
const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
const float segmentAlphaY = ang_res_y / 180.0 * M_PI;
const int edgeFeatureNum = 2;
const int surfFeatureNum = 4;
const int sectionsTotal = 6;
const float surroundingKeyframeSearchRadius = 50.0;
const int   surroundingKeyframeSearchNum = 50;const float historyKeyframeSearchRadius = 5.0;
const int   historyKeyframeSearchNum = 25;
const float historyKeyframeFitnessScore = 0.3;
const float globalMapVisualizationSearchRadius = 500.0;

/*!@ENABLE_CALIBRATION */    
extern int CALIBARTE_IMU;
extern int SHOW_CONFIGURATION;
    
/*!@INITIAL_PARAMETERS */  
extern double IMU_LIDAR_EXTRINSIC_ANGLE;
extern double IMU_MISALIGN_ANGLE;
extern double INIT_HEADING;
extern double REF_LAT;
extern double REF_LON;
extern double REF_ALT;

/*!@LIDAR_PARAMETERS */  
extern int LINE_NUM;
extern int SCAN_NUM;
extern double SCAN_PERIOD;
extern double EDGE_THRESHOLD;
extern double SURF_THRESHOLD;
extern double NEAREST_FEATURE_SEARCH_SQ_DIST;


/*!@KALMAN_FILTER */
extern double ACC_N;
extern double ACC_W;
extern double GYR_N;
extern double GYR_W;  
extern V3D INIT_POS_STD;
extern V3D INIT_VEL_STD;
extern V3D INIT_ATT_STD;
extern V3D INIT_ACC_STD;
extern V3D INIT_GYR_STD;

/*!@EXTRINSIC_PARAMETERS*/
extern V3D LEVER_ARM;
extern V3D INIT_BA;
extern V3D INIT_BW;
extern V3D INIT_TBL;
extern Q4D INIT_RBL;

/*!@FILTER_PARAMETERS*/
extern int AVERAGE_NUMS;
extern double LOW_PASS_FILTER_RATIO;
extern double STATIONARY_THRESHOULD;

/*!@SUB_TOPIC_NAME*/
extern std::string IMU_TOPIC;
extern std::string GPS_POS_TOPIC;
extern std::string GPS_VEL_TOPIC;
extern std::string GPS_HEADING_TOPIC;
extern std::string LIDAR_TOPIC;
extern std::string MAG_HEADING_TOPIC;
extern std::string WHEEL_ODOM_TOPIC;
extern std::string LASER_ODOM_TOPIC;

/*!@PUB_TOPIC_NAME*/
extern std::string LIDAR_ODOMETRY_TOPIC;
extern std::string LIDAR_MAPPING_TOPIC;
extern std::string FUSION_ODOMETRY_TOPIC;
extern std::string GPS_ODOMETRY_TOPIC;
extern std::string IMU_ODOMETRY_TOPIC;

/*!@UTILITY_SETTINGS*/
extern int ROBOT_TYPE;
extern int FUSION_TYPE;
extern int INITIALIZATION_TYPE;
extern int USE_REF;
extern int HIGH_OUTPUT_RATE_MODE;

/*!@TESTING*/
extern int VERBOSE;
extern int ICP_FREQ;
extern int MAX_LIDAR_NUMS;
extern int NUM_ITER;
extern double LIDAR_SCALE;
extern double LIDAR_STD;

void readParameters(ros::NodeHandle &n);

void readV3D(cv::FileStorage* file, const std::string& name, V3D& vec_eigen);

void readQ4D(cv::FileStorage* file, const std::string& name, Q4D& quat_eigen);

enum StateOrder
{
    O_R = 0,
    O_P = 3,
};

}

#endif