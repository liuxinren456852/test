#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

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

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <tic_toc.h>

#include <iostream>
#include <queue>
#include <math_utils.h>
#include <sensor_utils.hpp>
#include <StateEstimator.hpp>
#include <MapRingBuffer.h>
#include <parameters.h>


using namespace Eigen;
using namespace std;
using namespace math_utils;
using namespace sensor_utils;

namespace fusion {
    
typedef pcl::PointXYZI  PointType;
class Liom {
public:
    enum FusionMode {
	MODE_LIDAR = 0,
	MODE_IMU_LIDAR = 1,
	MODE_IMU_GPS = 2,
	MODE_IMU_LIDAR_GPS = 3,
    }; 
    
    
    Liom (ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~Liom();
    
    void run();
    void process();
    void processFusion();
    void processImuLidarFusion();
    void processImuGpsFusion();
    void processImuLidarGpsFusion();
    void processLidarOdom();
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imuIn);
    void laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    void laserCloudInfoCallback(const cloud_msgs::cloud_infoConstPtr& msgIn);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsIn);
    void outlierCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    void mapOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometryMsg);
    void initialization(); 
    void caliIMU();
    void predict();
    void alignIMU(const V3D& rpy, const V3D& acc_in, const V3D& gyr_in, V3D& acc_out, V3D& gyr_out);
    
    inline void publishCloudMsg(ros::Publisher& publisher,
				pcl::PointCloud<PointType>::Ptr cloud,
				const ros::Time& stamp,
				const std::string& frameID) {
	sensor_msgs::PointCloud2 msg;
	pcl::toROSMsg(*cloud, msg);
	msg.header.stamp = stamp;
	msg.header.frame_id = frameID;
	publisher.publish(msg);
    }	
    
    void publishLaserOdom(double timeStamp);
    void publishOdometry(double timeStamp);
    void publishOdometryYZX(double timeStamp);
    void publishImuOdometryYZX(double timeStamp);
    void publishOdometry();
    void publishGpsOdometry();
    
    
private:
    FusionMode mode = MODE_LIDAR;
    
    ros::NodeHandle nh_;			 
    ros::NodeHandle pnh_;
    
    //!@StateEstimator
    StateEstimator* estimator;
    
    //!@Subscriber
    ros::Subscriber subLaserCloud;
    ros::Subscriber subLaserCloudInfo;
    ros::Subscriber subOutlierCloud;
    ros::Subscriber subImu;
    ros::Subscriber subGPS_;
    ros::Subscriber subMapOdom_;		
    
    //!@Publishers
    ros::Publisher pubUndistortedPointCloud;

    ros::Publisher pubCornerPointsSharp;
    ros::Publisher pubCornerPointsLessSharp;
    ros::Publisher pubSurfPointsFlat;
    ros::Publisher pubSurfPointsLessFlat;

    ros::Publisher pubLaserCloudCornerLast;
    ros::Publisher pubLaserCloudSurfLast;
    ros::Publisher pubOutlierCloudLast;
	
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubIMUOdometry;
    ros::Publisher pubGpsOdometry;

    ros::Publisher pubLaserOdom;
    
    //!@PointCloudPtrs
    pcl::PointCloud<PointType>::Ptr distortedPointCloud;
    pcl::PointCloud<PointType>::Ptr outlierPointCloud;
    
    //!@Messages
    nav_msgs::Odometry laserOdom;
    nav_msgs::Odometry laserOdometry;
    nav_msgs::Odometry imuOdometry;
    nav_msgs::Odometry gpsOdometry;

    //!@Buffers
    MapRingBuffer<Imu> imuBuf_;
    MapRingBuffer<sensor_msgs::PointCloud2::ConstPtr> pclBuf_;
    MapRingBuffer<sensor_msgs::PointCloud2::ConstPtr> outlierBuf_;
    MapRingBuffer<cloud_msgs::cloud_info> cloudInfoBuf_;
    MapRingBuffer<Gps> gpsBuf_;
    
    //!@Times
    
    //!@Test
    V3D ba_tmp_;
    V3D bw_tmp_;
    V3D ba_init_;
    V3D bw_init_;
    bool isInititialized;
    bool isImuCalibrated;
    bool isRefSet;

    
    int calibrationSampleCount_ = 0;
    Gps gps_last_;
    
    GlobalState predictedState_;
    double predictedTime_;
    bool isFirstImu = true;
    V3D acc_last, gyr_last;   
    
    int lidarCounter = 0;
    double duration_;

};    
}



#endif