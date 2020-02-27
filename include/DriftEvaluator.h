#ifndef DRIFT_EVALUATOR_H_
#define DRIFT_EVALUATOR_H_

#include <iostream>
#include <math_utils.h>
#include <sensor_utils.hpp>
#include <MapRingBuffer.h>
#include <parameters.h>

using namespace std;
using namespace math_utils;
using namespace sensor_utils;
using namespace parameter;

namespace evaluation {

class Evaluator{ 
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   enum ErrorType {
	RDM = 0,
	ATE = 1,
	KITTI = 2
    };
    
    Evaluator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
    ~Evaluator();
    
    void run();
    void process();
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg);
    void laserOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometryMsg);
    void laserMappingCallback(const nav_msgs::Odometry::ConstPtr& mappingMsg);
    void loamCallback(const nav_msgs::Odometry::ConstPtr& loamMsg);
    void getGpsOdom(V3D& rn);
    void publishGpsOdometry(const Gps& gps, const V3D& rn);
    void processGPS();
    void processLiomapGPS();
    void processLOAM();
    void getClosestOdometry(MapRingBuffer<Odometry>& buf, const double& time, Odometry& odom);

    

    
private: 

    ros::NodeHandle nh_;			 
    ros::NodeHandle pnh_;
    
    ros::Subscriber subGroundTrueGPS_;
    ros::Subscriber subGroundTrueLOAM_;	
    ros::Subscriber subOdometry_;	
    ros::Subscriber subMapping_;	
    
    ros::Publisher pubGpsOdometry_;
    nav_msgs::Odometry gpsOdometry;
    
    MapRingBuffer<Gps> gpsBuf_;
    MapRingBuffer<Odometry> odomBuf_;
    MapRingBuffer<Odometry> mappingBuf_;
    
    Gps gpsRef_;

    double sumDistance_;
    
    Eigen::Matrix3d R_yzx_to_xyz;
    Eigen::Matrix3d R_xyz_to_yzx;
    Eigen::Quaterniond Q_yzx_to_xyz;
    Eigen::Quaterniond Q_xyz_to_yzx;
   
    bool GT_GPS;
    bool GT_LOAM;
    int ERROR_METRIC_TYPE = 0;
    bool isRefSet;
    
    
    ErrorType errorType_;
    double heading_ = 0.0;
    double mappingDriftRate_ = 0, odomDriftRate_ = 0;  
    double mappingDrift_ = 0, odomDrift_ = 0;
    int PATH_LENGTH = 50.0;
    int segCounter = 0;
    
    Gps lastGps_;
    Odometry kittiOdom_;
    Odometry kittiMapping_;
    Gps kittiGps_;
    V3D kittiRn_;
    bool updateKitti_ = false;
    Odometry mappingTransform_, odomTransform_;
    bool odomFlag = false;
    int counter = 0;
    
    bool transformLidarToGps_ = true;
    V3D init_rn;
    bool first_liom_flag = true;
    int liom_counter = 0;
};





};
































































#endif