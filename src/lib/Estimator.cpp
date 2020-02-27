#include <Estimator.h>

namespace fusion {
    
int Scan::scanCounter_ = 0;
    
Liom::Liom(ros::NodeHandle& nh, ros::NodeHandle& pnh):
    nh_(nh),
    pnh_(pnh) {
    
};
Liom::~Liom()
{
    delete estimator;
}

void Liom::run()
{
    initialization();
    subMapOdom_ = pnh_.subscribe<nav_msgs::Odometry>(LIDAR_MAPPING_TOPIC, 5, &Liom::mapOdometryCallback, this); 
    switch (parameter::FUSION_TYPE) {
	case 0:
	    cout << "=====================MODE_LIDAR=====================" << endl;
	    mode = MODE_LIDAR;
	    subLaserCloud = pnh_.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 2, &Liom::laserCloudCallback, this);
	    subLaserCloudInfo = pnh_.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 2, &Liom::laserCloudInfoCallback, this);
	    subOutlierCloud = pnh_.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 2, &Liom::outlierCloudCallback, this);
	    cout << "Subscribe to: " << LIDAR_TOPIC << endl;
	    break;
	case 1:
	    cout << "=====================MODE_IMU_LIDAR=====================" << endl;
	    mode = MODE_IMU_LIDAR;
	    subImu = pnh_.subscribe<sensor_msgs::Imu>(IMU_TOPIC, 100, &Liom::imuCallback, this); 
	    subLaserCloud = pnh_.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 2, &Liom::laserCloudCallback, this);
	    subLaserCloudInfo = pnh_.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 2, &Liom::laserCloudInfoCallback, this);
	    subOutlierCloud = pnh_.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 2, &Liom::outlierCloudCallback, this);
	    cout << "Subscribe to: " << IMU_TOPIC << endl;
	    cout << "Subscribe to: " << LIDAR_TOPIC << endl;
	    break;
	case 2:
	    cout << "=====================MODE_IMU_GPS=====================" << endl;
	    mode = MODE_IMU_GPS;
	    subImu = pnh_.subscribe<sensor_msgs::Imu>(IMU_TOPIC, 100, &Liom::imuCallback, this); 
	    subGPS_ = pnh_.subscribe<sensor_msgs::NavSatFix>(GPS_POS_TOPIC, 5, &Liom::gpsCallback, this);///fix/gstar /novatel718d/pos
	    cout << "Subscribe to: " << IMU_TOPIC << endl;
	    cout << "Subscribe to: " << GPS_POS_TOPIC << endl;
	    break;
	case 3:
	    cout << "=====================MODE_IMU_LIDAR_GPS=====================" << endl;
	    mode = MODE_IMU_LIDAR_GPS;
	    subImu = pnh_.subscribe<sensor_msgs::Imu>(IMU_TOPIC, 100, &Liom::imuCallback, this); 
	    subLaserCloud = pnh_.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 2, &Liom::laserCloudCallback, this);
	    subLaserCloudInfo = pnh_.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 2, &Liom::laserCloudInfoCallback, this);
	    subOutlierCloud = pnh_.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 2, &Liom::outlierCloudCallback, this);
	    subGPS_ = pnh_.subscribe<sensor_msgs::NavSatFix>(GPS_POS_TOPIC, 5, &Liom::gpsCallback, this);
	    cout << "Subscribe to: " << IMU_TOPIC << endl;
	    cout << "Subscribe to: " << LIDAR_TOPIC << endl;
	    cout << "Subscribe to: " << GPS_POS_TOPIC << endl;
	    break;
	default:
	    break;
		
    };
    
    if (SHOW_CONFIGURATION) {
	cout << "IMU_LIDAR_EXTRINSIC_ANGLE: " << IMU_LIDAR_EXTRINSIC_ANGLE << endl;
	cout << "IMU_MISALIGN_ANGLE: " << IMU_MISALIGN_ANGLE << endl;
	cout << "INIT_HEADING: " << INIT_HEADING << endl;
	cout.precision(20);
	cout << "REF_LAT: " << REF_LAT << endl;
	cout.precision(20);
	cout << "REF_LON: " << REF_LON << endl;
	cout << "REF_ALT: " << REF_ALT << endl;

	cout << "INIT_POS_STD: " << INIT_POS_STD.transpose() << endl;
	cout << "INIT_VEL_STD: " << INIT_VEL_STD.transpose() << endl;
	cout << "INIT_ATT_STD: " << INIT_ATT_STD.transpose() << endl;
	cout << "INIT_ACC_STD: " << INIT_ACC_STD.transpose() << endl;
	cout << "INIT_GYR_STD: " << INIT_GYR_STD.transpose() << endl;
	cout << "INIT_BA: " << INIT_BA.transpose() << endl;
	cout << "INIT_BW: " << INIT_BW.transpose() << endl;
	cout << "INIT_TBL: " << INIT_TBL.transpose() << endl;
	cout << "INIT_RBL: " << INIT_RBL.toRotationMatrix() << endl;
	
	cout << "ACC_N: " << ACC_N << endl;
	cout << "GYR_N: " << GYR_N << endl;
	cout << "ACC_W: " << ACC_W << endl;
	cout << "GYR_W: " << GYR_W << endl;
	
	cout << "VERBOSE: " << VERBOSE << endl;
	cout << "ICP_FREQ: " << ICP_FREQ << endl;

	cout << "LINE_NUM: " << LINE_NUM << endl;
	cout << "SCAN_NUM: " << SCAN_NUM << endl;
	cout << "SCAN_PERIOD: " << SCAN_PERIOD << endl;
	cout << "EDGE_THRESHOLD: " << EDGE_THRESHOLD << endl;
	cout << "SURF_THRESHOLD: " << SURF_THRESHOLD << endl;
	cout << "NEAREST_FEATURE_SEARCH_SQ_DIST: " << NEAREST_FEATURE_SEARCH_SQ_DIST << endl;
	
	cout << "CALIBARTE_IMU: " << CALIBARTE_IMU << endl;
	cout << "USE_REF: " << USE_REF << endl;
	cout << "HIGH_OUTPUT_RATE_MODE: " << HIGH_OUTPUT_RATE_MODE << endl;
    }

}
void Liom::initialization()
{
    estimator = new StateEstimator();
    
    pubUndistortedPointCloud = pnh_.advertise<sensor_msgs::PointCloud2>("/undistorted_point_cloud", 1);
    
    pubCornerPointsSharp = pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
    pubCornerPointsLessSharp = pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
    pubSurfPointsFlat = pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
    pubSurfPointsLessFlat = pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

    pubLaserCloudCornerLast = pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
    pubLaserCloudSurfLast = pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
    pubOutlierCloudLast = pnh_.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
    pubLaserOdometry = pnh_.advertise<nav_msgs::Odometry> (LIDAR_ODOMETRY_TOPIC, 5);
    pubIMUOdometry = pnh_.advertise<nav_msgs::Odometry> (IMU_ODOMETRY_TOPIC, 5);
    pubGpsOdometry = pnh_.advertise<nav_msgs::Odometry> (GPS_ODOMETRY_TOPIC, 5);

    pubLaserOdom = pnh_.advertise<nav_msgs::Odometry> (LASER_ODOM_TOPIC, 5);
    
    distortedPointCloud.reset(new pcl::PointCloud<PointType>());
    outlierPointCloud.reset(new pcl::PointCloud<PointType>());
    
    imuBuf_.allocate(500);
    gpsBuf_.allocate(2);
    pclBuf_.allocate(3);
    outlierBuf_.allocate(3);
    cloudInfoBuf_.allocate(3);

    ba_tmp_.setZero();
    bw_tmp_.setZero();
    ba_init_ = INIT_BA;
    bw_init_ = INIT_BW;
    
    isInititialized = false;
    isImuCalibrated = false;
    isRefSet = false;
    
    duration_ = 0.0;

    if (USE_REF) {
	estimator->gpsRef_ = Gps(0.0, deg2rad(REF_LAT), deg2rad(REF_LON), deg2rad(REF_ALT));
	isRefSet = true;
    }
	
    

}

void Liom::mapOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometryMsg)
{
	cout << "correction by mapping." << endl;
	geometry_msgs::Quaternion geoQuat = odometryMsg->pose.pose.orientation;
	V3D t_yzx(odometryMsg->pose.pose.position.x, odometryMsg->pose.pose.position.y, odometryMsg->pose.pose.position.z);
	Q4D q_yzx(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
	V3D t_xyz = estimator->Q_yzx_to_xyz * t_yzx;
	Q4D q_xyz = estimator->Q_yzx_to_xyz*q_yzx*estimator->Q_yzx_to_xyz.inverse();
	
	V3D rpy = math_utils::Q2rpy(q_xyz);
// 	estimator->correctOrientation(q_xyz);
// 	estimator->correctRollPitch(rpy[0], rpy[1]);
// 	
// 	V3D gravity_w;
// 	gravity_w << 0, 0, -G0;
// 	
// 	Q4D quad = math_utils::rpy2Quat(V3D(rpy[0], rpy[1], 0.0f));
// 	estimator->gravity_feedback = quad.inverse() * gravity_w;
}

void Liom::caliIMU()
{
    Imu imu;
    while(imuBuf_.getSize() != 0) {
	imuBuf_.getFirstMeas(imu);
	ba_tmp_ += imu.acc - V3D(0, 0, G0);
	bw_tmp_ += imu.gyr;
	calibrationSampleCount_++;
	if(calibrationSampleCount_ == AVERAGE_NUMS){
	    ba_init_ = ba_tmp_ * (1./calibrationSampleCount_);
	    bw_init_ = bw_tmp_ * (1./calibrationSampleCount_);
	    isImuCalibrated = true;
	    ROS_INFO_STREAM("INIT : IMU calibrate succesfully!\n ba0: " << ba_init_.transpose() << ",\n bg0: " << bw_init_.transpose());
	    break;
	}
	imuBuf_.clean(imu.time);
	pclBuf_.clean(imu.time);
	outlierBuf_.clean(imu.time);
	cloudInfoBuf_.clean(imu.time);
    }
}

void Liom::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
{
    cout << "gpsCallback : " << gpsMsg->latitude << ", " << gpsMsg->longitude  << ", " <<  gpsMsg->altitude << endl;
    Gps gps(gpsMsg->header.stamp.toSec(), deg2rad(gpsMsg->latitude), deg2rad(gpsMsg->longitude), gpsMsg->altitude);
    gpsBuf_.addMeas(gps, gpsMsg->header.stamp.toSec());  
 
    if (!USE_REF && isRefSet == false) {
	estimator->gpsRef_ = Gps(gps.time, gps.lat, gps.lon, gps.alt);
	isRefSet = true;
    }
    
    publishGpsOdometry();
    
}


void Liom::imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
//     cout << "imuCallback" << endl;  
    V3D rpy;
    V3D acc_raw, gyr_raw;
    V3D acc_aligned, gyr_aligned;
    acc_raw << imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z;
    gyr_raw << imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z;
    rpy << deg2rad(0.0), deg2rad(0.0), deg2rad(IMU_MISALIGN_ANGLE);
    alignIMU(rpy, acc_raw, gyr_raw, acc_aligned, gyr_aligned);
    Imu imu(imuMsg->header.stamp.toSec(), acc_aligned, gyr_aligned);
    imuBuf_.addMeas(imu, imuMsg->header.stamp.toSec());
    
    if (HIGH_OUTPUT_RATE_MODE && estimator->isInitialized()) {
	predict();
    }
    
    process();
    
}


void Liom::predict()
{

    while((imuBuf_.itMeas_ = imuBuf_.measMap_.upper_bound(predictedTime_)) != imuBuf_.measMap_.end()){
	double dt = imuBuf_.itMeas_->first - predictedTime_;
	Imu imu = imuBuf_.itMeas_->second;
	
	if (isFirstImu == true) {
	    acc_last = imu.acc;
	    gyr_last = imu.gyr;  
	    isFirstImu = false;
	}
	Vector3d un_acc_0 = predictedState_.qbn_ * (acc_last - predictedState_.ba_) + predictedState_.gn_;
	Vector3d un_gyr = 0.5 * (gyr_last + imu.gyr) - predictedState_.bw_;
	Q4D dq = axis2Quat(un_gyr * dt);
	predictedState_.qbn_ =  (predictedState_.qbn_ * dq).normalized();
        Vector3d un_acc_1 = predictedState_.qbn_ * (imu.acc - predictedState_.ba_) + predictedState_.gn_;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        predictedState_.rn_ += dt * predictedState_.vn_ + 0.5 * dt * dt * un_acc;
        predictedState_.vn_ += dt * un_acc;
	
	predictedTime_ += dt;
	
	acc_last = imu.acc;
	gyr_last = imu.gyr;
    } 
    
    publishOdometry();
    
}

void Liom::laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    pclBuf_.addMeas(laserCloudMsg, laserCloudMsg->header.stamp.toSec());
//     pclBuf_.addMeas(*laserCloudMsg, laserCloudMsg->header.stamp.toSec());
}
void Liom::laserCloudInfoCallback(const cloud_msgs::cloud_infoConstPtr& cloudInfoMsg)
{
    cloudInfoBuf_.addMeas(*cloudInfoMsg, cloudInfoMsg->header.stamp.toSec());

}
void Liom::outlierCloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    outlierBuf_.addMeas(laserCloudMsg, laserCloudMsg->header.stamp.toSec());
//     outlierBuf_.addMeas(*laserCloudMsg, laserCloudMsg->header.stamp.toSec());

}

void Liom::process()
{
    
    
    if (CALIBARTE_IMU == true && isImuCalibrated == false) {
	cout << "Calibrating IMU...Please stop moving..." << endl;
	caliIMU();
	INIT_BA = ba_init_;
	INIT_BW = bw_init_;
	return;
    }
    

    switch (mode) {
	case MODE_LIDAR:
	    processLidarOdom();
	    break;
	case MODE_IMU_LIDAR:
	    processFusion();
	    break;
	case MODE_IMU_GPS:
	    processImuGpsFusion();
	    break;
	case MODE_IMU_LIDAR_GPS:
	    processFusion();
	    break;
	default:
	    break;
    };


    

}

void Liom::processImuLidarFusion()
{

    if (imuBuf_.empty() || pclBuf_.empty() || cloudInfoBuf_.empty() || outlierBuf_.empty())
	return; 
    
    if (!estimator->isInitialized()) {
	double scanTime;
	pclBuf_.getLastTime(scanTime);
	
	sensor_msgs::PointCloud2::ConstPtr pclMsg;
	pclBuf_.getLastMeas(pclMsg);
	distortedPointCloud->clear();
	pcl::fromROSMsg(*pclMsg, *distortedPointCloud);
	
	sensor_msgs::PointCloud2::ConstPtr outlierMsg;
	outlierBuf_.getLastMeas(outlierMsg);
	outlierPointCloud->clear();
	pcl::fromROSMsg(*outlierMsg, *outlierPointCloud);
	
	cloud_msgs::cloud_info cloudInfoMsg;
	cloudInfoBuf_.getLastMeas(cloudInfoMsg);	

	Imu imu;
	imuBuf_.getLastMeas(imu);
	estimator->initialization(scanTime, imu, distortedPointCloud, cloudInfoMsg, outlierPointCloud, ba_init_, bw_init_);
	
	predictedState_ = estimator->globalState_;
	predictedTime_ = estimator->getTime();
	
	pclBuf_.clean(estimator->getTime());
	cloudInfoBuf_.clean(estimator->getTime());
	outlierBuf_.clean(estimator->getTime());
	
// 	isInititialized = true;
	cout.precision(20);
	cout << "Initialization success. Time: " << scanTime << endl;
    }
    
    double lastPclTime;
    pclBuf_.getLastTime(lastPclTime);
    
    while (!pclBuf_.empty() && estimator->getTime() < lastPclTime) {
	//cout << "--------------------------------" << endl;
TicToc ts_total;
TicToc ts_prepare;
	pclBuf_.itMeas_ = pclBuf_.measMap_.upper_bound(estimator->getTime());
	sensor_msgs::PointCloud2::ConstPtr pclMsg = pclBuf_.itMeas_->second;
	double scanTime = pclBuf_.itMeas_->first; 
	distortedPointCloud->clear();
	pcl::fromROSMsg(*pclMsg, *distortedPointCloud);
	
	outlierBuf_.itMeas_ = outlierBuf_.measMap_.upper_bound(estimator->getTime());
	sensor_msgs::PointCloud2::ConstPtr outlierMsg = outlierBuf_.itMeas_->second;
	outlierPointCloud->clear();
	pcl::fromROSMsg(*outlierMsg, *outlierPointCloud);
	
	cloudInfoBuf_.itMeas_ = cloudInfoBuf_.measMap_.upper_bound(estimator->getTime());
	cloud_msgs::cloud_info cloudInfoMsg = cloudInfoBuf_.itMeas_->second;
	
	
	double lastImuTime;
	imuBuf_.getLastTime(lastImuTime);
	if (lastImuTime < scanTime) {
	    ROS_WARN("Wait for more IMU measurement!");
	    break;
	}
	int counter = 0;
double time_prepare = ts_prepare.toc();
	
TicToc ts_imu;
	while(estimator->getTime() < scanTime && (imuBuf_.itMeas_ = imuBuf_.measMap_.upper_bound(estimator->getTime())) != imuBuf_.measMap_.end()){
	    double dt = std::min(imuBuf_.itMeas_->first, scanTime) - estimator->getTime();
	    Imu imu = imuBuf_.itMeas_->second;
	    estimator->processImu(dt, imu.acc, imu.gyr);
	    counter++;
	}
	//cout << "imu size: " << counter << endl;
double time_imu = ts_imu.toc();


TicToc ts_pcl;
	estimator->processPcl(scanTime, distortedPointCloud, cloudInfoMsg, outlierPointCloud);
	if (HIGH_OUTPUT_RATE_MODE) {
	    predictedState_ = estimator->globalState_;
	    predictedTime_ = estimator->getTime();    
	}
	//cout << "ba: " << estimator->globalState_.ba_.transpose() << endl;
	//cout << "bw: " << estimator->globalState_.bw_.transpose() << endl;
double time_pcl = ts_pcl.toc();

TicToc ts_pub;

	if (pubLaserCloudCornerLast.getNumSubscribers() != 0){
	    publishCloudMsg(pubLaserCloudCornerLast, estimator->scan_last_->cornerPointsLessSharpYZX_, ros::Time().fromSec(scanTime), "/camera"); 
	}
	if (pubLaserCloudSurfLast.getNumSubscribers() != 0){
	    publishCloudMsg(pubLaserCloudSurfLast, estimator->scan_last_->surfPointsLessFlatYZX_, ros::Time().fromSec(scanTime), "/camera"); 
	} 
	if (pubOutlierCloudLast.getNumSubscribers() != 0){
	    publishCloudMsg(pubOutlierCloudLast, estimator->scan_last_->outlierPointCloudYZX_, ros::Time().fromSec(scanTime), "/camera"); 
	}

	publishOdometryYZX(scanTime);
/*
	if (pubLaserCloudCornerLast.getNumSubscribers() != 0){
	    publishCloudMsg(pubLaserCloudCornerLast, estimator->scan_last_->cornerPointsLessSharp_, ros::Time().fromSec(scanTime), "/lidar"); 
	}
	if (pubLaserCloudSurfLast.getNumSubscribers() != 0){
	    publishCloudMsg(pubLaserCloudSurfLast, estimator->scan_last_->surfPointsLessFlat_, ros::Time().fromSec(scanTime), "/lidar"); 
	} 
	if (pubOutlierCloudLast.getNumSubscribers() != 0){
	    publishCloudMsg(pubOutlierCloudLast, estimator->scan_last_->outlierPointCloud_, ros::Time().fromSec(scanTime), "/lidar"); 
	}*/
	
	//publishOdometry(scanTime);

	//publishLaserOdom(scanTime);
double time_pub = ts_pub.toc();
	imuBuf_.clean(estimator->getTime());
	pclBuf_.clean(estimator->getTime());
	cloudInfoBuf_.clean(estimator->getTime());
	outlierBuf_.clean(estimator->getTime());
	
double time_total = ts_total.toc();
// 	cout.precision(20);
// 	cout << "time_prepare: " << time_prepare << endl;
// 	cout << "time_imu: " << time_imu << endl;
// 	cout << "time_pcl: " << time_pcl << endl;
// 	cout << "time_pub: " << time_pub << endl;
// 	cout << "ratio: " << 100.0 * time_pcl / time_total << " %, time_total: " << time_total << endl;
	
	
    }
}


void Liom::processFusion()
{

    if (imuBuf_.empty() || pclBuf_.empty() || cloudInfoBuf_.empty() || outlierBuf_.empty())
	return; 
    
    if (!estimator->isInitialized()) {
	double scanTime;
	pclBuf_.getLastTime(scanTime);
	
	sensor_msgs::PointCloud2::ConstPtr pclMsg;
	pclBuf_.getLastMeas(pclMsg);
	distortedPointCloud->clear();
	pcl::fromROSMsg(*pclMsg, *distortedPointCloud);
	
	sensor_msgs::PointCloud2::ConstPtr outlierMsg;
	outlierBuf_.getLastMeas(outlierMsg);
	outlierPointCloud->clear();
	pcl::fromROSMsg(*outlierMsg, *outlierPointCloud);
	
	cloud_msgs::cloud_info cloudInfoMsg;
	cloudInfoBuf_.getLastMeas(cloudInfoMsg);	

	Imu imu;
	imuBuf_.getLastMeas(imu);
	
	estimator->processPcl(scanTime, imu, distortedPointCloud, cloudInfoMsg, outlierPointCloud);
	
	predictedState_ = estimator->globalState_;
	predictedTime_ = estimator->getTime();
	
	pclBuf_.clean(estimator->getTime());
	cloudInfoBuf_.clean(estimator->getTime());
	outlierBuf_.clean(estimator->getTime());

    }
    
    double lastPclTime;
    pclBuf_.getLastTime(lastPclTime);
    
    while (!pclBuf_.empty() && estimator->getTime() < lastPclTime) {
	TicToc ts_total;
	pclBuf_.itMeas_ = pclBuf_.measMap_.upper_bound(estimator->getTime());
	sensor_msgs::PointCloud2::ConstPtr pclMsg = pclBuf_.itMeas_->second;
	double scanTime = pclBuf_.itMeas_->first; 
	distortedPointCloud->clear();
	pcl::fromROSMsg(*pclMsg, *distortedPointCloud);
	
	outlierBuf_.itMeas_ = outlierBuf_.measMap_.upper_bound(estimator->getTime());
	sensor_msgs::PointCloud2::ConstPtr outlierMsg = outlierBuf_.itMeas_->second;
	outlierPointCloud->clear();
	pcl::fromROSMsg(*outlierMsg, *outlierPointCloud);
	
	cloudInfoBuf_.itMeas_ = cloudInfoBuf_.measMap_.upper_bound(estimator->getTime());
	cloud_msgs::cloud_info cloudInfoMsg = cloudInfoBuf_.itMeas_->second;
	
	
	double lastImuTime;
	imuBuf_.getLastTime(lastImuTime);
	if (lastImuTime < scanTime) {
	    //ROS_WARN("Wait for more IMU measurement!");
	    break;
	}

	int imu_couter = 0;
	while(estimator->getTime() < scanTime && (imuBuf_.itMeas_ = imuBuf_.measMap_.upper_bound(estimator->getTime())) != imuBuf_.measMap_.end()){
	    double dt = std::min(imuBuf_.itMeas_->first, scanTime) - estimator->getTime();
	    Imu imu = imuBuf_.itMeas_->second;
	    estimator->processImu(dt, imu.acc, imu.gyr);
// 	    if (imu_couter == 5) {
// 		publishImuOdometryYZX(imu.time);
// 		imu_couter = 0;
// 	    } else {
// 		imu_couter++;
// 	    }
		
	}

	Imu imu;
	imuBuf_.getLastMeas(imu);
	
	estimator->processPcl(scanTime, imu, distortedPointCloud, cloudInfoMsg, outlierPointCloud);
	
	
	if (HIGH_OUTPUT_RATE_MODE) {
	    predictedState_ = estimator->globalState_;
	    predictedTime_ = estimator->getTime();    
	}
	

	if (pubLaserCloudCornerLast.getNumSubscribers() != 0){
	    publishCloudMsg(pubLaserCloudCornerLast, estimator->scan_last_->cornerPointsLessSharpYZX_, ros::Time().fromSec(scanTime), "/camera"); 
	}
	if (pubLaserCloudSurfLast.getNumSubscribers() != 0){
	    publishCloudMsg(pubLaserCloudSurfLast, estimator->scan_last_->surfPointsLessFlatYZX_, ros::Time().fromSec(scanTime), "/camera"); 
	} 
	if (pubOutlierCloudLast.getNumSubscribers() != 0){
	    publishCloudMsg(pubOutlierCloudLast, estimator->scan_last_->outlierPointCloudYZX_, ros::Time().fromSec(scanTime), "/camera"); 
	}
	publishOdometryYZX(scanTime);

	imuBuf_.clean(estimator->getTime());
	pclBuf_.clean(estimator->getTime());
	cloudInfoBuf_.clean(estimator->getTime());
	outlierBuf_.clean(estimator->getTime());
	
	double time_total = ts_total.toc();

	if (VERBOSE) {
	    cout << "ba: " << estimator->globalState_.ba_.transpose() << endl;
	    cout << "bw: " << estimator->globalState_.bw_.transpose() << endl;
			cout << "gw: " << estimator->globalState_.gn_.transpose() << endl;
	    duration_ = (duration_*lidarCounter + time_total)/(lidarCounter + 1);
	    lidarCounter++;
	    cout << "Odometry: time: " << duration_ << endl;
	}
    }
}

void Liom::processImuGpsFusion()
{

}

void Liom::processImuLidarGpsFusion()
{


}



void Liom::alignIMU(const V3D& rpy, const V3D& acc_in, const V3D& gyr_in, V3D& acc_out, V3D& gyr_out)
{
    M3D R = rpy2R(rpy);
    acc_out = R.transpose()*acc_in;
    gyr_out = R.transpose()*gyr_in;
    
//     R << -0.1762,-0.9816,0.0738,
//         0.9831,-0.1793,-0.0366,
//         0.0492,0.0661,0.9966;
    
//     M3D R = INIT_RBL.toRotationMatrix();
//     acc_out = R*acc_in;
//     gyr_out = R*gyr_in;
}


void Liom::processLidarOdom()
{

}
 
void Liom::publishOdometry()
{
    laserOdometry.header.frame_id = "/lidar_init";
    laserOdometry.child_frame_id = "/laser_odom";
    laserOdometry.header.stamp = ros::Time().fromSec(predictedTime_);
    laserOdometry.pose.pose.orientation.x = predictedState_.qbn_.x();
    laserOdometry.pose.pose.orientation.y = predictedState_.qbn_.y();
    laserOdometry.pose.pose.orientation.z = predictedState_.qbn_.z();
    laserOdometry.pose.pose.orientation.w = predictedState_.qbn_.w();
    laserOdometry.pose.pose.position.x = predictedState_.rn_[0];
    laserOdometry.pose.pose.position.y = predictedState_.rn_[1];
    laserOdometry.pose.pose.position.z = predictedState_.rn_[2];
    pubLaserOdometry.publish(laserOdometry);
    
    tf::TransformBroadcaster tfBroadcaster; 
    tf::StampedTransform laserOdometryTrans;
    laserOdometryTrans.frame_id_ = "/lidar_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom"; 
    laserOdometryTrans.stamp_ = ros::Time().fromSec(predictedTime_);
    laserOdometryTrans.setRotation(tf::Quaternion(predictedState_.qbn_.x(), predictedState_.qbn_.y(), predictedState_.qbn_.z(), predictedState_.qbn_.w()));
    laserOdometryTrans.setOrigin(tf::Vector3(predictedState_.rn_[0], predictedState_.rn_[1], predictedState_.rn_[2]));
    tfBroadcaster.sendTransform(laserOdometryTrans);
}

 
void Liom::publishOdometry(double timeStamp){  
    laserOdometry.header.frame_id = "/lidar_init";
    laserOdometry.child_frame_id = "/laser_odom";
    laserOdometry.header.stamp = ros::Time().fromSec(timeStamp);
    laserOdometry.pose.pose.orientation.x = estimator->globalState_.qbn_.x();
    laserOdometry.pose.pose.orientation.y = estimator->globalState_.qbn_.y();
    laserOdometry.pose.pose.orientation.z = estimator->globalState_.qbn_.z();
    laserOdometry.pose.pose.orientation.w = estimator->globalState_.qbn_.w();
    laserOdometry.pose.pose.position.x = estimator->globalState_.rn_[0];
    laserOdometry.pose.pose.position.y = estimator->globalState_.rn_[1];
    laserOdometry.pose.pose.position.z = estimator->globalState_.rn_[2];
    V3D phi = Quat2axis(estimator->linState_.qbn_);
    laserOdometry.twist.twist.angular.x = phi[0];
    laserOdometry.twist.twist.angular.y = phi[1];
    laserOdometry.twist.twist.angular.z = phi[2];
    laserOdometry.twist.twist.linear.x = estimator->globalState_.vn_[0];
    laserOdometry.twist.twist.linear.y = estimator->globalState_.vn_[1];
    laserOdometry.twist.twist.linear.z = estimator->globalState_.vn_[2];
    pubLaserOdometry.publish(laserOdometry);
    
    tf::TransformBroadcaster tfBroadcaster; 
    tf::StampedTransform laserOdometryTrans;
    laserOdometryTrans.frame_id_ = "/lidar_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom"; 
    laserOdometryTrans.stamp_ = ros::Time().fromSec(timeStamp);
    laserOdometryTrans.setRotation(tf::Quaternion(estimator->globalState_.qbn_.x(), estimator->globalState_.qbn_.y(), estimator->globalState_.qbn_.z(), estimator->globalState_.qbn_.w()));
    laserOdometryTrans.setOrigin(tf::Vector3(estimator->globalState_.rn_[0], estimator->globalState_.rn_[1], estimator->globalState_.rn_[2]));
    tfBroadcaster.sendTransform(laserOdometryTrans);
}

void Liom::publishOdometryYZX(double timeStamp){  
    laserOdometry.header.frame_id = "/camera_init";
    laserOdometry.child_frame_id = "/laser_odom";
    laserOdometry.header.stamp = ros::Time().fromSec(timeStamp);
    laserOdometry.pose.pose.orientation.x = estimator->globalStateYZX_.qbn_.x();
    laserOdometry.pose.pose.orientation.y = estimator->globalStateYZX_.qbn_.y();
    laserOdometry.pose.pose.orientation.z = estimator->globalStateYZX_.qbn_.z();
    laserOdometry.pose.pose.orientation.w = estimator->globalStateYZX_.qbn_.w();
    laserOdometry.pose.pose.position.x = estimator->globalStateYZX_.rn_[0];
    laserOdometry.pose.pose.position.y = estimator->globalStateYZX_.rn_[1];
    laserOdometry.pose.pose.position.z = estimator->globalStateYZX_.rn_[2];
    pubLaserOdometry.publish(laserOdometry);
    
    tf::TransformBroadcaster tfBroadcaster; 
    tf::StampedTransform laserOdometryTrans;
    laserOdometryTrans.frame_id_ = "/camera_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom"; 
    laserOdometryTrans.stamp_ = ros::Time().fromSec(timeStamp);
    laserOdometryTrans.setRotation(tf::Quaternion(estimator->globalStateYZX_.qbn_.x(), estimator->globalStateYZX_.qbn_.y(), estimator->globalStateYZX_.qbn_.z(), estimator->globalStateYZX_.qbn_.w()));
    laserOdometryTrans.setOrigin(tf::Vector3(estimator->globalStateYZX_.rn_[0], estimator->globalStateYZX_.rn_[1], estimator->globalStateYZX_.rn_[2]));
    tfBroadcaster.sendTransform(laserOdometryTrans);
}


void Liom::publishImuOdometryYZX(double timeStamp)
{
    laserOdometry.header.frame_id = "/camera_init";
    laserOdometry.child_frame_id = "/imu_odom";
    laserOdometry.header.stamp = ros::Time().fromSec(timeStamp);
    
    
    V3D pos_xyz = estimator->globalState_.qbn_ * estimator->filter_->state_.rn_ + estimator->globalState_.rn_;
    Q4D rot_xyz = estimator->globalState_.qbn_ * estimator->filter_->state_.qbn_;
    Q4D rot_yzx;
    V3D pos_yzx;
    pos_yzx = estimator->Q_xyz_to_yzx * pos_xyz;
    rot_yzx = estimator->Q_xyz_to_yzx * rot_xyz * estimator->Q_xyz_to_yzx.inverse();

    
    
/*    
    Q4D rot_yzx(estimator->quad_);
    V3D pos_yzx = estimator->pos_;
    pos_yzx = estimator->Q_xyz_to_yzx * pos_yzx;
    rot_yzx = estimator->Q_xyz_to_yzx * rot_yzx * estimator->Q_xyz_to_yzx.inverse();*/
    
    laserOdometry.pose.pose.orientation.x = rot_yzx.x();
    laserOdometry.pose.pose.orientation.y = rot_yzx.y();
    laserOdometry.pose.pose.orientation.z = rot_yzx.z();
    laserOdometry.pose.pose.orientation.w = rot_yzx.w();
    laserOdometry.pose.pose.position.x = pos_yzx[0];
    laserOdometry.pose.pose.position.y = pos_yzx[1];
    laserOdometry.pose.pose.position.z = pos_yzx[2];
    pubIMUOdometry.publish(laserOdometry);
    
    
    tf::TransformBroadcaster tfBroadcaster; 
    tf::StampedTransform laserOdometryTrans;
    laserOdometryTrans.frame_id_ = "/camera_init";
    laserOdometryTrans.child_frame_id_ = "/imu_odom"; 
    laserOdometryTrans.stamp_ = ros::Time().fromSec(timeStamp);
    laserOdometryTrans.setRotation(tf::Quaternion(rot_yzx.x(), rot_yzx.y(), rot_yzx.z(), rot_yzx.w()));
    laserOdometryTrans.setOrigin(tf::Vector3(pos_yzx[0], pos_yzx[1], pos_yzx[2]));
    tfBroadcaster.sendTransform(laserOdometryTrans);
}



void Liom::publishGpsOdometry(){
    Gps gps;
    if (!gpsBuf_.getLastMeas(gps)) 
	return;
    Q4D Cbn = Q4D::Identity();
    V3D rn = EarthParams::getDpr(estimator->gpsRef_.pn()) * (gps.pn() - estimator->gpsRef_.pn());

    
    
    
    double dt = gps.time - gps_last_.time;
    
    if (dt < 0.1f && dt > 3.0f) {
	gps_last_ = gps;
	return;
    }	


    V3D vn = EarthParams::getDpr(estimator->gpsRef_.pn())*(gps.pn() - gps_last_.pn()) / dt;
    // cout << "GPS dt " << dt << ": " << rad2deg(gps.lat) << ", " << rad2deg(gps.lon) << ", " << gps.lat << endl;
    if (vn.norm() < 0.5) {
	gps_last_ = gps;
	return;
    }
    
    double heading = atan2(vn(1), vn(0));
    cout << "heading: " << rad2deg(heading) << endl;
    M3D gpsCbn = ypr2R(V3D(heading, 0.0, 0.0));
    Cbn = gpsCbn;
    
    double roll_init=0.0, pitch_init=0.0, yaw_init=deg2rad(INIT_HEADING);
    Q4D qbn = rpy2Quat(V3D(roll_init, pitch_init, yaw_init));
//     Cbn = qbn.inverse().toRotationMatrix() * Cbn * qbn.toRotationMatrix();
    rn = qbn.inverse() * rn;
    
    Cbn = estimator->Q_xyz_to_yzx * Cbn * estimator->Q_xyz_to_yzx.inverse();
    rn = estimator->Q_xyz_to_yzx * rn;
    
    
    gpsOdometry.header.frame_id = "/camera_init";
    gpsOdometry.child_frame_id = "/gps_odom";
    gpsOdometry.header.stamp = ros::Time().fromSec(gps.time);
    gpsOdometry.pose.pose.orientation.x = Cbn.x();
    gpsOdometry.pose.pose.orientation.y = Cbn.y();
    gpsOdometry.pose.pose.orientation.z = Cbn.z();
    gpsOdometry.pose.pose.orientation.w = Cbn.w();
    gpsOdometry.pose.pose.position.x = rn[0];
    gpsOdometry.pose.pose.position.y = rn[1];
    gpsOdometry.pose.pose.position.z = rn[2];
    pubGpsOdometry.publish(gpsOdometry);
    
    tf::TransformBroadcaster tfBroadcaster; 
    tf::StampedTransform gpsOdometryTrans;
    gpsOdometryTrans.frame_id_ = "/camera_init";
    gpsOdometryTrans.child_frame_id_ = "/gps_odom"; 
    gpsOdometryTrans.stamp_ = ros::Time().fromSec(gps.time);
    gpsOdometryTrans.setRotation(tf::Quaternion(Cbn.x(), Cbn.y(), Cbn.z(), Cbn.w()));
    gpsOdometryTrans.setOrigin(tf::Vector3(rn[0], rn[1], rn[2]));
    tfBroadcaster.sendTransform(gpsOdometryTrans);
    
    gps_last_ = gps;
    gpsBuf_.clean(gps.time);
}

  
void Liom::publishLaserOdom(double timeStamp)
{
    //cout << "publishLaserOdom" << endl;
    laserOdom.header.frame_id = "/lidar_init";
    laserOdom.child_frame_id = "/lio_odom";
    laserOdom.header.stamp = ros::Time().fromSec(timeStamp);
    laserOdom.pose.pose.orientation.x = estimator->linState_.qbn_.x();
    laserOdom.pose.pose.orientation.y = estimator->linState_.qbn_.y();
    laserOdom.pose.pose.orientation.z = estimator->linState_.qbn_.z();
    laserOdom.pose.pose.orientation.w = estimator->linState_.qbn_.w();
    laserOdom.pose.pose.position.x = estimator->linState_.rn_[0];
    laserOdom.pose.pose.position.y = estimator->linState_.rn_[1];
    laserOdom.pose.pose.position.z = estimator->linState_.rn_[2];
    pubLaserOdom.publish(laserOdom);

    tf::TransformBroadcaster tfBroadcaster; 
    tf::StampedTransform laserOdomTrans;
    laserOdomTrans.frame_id_ = "/lidar_init";
    laserOdomTrans.child_frame_id_ = "/lio_odom"; 
    laserOdomTrans.stamp_ = ros::Time().fromSec(timeStamp);
    laserOdomTrans.setRotation(tf::Quaternion(estimator->linState_.qbn_.x(), estimator->linState_.qbn_.y(), estimator->linState_.qbn_.z(), estimator->linState_.qbn_.w()));
    laserOdomTrans.setOrigin(tf::Vector3(estimator->linState_.rn_[0], estimator->linState_.rn_[1], estimator->linState_.rn_[2]));
    tfBroadcaster.sendTransform(laserOdomTrans);
}

}