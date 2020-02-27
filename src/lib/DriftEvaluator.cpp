#include <DriftEvaluator.h>

// #define USE_LIOM

namespace evaluation {

Evaluator::Evaluator(ros::NodeHandle& nh, ros::NodeHandle& pnh):
    nh_(nh),
    pnh_(pnh)
{
    GT_GPS = true;
    GT_LOAM = false;
}


Evaluator::~Evaluator()
{

}


void Evaluator::run()
{
    pubGpsOdometry_ = pnh_.advertise<nav_msgs::Odometry> (GPS_ODOMETRY_TOPIC, 5);    
    subOdometry_ = pnh_.subscribe<nav_msgs::Odometry>(LIDAR_ODOMETRY_TOPIC, 5, &Evaluator::laserOdometryCallback, this);
    subMapping_ = pnh_.subscribe<nav_msgs::Odometry>(LIDAR_MAPPING_TOPIC, 5, &Evaluator::laserMappingCallback, this);
    if (GT_GPS) {
	subGroundTrueGPS_ = pnh_.subscribe<sensor_msgs::NavSatFix>(GPS_POS_TOPIC, 5, &Evaluator::gpsCallback, this);
    } else if (GT_LOAM) {
	subGroundTrueLOAM_ = pnh_.subscribe<nav_msgs::Odometry>(LIDAR_MAPPING_TOPIC, 5, &Evaluator::loamCallback, this);
    }
    
    gpsBuf_.allocate(10);
    odomBuf_.allocate(500);
    mappingBuf_.allocate(500);

    double heading = 0.0;
    std::string errorType = "";
    
    pnh_.getParam("heading", heading);
    heading_ = deg2rad(heading);
    
    pnh_.getParam("errorType", errorType);
    if (errorType == "RDM") {
	errorType_ = RDM;
    } else if (errorType == "ATE") {
	errorType_ = ATE;
    } else if (errorType == "KITTI") {
	errorType_ = KITTI;
    }
    
    ROS_INFO_STREAM("Initial heading: " << heading);
    ROS_INFO_STREAM("Error type: " << errorType);
    
    isRefSet = false;	
    sumDistance_ = 0.0;

    R_yzx_to_xyz << 0., 0., 1.,
		    1., 0., 0.,
		    0., 1., 0.;
    R_xyz_to_yzx = R_yzx_to_xyz.transpose();
    Q_yzx_to_xyz = 	R_yzx_to_xyz;
    Q_xyz_to_yzx = R_xyz_to_yzx;
}

void Evaluator::process()
{
    if (GT_GPS) {
	processGPS();
// 	processLiomapGPS();
    } else if (GT_LOAM) {
	processLOAM();
    }
}

void Evaluator::processGPS()
{

    if (odomBuf_.getSize()<1 || mappingBuf_.getSize()<1 || gpsBuf_.empty() || first_liom_flag)
	return; 
    
    Gps gps;
    gpsBuf_.getLastMeas(gps);
    
    if (lastGps_.time == 0.0) {
	lastGps_ = gps;
	gpsBuf_.clean(gps.time);
	
	kittiGps_ = gps;
	V3D rn = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - gpsRef_.pn());
	double roll_init=0.0, pitch_init=0.0, yaw_init=heading_;
	Q4D qbn = rpy2Quat(V3D(roll_init, pitch_init, yaw_init));
	rn = qbn.inverse() * rn;
	
	kittiRn_ = rn;
	mappingBuf_.getLastMeas(kittiMapping_);
	odomBuf_.getLastMeas(kittiOdom_);
	mappingBuf_.clean(kittiMapping_.time);
	odomBuf_.clean(kittiOdom_.time);
	
	segCounter = 0;
	return;
    }
    
/*    double lastMappingTime;
    mappingBuf_.getLastTime(lastMappingTime);
    if (lastMappingTime < gps.time) {
	//cout.precision(20);
	//cout << "mapping time: " << lastMappingTime << endl;
	//ROS_WARN("Wait for more Mapping measurement!");
	return;
    }    
    
    double lastOdomTime;
    odomBuf_.getLastTime(lastOdomTime);
    if (lastOdomTime < gps.time) {
	//cout.precision(20);
	//cout << "odometry time: " << lastOdomTime << endl;
	//ROS_WARN("Wait for more Odom measurement!");
	return;
    } */   
    
    // GPS measurement
    V3D rn = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - gpsRef_.pn());
    double roll_init=0.0, pitch_init=0.0, yaw_init=heading_;
    Q4D qbn = rpy2Quat(V3D(roll_init, pitch_init, yaw_init));
    rn = qbn.inverse() * rn  + init_rn;

    V3D translation_gps = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - lastGps_.pn());
    double dt = gps.time - lastGps_.time;
    V3D vn = translation_gps / dt; //ENU	 
    double gpsHeading = 0.0;
    if (vn.norm() > 0.5) {
	gpsHeading = atan2(vn(1), vn(0));
	cout << "gps heading: " << rad2deg(gpsHeading) << endl;
    }
    
    sumDistance_ += (translation_gps).norm();
    
    Odometry mappingOdom, pureOdom;
//     getClosestOdometry(mappingBuf_, gps.time, mappingOdom);
//     getClosestOdometry(odomBuf_, gps.time, pureOdom);
    mappingBuf_.getLastMeas(mappingOdom);
    odomBuf_.getLastMeas(pureOdom);
    
    
    double mappingDrift, odomDrift;
    mappingDrift =  (rn - mappingOdom.translation).norm();
    odomDrift = (rn - pureOdom.translation).norm();    
    cout << "init_rn   : " << init_rn.transpose() << endl;
    cout << "gps     rn: " << rn.transpose() << endl;
    cout << "mapping rn: " << mappingOdom.translation.transpose() << endl;

    //!@RDM
    if (sumDistance_ < 1.0) {
	mappingDriftRate_ = 0.0;
	odomDriftRate_ = 0.0;
    } else {
	mappingDriftRate_ = 100.0 * mappingDrift / sumDistance_;
	odomDriftRate_ = 100.0 * odomDrift / sumDistance_;
    }
    
    //!@ATE
    mappingDrift_ = mappingDrift;
    odomDrift_ = odomDrift;
    
    //!@KITTI
    int nSegmentation = int(sumDistance_) / PATH_LENGTH;
    V3D gpsTranslation;
    //cout << "nSegmentation: " << nSegmentation << ", PATH_LENGTH: " << PATH_LENGTH << ", " << int(sumDistance_) / PATH_LENGTH << endl;
    if (nSegmentation > segCounter) {
	segCounter++;
	mappingTransform_ = mappingOdom.boxMinus(kittiMapping_);
	odomTransform_ = pureOdom.boxMinus(kittiOdom_);
	
	mappingTransform_.translation = mappingOdom.translation - kittiMapping_.translation;
	odomTransform_.translation = pureOdom.translation - kittiOdom_.translation;
	gpsTranslation = rn - kittiRn_;
	/*
	cout << "rn: " << rn.transpose() << endl;
	cout << "mappingOdom: " << mappingOdom.translation.transpose() << endl;
	
	cout << "kittiRn_: " << kittiRn_.transpose() << endl;
	cout << "kittiMapping_.translation: " << kittiMapping_.translation.transpose() << endl;
	
	cout << "gpsTranslation: " << gpsTranslation.transpose() << endl;
	cout << "mappingTransform_.translation: " << mappingTransform_.translation.transpose() << endl;*/
	
	kittiGps_ = gps;
	kittiRn_ = rn;
	kittiMapping_ = mappingOdom;
	kittiOdom_ = pureOdom;
	updateKitti_ = true;
    }

    
    
    lastGps_ = gps;
    mappingBuf_.clean(mappingOdom.time);  
    odomBuf_.clean(pureOdom.time);  
//     gpsBuf_.clean(gps.time);  
    
    switch (errorType_) {
	case RDM:
	    cout << "sumDistance_: " << sumDistance_  << endl;
	    cout << "==== RDM ==== mapping: " << mappingDriftRate_ << "%, odometry: " <<  odomDriftRate_ << "%." << endl;
	    cout << "==== ATE ==== mapping: " << mappingDrift_ << " m, odometry: " <<  odomDrift_ << " m." << endl;
	    break;
	case ATE:
	    cout << "sumDistance_: " << sumDistance_  << endl;
	    cout << "==== RDM ==== mapping: " << mappingDriftRate_ << "%, odometry: " <<  odomDriftRate_ << "%." << endl;
	    cout << "==== ATE ==== mapping: " << mappingDrift_ << " m, odometry: " <<  odomDrift_ << " m." << endl;
	    break;
	default:
	    break;
    }

    
    publishGpsOdometry(gps, rn);
    
    if (transformLidarToGps_) {
	mappingOdom.translation;
	V3D rn = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - gpsRef_.pn());
	double roll_init=0.0, pitch_init=0.0, yaw_init=heading_;
	Q4D qbn = rpy2Quat(V3D(roll_init, pitch_init, yaw_init));
	rn = qbn.inverse() * rn;
	
	V3D pos = EarthParams::getDrp(gpsRef_.pn()) * (mappingOdom.translation);
	cout << "lat: " << rad2deg(pos[0]) << ", lon: " << rad2deg(pos[1]) << ", alt: " << pos[2] << endl;
    }
    

}

void Evaluator::processLiomapGPS()
{
    if (odomBuf_.getSize()<2 || mappingBuf_.empty() || gpsBuf_.getSize()<2 )
	return; 
    
    Gps gps;
    gpsBuf_.getLastMeas(gps);
    Gps lastGps;
    gpsBuf_.getLastLastMeas(lastGps);
    
    if (lastGps_.time == 0.0) {
// 	V3D translation_gps = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - lastGps.pn());
// 	double dt = gps.time - lastGps.time;
// 	V3D vn = translation_gps / dt; //ENU	 
// 	double gpsHeading = 0.0;
// 	if (vn.norm() > 0.2) {
// 	    gpsHeading = atan2(vn(1), vn(0));
// 	    cout << "gps heading: " << rad2deg(gpsHeading) << endl;
// 	}
// 	heading_ = gpsHeading+deg2rad(7.0f);
	lastGps_ = gps;
// 	cout << "========heading:====== " << rad2deg(heading_) << endl; 
	
	gpsBuf_.clean(gps.time);
	
// 	kittiGps_ = gps;
// 	V3D rn = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - gpsRef_.pn());
// 	double roll_init=0.0, pitch_init=0.0, yaw_init=heading_;
// 	Q4D qbn = rpy2Quat(V3D(roll_init, pitch_init, yaw_init));
// 	rn = qbn.inverse() * rn;
// 	
// 	kittiRn_ = rn;
// 	odomBuf_.getLastMeas(kittiOdom_);
// 	odomBuf_.clean(kittiOdom_.time);
	
// 	segCounter = 0;
	return;
    }
    

    cout.precision(20);
    cout << "gps time: " << gps.time << endl;

/*    double lastOdomTime;
    odomBuf_.getLastTime(lastOdomTime);
    if (lastOdomTime < gps.time) {
	cout.precision(20);
	cout << "odometry time: " << lastOdomTime << endl;
	ROS_WARN("Wait for more Odom measurement!");
	return;
    }  */  
    
    
    // GPS measurement
    V3D rn = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - gpsRef_.pn());
    double roll_init=0.0, pitch_init=0.0, yaw_init=heading_;
    Q4D qbn = rpy2Quat(V3D(roll_init, pitch_init, yaw_init));
    rn = qbn.inverse() * rn;

    V3D translation_gps = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - lastGps.pn());
    sumDistance_ += (translation_gps.head<2>()).norm();
    
    Odometry mappingOdom, pureOdom;
    mappingBuf_.getLastMeas(mappingOdom);
    odomBuf_.getLastMeas(pureOdom);
    //getClosestOdometry(odomBuf_, gps.time, pureOdom);
    
    double mappingDrift, odomDrift;
    mappingDrift =  (rn - mappingOdom.translation).head<2>().norm();
    odomDrift = (rn - pureOdom.translation).head<2>().norm();       

    //!@RDM
    if (sumDistance_ < 1.0) {
	mappingDriftRate_ = 0.0;
	odomDriftRate_ = 0.0;
    } else {
	mappingDriftRate_ = 100.0 * mappingDrift / sumDistance_;
	odomDriftRate_ = 100.0 * odomDrift / sumDistance_;
    }
    
    //!@ATE
    mappingDrift_ = mappingDrift;
    odomDrift_ = odomDrift;
    
//     //!@KITTI
//     int nSegmentation = int(sumDistance_) / PATH_LENGTH;
//     V3D gpsTranslation;
//     //cout << "nSegmentation: " << nSegmentation << ", PATH_LENGTH: " << PATH_LENGTH << ", " << int(sumDistance_) / PATH_LENGTH << endl;
//     if (nSegmentation > segCounter) {
// 	segCounter++;
// 	mappingTransform_ = mappingOdom.boxMinus(kittiMapping_);
// 	odomTransform_ = pureOdom.boxMinus(kittiOdom_);
// 	
// 	mappingTransform_.translation = mappingOdom.translation - kittiMapping_.translation;
// 	odomTransform_.translation = pureOdom.translation - kittiOdom_.translation;
// 	gpsTranslation = rn - kittiRn_;
// 	/*
// 	cout << "rn: " << rn.transpose() << endl;
// 	cout << "mappingOdom: " << mappingOdom.translation.transpose() << endl;
// 	
// 	cout << "kittiRn_: " << kittiRn_.transpose() << endl;
// 	cout << "kittiMapping_.translation: " << kittiMapping_.translation.transpose() << endl;
// 	
// 	cout << "gpsTranslation: " << gpsTranslation.transpose() << endl;
// 	cout << "mappingTransform_.translation: " << mappingTransform_.translation.transpose() << endl;*/
// 	
// 	kittiGps_ = gps;
// 	kittiRn_ = rn;
// 	kittiMapping_ = mappingOdom;
// 	kittiOdom_ = pureOdom;
// 	updateKitti_ = true;
//     }

    lastGps_ = gps;
    gpsBuf_.clean(lastGps.time);  
    
    switch (errorType_) {
	case RDM:
	    cout << "sumDistance_: " << sumDistance_  << endl;
	    cout << "==== RDM ==== mapping: " << mappingDriftRate_ << "%, odometry: " <<  odomDriftRate_ << "%." << endl;
	    cout << "==== ATE ==== mapping: " << mappingDrift_ << " m, odometry: " <<  odomDrift_ << " m." << endl;
	    break;
	case ATE:
	    cout << "sumDistance_: " << sumDistance_  << endl;
	    cout << "==== RDM ==== mapping: " << mappingDriftRate_ << "%, odometry: " <<  odomDriftRate_ << "%." << endl;
	    cout << "==== ATE ==== mapping: " << mappingDrift_ << " m, odometry: " <<  odomDrift_ << " m." << endl;
	    break;
	default:
	    break;
    }

    
    publishGpsOdometry(gps, rn);
}


void Evaluator::getClosestOdometry(MapRingBuffer< Odometry >& buf, const double& time, Odometry& odom)
{
    if (buf.empty()) {
	ROS_WARN("The Odometry Buffer is Empty.");
	return; 
    }
	
    
    if (buf.getSize() == 1) {
	buf.getLastMeas(odom);
	return;
    }
	
    
    double dt_front, dt_back;
    double odomTransDrift, mappingTransDrift, odomDriftRate, mappingDriftRate;    
    Odometry odom_front, odom_back;
    
    buf.itMeas_ = buf.measMap_.lower_bound(time);
    buf.itMeas_--;
    dt_front = time - buf.itMeas_->first;
    odom_front = buf.itMeas_->second;

    
    buf.itMeas_ = buf.measMap_.upper_bound(time);
    dt_back = buf.itMeas_->first - time;
    odom_back = buf.itMeas_->second;

    if (dt_front < dt_back) {
	odom = odom_front;
    } else {
	odom = odom_back;
    }
}



void Evaluator::processLOAM()
{

}


void Evaluator::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
{
//     cout.precision(20);
    cout << "gpsCallback : " << gpsMsg->latitude << ", " << gpsMsg->longitude  << ", " <<  gpsMsg->altitude << endl;

    if (odomFlag == false)
	return;
    
    Gps gps(gpsMsg->header.stamp.toSec(), deg2rad(gpsMsg->latitude), deg2rad(gpsMsg->longitude), gpsMsg->altitude);
    if (isRefSet == false) {
	gpsRef_ = Gps(gps.time, gps.lat, gps.lon, gps.alt);
	isRefSet = true;
    }
    
    gpsBuf_.addMeas(gps, gpsMsg->header.stamp.toSec());  

}

void Evaluator::laserOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometryMsg)
{
    //cout << "laserOdometryCallback" << endl;
    geometry_msgs::Quaternion geoQuat = odometryMsg->pose.pose.orientation;
    V3D t_yzx(odometryMsg->pose.pose.position.x, odometryMsg->pose.pose.position.y, odometryMsg->pose.pose.position.z);
    Q4D q_yzx(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
    V3D t_xyz = Q_yzx_to_xyz * t_yzx;
    Q4D q_xyz = Q_yzx_to_xyz*q_yzx*Q_yzx_to_xyz.inverse();
    //cout << "trans: " << t_xyz.transpose() << endl;;
    
#ifdef USE_LIOM
    Odometry odom(odometryMsg->header.stamp.toSec(), q_yzx, t_yzx);
    odomBuf_.addMeas(odom, odometryMsg->header.stamp.toSec());  

#else
    Odometry odom(odometryMsg->header.stamp.toSec(), q_xyz, t_xyz);
    odomBuf_.addMeas(odom, odometryMsg->header.stamp.toSec());  
#endif
    
    
    //TODO 1.将YZX转换成XYZ,装入Buffer中
    // 2. 将GPS转换成XYZ,装入Buffer中
    // 3. 找离GPS最近的测量值,计算最后一组的位移差.

}

void Evaluator::laserMappingCallback(const nav_msgs::Odometry::ConstPtr& mappingMsg)
{
    //cout << "laserMappingCallback" << endl;
    
    geometry_msgs::Quaternion geoQuat = mappingMsg->pose.pose.orientation;
    V3D t_yzx(mappingMsg->pose.pose.position.x, mappingMsg->pose.pose.position.y, mappingMsg->pose.pose.position.z);
    Q4D q_yzx(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
    V3D t_xyz = Q_yzx_to_xyz * t_yzx;
    Q4D q_xyz = Q_yzx_to_xyz*q_yzx*Q_yzx_to_xyz.inverse();
    
    
//     if (liom_counter == 50 && first_liom_flag) {
#ifdef USE_LIOM  
	init_rn = t_yzx;
#else  
// 	init_rn = t_xyz;
#endif
	
	
// 	Gps gps;
// 	gpsBuf_.getLastMeas(gps);
// 	gpsRef_ = gps;
// 
	first_liom_flag = false;
// 	
// 	
//     } else {
// 	liom_counter++;
//     }
#ifdef USE_LIOM  
    Odometry odom(mappingMsg->header.stamp.toSec(), q_yzx, t_yzx);
    mappingBuf_.addMeas(odom, mappingMsg->header.stamp.toSec());  
#else  
    Odometry odom(mappingMsg->header.stamp.toSec(), q_xyz, t_xyz);
    mappingBuf_.addMeas(odom, mappingMsg->header.stamp.toSec());  
#endif

#ifdef USE_LIOM  
//     if (counter >= 2)
	odomFlag = true;
//     else
// 	counter++; 
#else  
    odomFlag = true;
#endif
}


void Evaluator::loamCallback(const nav_msgs::Odometry::ConstPtr& loamMsg)
{

}




void Evaluator::publishGpsOdometry(const Gps& gps, const V3D& rn_xyz)
{
#ifdef USE_LIOM
    V3D rn = rn_xyz;
    Q4D Cbn = Q4D::Identity();
    
    gpsOdometry.header.frame_id = "/world";
    gpsOdometry.child_frame_id = "/gps_odom";


    gpsOdometry.header.stamp = ros::Time().fromSec(gps.time);
    gpsOdometry.pose.pose.orientation.x = Cbn.x();
    gpsOdometry.pose.pose.orientation.y = Cbn.y();
    gpsOdometry.pose.pose.orientation.z = Cbn.z();
    gpsOdometry.pose.pose.orientation.w = Cbn.w();
    gpsOdometry.pose.pose.position.x = rn[0];
    gpsOdometry.pose.pose.position.y = rn[1];
    gpsOdometry.pose.pose.position.z = rn[2];
    
    gpsOdometry.twist.twist.linear.x = sumDistance_;    	
    gpsOdometry.twist.twist.linear.y = mappingDrift_;
    gpsOdometry.twist.twist.linear.z = mappingDriftRate_;	
    gpsOdometry.twist.twist.angular.y = odomDrift_;
    gpsOdometry.twist.twist.angular.z = odomDriftRate_;	
    pubGpsOdometry_.publish(gpsOdometry);
    

    tf::TransformBroadcaster tfBroadcaster; 
    tf::StampedTransform gpsOdometryTrans;
    gpsOdometryTrans.frame_id_ = "/world";
    gpsOdometryTrans.child_frame_id_ = "/gps_odom"; 
    gpsOdometryTrans.stamp_ = ros::Time().fromSec(gps.time);
    gpsOdometryTrans.setRotation(tf::Quaternion(Cbn.x(), Cbn.y(), Cbn.z(), Cbn.w()));
    gpsOdometryTrans.setOrigin(tf::Vector3(rn[0], rn[1], rn[2]));
    tfBroadcaster.sendTransform(gpsOdometryTrans);
    
#else
    

    Q4D Cbn = Q4D::Identity();
    Cbn = Q_xyz_to_yzx * Cbn * Q_xyz_to_yzx.inverse();
    V3D rn = Q_xyz_to_yzx * rn_xyz;  

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
    
    gpsOdometry.twist.twist.linear.x = sumDistance_;    	
    gpsOdometry.twist.twist.linear.y = mappingDrift_;
    gpsOdometry.twist.twist.linear.z = mappingDriftRate_;	
    gpsOdometry.twist.twist.angular.y = odomDrift_;
    gpsOdometry.twist.twist.angular.z = odomDriftRate_;	
    pubGpsOdometry_.publish(gpsOdometry);
    
    tf::TransformBroadcaster tfBroadcaster; 
    tf::StampedTransform gpsOdometryTrans;
    gpsOdometryTrans.frame_id_ = "/camera_init";
    gpsOdometryTrans.child_frame_id_ = "/gps_odom"; 
    gpsOdometryTrans.stamp_ = ros::Time().fromSec(gps.time);
    gpsOdometryTrans.setRotation(tf::Quaternion(Cbn.x(), Cbn.y(), Cbn.z(), Cbn.w()));
    gpsOdometryTrans.setOrigin(tf::Vector3(rn[0], rn[1], rn[2]));
    tfBroadcaster.sendTransform(gpsOdometryTrans);
#endif  
    
    

    

}


    
    
//     if (odomBuf_.empty() || gpsBuf_.empty())
// 	return; 
//     
//     Gps gps;
//     gpsBuf_.getLastMeas(gps);
//     V3D rn = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - gpsRef_.pn());
//     
//     if (lastGps_.time == 0.0) {
// 	lastGps_ = gps;
// 	gpsBuf_.clean(gps.time);
// 	return;
//     }
// 
//     double lastOdomTime;
//     odomBuf_.getLastTime(lastOdomTime);
//     if (lastOdomTime < gps.time) {
// 	ROS_WARN("Wait for more Odom measurement!");
// 	return;
//     }    
//     
//     odomBuf_.itMeas_ = odomBuf_.measMap_.lower_bound(gps.time);
//     double dt_front = gps.time - odomBuf_.itMeas_->first;
//     Odometry odom_front = odomBuf_.itMeas_->second;
//     
//     odomBuf_.itMeas_ = odomBuf_.measMap_.upper_bound(gps.time);
//     double dt_back = odomBuf_.itMeas_->first - gps.time;
//     Odometry odom_back = odomBuf_.itMeas_->second;
//  
//     V3D translation_gps = EarthParams::getDpr(gpsRef_.pn()) * (gps.pn() - lastGps_.pn());
//     sumDistance_ += (translation_gps).norm();
//     //sumDistance_ += (translation_gps.segment<2>(0)).norm();
//     double transDrift;
//     double driftRate;
//     
//     
//     if (dt_front < dt_back) {
// 	transDrift = (rn - odom_front.translation).norm();
// 	driftRate = 100.0 * transDrift / sumDistance_;
// cout << "dt: " << dt_front << ", driftRate: " << driftRate << "%, transDrift: " << transDrift << ", sumDistance: " << sumDistance_<< endl;
// 
//     } else {
// 	transDrift = (rn - odom_back.translation).norm();
// 	driftRate = 100.0 * transDrift / sumDistance_;
// cout << "dt: " << dt_back << ", driftRate: " << driftRate << "%, transDrift: " << transDrift << ", sumDistance: " << sumDistance_<< endl;
// 
//     }
//     cout << "rn: " << rn.transpose() << endl;
// 
//     lastGps_ = gps;
//     gpsBuf_.clean(gps.time);







};































































