#ifndef _STATE_ESTIMATOR_HPP_
#define _STATE_ESTIMATOR_HPP_

// #include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include "cloud_msgs/cloud_info.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>

#include "integrationBase.h"
#include <tic_toc.h>
#include <math_utils.h>
#include <parameters.h>
#include <KalmanFilter.hpp>
#include <sensor_utils.hpp>
#include <integrationBase.h>

using namespace Eigen;
using namespace std;
using namespace math_utils;
using namespace sensor_utils;
using namespace parameter;
using namespace filter;

namespace fusion {
    
typedef pcl::PointXYZI  PointType;

const int LINE_NUM_ = 16;
const int SCAN_NUM_ = 1800;

struct Smooth{ 
	Smooth () {
		value = 0.0;
		ind = 0;
	}
	double value;
	size_t ind;
};

struct byValue{ 
	bool operator()(Smooth const &left, Smooth const &right) { 
		return left.value < right.value;
	}
};

struct Scan {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
	enum Status {
		STATUS_INITIALIZATION,
		STATUS_RUNNING
		};
    
	Scan()
		: id_(scanCounter_++) {

		distPointCloud_.reset(new pcl::PointCloud<PointType>());
		undistPointCloud_.reset(new pcl::PointCloud<PointType>());
		outlierPointCloud_.reset(new pcl::PointCloud<PointType>());
		cornerPointsSharp_.reset(new pcl::PointCloud<PointType>());
		cornerPointsLessSharp_.reset(new pcl::PointCloud<PointType>());
		surfPointsFlat_.reset(new pcl::PointCloud<PointType>());
		surfPointsLessFlat_.reset(new pcl::PointCloud<PointType>());
		cloudInfo_.reset(new cloud_msgs::cloud_info());
		
		cornerPointsLessSharpYZX_.reset(new pcl::PointCloud<PointType>());
		surfPointsLessFlatYZX_.reset(new pcl::PointCloud<PointType>());
		outlierPointCloudYZX_.reset(new pcl::PointCloud<PointType>());

		cloudCurvature_.resize(LINE_NUM*SCAN_NUM);
		cloudSmoothness_.resize(LINE_NUM*SCAN_NUM);
		
		reset();
	
	}


	~Scan() {
		distPointCloud_.reset();
		undistPointCloud_.reset();
		outlierPointCloud_.reset();
		cornerPointsSharp_.reset();
		cornerPointsLessSharp_.reset();
		surfPointsFlat_.reset();
		surfPointsLessFlat_.reset();
		cloudInfo_.reset();
		
		cornerPointsLessSharpYZX_.reset();
		surfPointsLessFlatYZX_.reset();
		outlierPointCloudYZX_.reset();
		
	};
    
    
	void reset() {
		time_ = 0.0;

		distPointCloud_->clear();
		undistPointCloud_->clear();
		outlierPointCloud_->clear();
		cornerPointsSharp_->clear();
		cornerPointsLessSharp_->clear();
		surfPointsFlat_->clear();
		surfPointsLessFlat_->clear();
				
		cornerPointsLessSharpYZX_->clear();
		surfPointsLessFlatYZX_->clear();
		outlierPointCloudYZX_->clear();
		
		cloudCurvature_.assign(LINE_NUM*SCAN_NUM,0.0);
		cloudSmoothness_.assign(LINE_NUM*SCAN_NUM,Smooth());

	}
    
	void setPcl(double time, pcl::PointCloud<PointType>::Ptr distPointCloud, cloud_msgs::cloud_info cloudInfo, pcl::PointCloud<PointType>::Ptr outlierPointCloud) {
		distPointCloud_ = distPointCloud;
		*(cloudInfo_) = cloudInfo;
		outlierPointCloud_ = outlierPointCloud;
		time_ = time;
	}
    
	//!@ScanInfo
	static int scanCounter_;
	int id_;
	double time_;
	//!@PointCloud
	pcl::PointCloud<PointType>::Ptr distPointCloud_;
	pcl::PointCloud<PointType>::Ptr undistPointCloud_;
	pcl::PointCloud<PointType>::Ptr outlierPointCloud_;
	cloud_msgs::cloud_info::Ptr cloudInfo_;
	//!@PclFeatures
	std::vector<double> cloudCurvature_;
	std::vector<Smooth> cloudSmoothness_;
	
	int cloudNeighborPicked_[LINE_NUM_*SCAN_NUM_];
	int cloudLabel_[LINE_NUM_*SCAN_NUM_];

	pcl::PointCloud<PointType>::Ptr cornerPointsSharp_;
	pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp_;
	pcl::PointCloud<PointType>::Ptr surfPointsFlat_;
	pcl::PointCloud<PointType>::Ptr surfPointsLessFlat_;
	
	pcl::PointCloud<PointType>::Ptr cornerPointsLessSharpYZX_;
	pcl::PointCloud<PointType>::Ptr surfPointsLessFlatYZX_;
	pcl::PointCloud<PointType>::Ptr outlierPointCloudYZX_;

};
typedef shared_ptr<Scan> ScanPtr;



class StateEstimator {
   
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	enum FusionStatus {
		STATUS_INIT = 0,
		STATUS_FIRST_SCAN = 1,
		STATUS_SECOND_SCAN = 2,
		STATUS_RUNNING = 3,
		STATUS_RESET = 4,
	};
    
 
	StateEstimator() {
	filter_ = new StatePredictor();
	
	downSizeFilter_.setLeafSize(0.2, 0.2, 0.2);
	kdtreeCorner_.reset(new pcl::KdTreeFLANN<PointType>());
	kdtreeSurf_.reset(new pcl::KdTreeFLANN<PointType>());
	scan_new_.reset(new Scan());
	scan_last_.reset(new Scan());

	keypoints_.reset(new pcl::PointCloud<PointType>());
	jacobians_.reset(new pcl::PointCloud<PointType>());  
	keypointCorns_.reset(new pcl::PointCloud<PointType>());
	keypointSurfs_.reset(new pcl::PointCloud<PointType>());
	jacobianCoffCorns.reset(new pcl::PointCloud<PointType>());
	jacobianCoffSurfs.reset(new pcl::PointCloud<PointType>());

	surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
	surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());
	
	pointSelCornerInd.resize(LINE_NUM*SCAN_NUM);
	pointSearchCornerInd1.resize(LINE_NUM*SCAN_NUM);
	pointSearchCornerInd2.resize(LINE_NUM*SCAN_NUM);
	pointSelSurfInd.resize(LINE_NUM*SCAN_NUM);
	pointSearchSurfInd1.resize(LINE_NUM*SCAN_NUM);
	pointSearchSurfInd2.resize(LINE_NUM*SCAN_NUM);
	pointSearchSurfInd3.resize(LINE_NUM*SCAN_NUM);
	
	
	status_ = STATUS_INIT;
	globalState_.setIdentity();
	globalStateYZX_.setIdentity();
	
	R_yzx_to_xyz << 0., 0., 1.,
			1., 0., 0.,
			0., 1., 0.;
	R_xyz_to_yzx = R_yzx_to_xyz.transpose();
	Q_yzx_to_xyz = 	R_yzx_to_xyz;
	Q_xyz_to_yzx = R_xyz_to_yzx;
	
	gravity_feedback << 0, 0, -G0;

	};
	~StateEstimator() {
		delete filter_;
		delete preintegration_;
	};
    
	inline const double getTime () const { 
		return filter_->time_; 
	};
    
	inline bool isInitialized () const {
		return status_ != STATUS_INIT;
	}
    
	void initialization(double time, const Imu& imu, pcl::PointCloud<PointType>::Ptr distortedPointCloud, cloud_msgs::cloud_info cloudInfo, pcl::PointCloud<PointType>::Ptr outlierPointCloud, V3D ba_init, V3D bw_init) {
		scan_new_->setPcl(time, distortedPointCloud, cloudInfo, outlierPointCloud);
		undistortPcl(scan_new_);
		calculateSmoothness(scan_new_);
		markOccludedPoints(scan_new_);
		extractFeatures(scan_new_);
		if (scan_new_->cornerPointsLessSharp_->points.size() < 10 || scan_new_->surfPointsLessFlat_->points.size() < 100) {
				ROS_WARN("Too few features. Initialization fails.");
				scan_new_.reset(new Scan());
				return;
		}
	
		kdtreeCorner_->setInputCloud(scan_new_->cornerPointsLessSharp_);
		kdtreeSurf_->setInputCloud(scan_new_->surfPointsLessFlat_);

		//cout << "initialize" << endl;
		Fk_.resize(GlobalState::DIM_OF_STATE_,GlobalState::DIM_OF_STATE_);
		Gk_.resize(GlobalState::DIM_OF_STATE_,GlobalState::DIM_OF_NOISE_);
		Pk_.resize(GlobalState::DIM_OF_STATE_,GlobalState::DIM_OF_STATE_);
		Qk_.resize(GlobalState::DIM_OF_NOISE_,GlobalState::DIM_OF_NOISE_);
		IKH_.resize(GlobalState::DIM_OF_STATE_,GlobalState::DIM_OF_STATE_);
		
		Fk_.setIdentity();
		Gk_.setZero();
		Pk_.setZero();
		Qk_.setZero();
		
		V3D acc_init, gyr_init;
		acc_init = imu.acc;
		gyr_init = imu.gyr;

		V3D rn_init = V3D(0,0,0);
		V3D vn_init = V3D(0,0,0);
		
					double roll_init, pitch_init, yaw_init = deg2rad(0.0);
		StatePredictor::calcRP(acc_init-ba_init, roll_init, pitch_init);

		globalState_ = GlobalState(rn_init, vn_init, rpy2Quat(V3D(roll_init, pitch_init, yaw_init)), ba_init, bw_init);
		filter_->initialization(time, rn_init, vn_init, ba_init, bw_init, acc_init, gyr_init);
		
		linState_.setIdentity();

		scan_last_.swap(scan_new_);
		scan_new_->reset();
		
		status_ = STATUS_RUNNING;
	}
    

	V3D pos_;
	V3D vel_;
	M3D quad_;
	V3D acc_0;
	V3D gyr_0;
	void processImu(double dt, const Vector3d &acc, const Vector3d &gyr) {

		switch (status_) {
			case STATUS_INIT:
				break;
			case STATUS_FIRST_SCAN:
				preintegration_->push_back(dt, acc, gyr);
				filter_->time_ += dt;
				acc_0 = acc;
				gyr_0 = gyr;
				break;
			case STATUS_RUNNING:
			//cout << "IMU prediction" << endl;
// 				filter_->predict(dt, acc, gyr, globalState_.qbn_, true);
				filter_->predict(dt, acc, gyr, true);
// 				filter_->predict(dt, acc, gyr, gravity_feedback, true);
				break;
			default:
				break;
		}
	
		Vector3d un_acc_0 = quad_ * (acc_0 - INIT_BA) + filter_->state_.gn_;
		Vector3d un_gyr = 0.5 * (gyr_0 + gyr) - INIT_BW;
		quad_ *= math_utils::deltaQ(un_gyr * dt).toRotationMatrix();
		Vector3d un_acc_1 = quad_ * (acc - INIT_BA) + filter_->state_.gn_;
		Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
		pos_ += dt * vel_ + 0.5 * dt * dt * un_acc;
		vel_ += dt * un_acc;

		acc_0 = acc;
		gyr_0 = gyr;

// 	M3D R;
// 	R << -0.1762,-0.9816,0.0738,
// 	    0.9831,-0.1793,-0.0366,
// 	    0.0492,0.0661,0.9966;
// 	V3D ypr = math_utils::R2ypr(R);
// 	cout << "ypr: " << rad2deg(ypr).transpose() << endl;;
	}	
    
	double duration_fea_ = 0;
	double duration_opt_ = 0;
	double num_of_edge_ = 0;
	double num_of_surf_ = 0;
	int lidarCounter = 0;
	void processPcl(double time, const Imu& imu, pcl::PointCloud<PointType>::Ptr distortedPointCloud, cloud_msgs::cloud_info cloudInfo, pcl::PointCloud<PointType>::Ptr outlierPointCloud) {
		TicToc ts_fea;
		scan_new_->setPcl(time, distortedPointCloud, cloudInfo, outlierPointCloud);
		undistortPcl(scan_new_);
		calculateSmoothness(scan_new_);
		markOccludedPoints(scan_new_);
		extractFeatures(scan_new_);
		imu_last_ = imu;
		double time_fea = ts_fea.toc();

		TicToc ts_opt;
		switch (status_) {
			case STATUS_INIT:
				if(processFirstScan()) status_ = STATUS_FIRST_SCAN;
				break;
			case STATUS_FIRST_SCAN:
				if(processSecondScan()) status_ = STATUS_RUNNING;
				else status_ = STATUS_INIT;
				break;
			case STATUS_RUNNING:
				if(!processScan()) status_ = STATUS_RUNNING;
				break;
		}
		double time_opt = ts_opt.toc();
	
		if (VERBOSE) {
				duration_fea_ = (duration_fea_*lidarCounter + time_fea)/(lidarCounter + 1);
				duration_opt_ = (duration_opt_*lidarCounter + time_opt)/(lidarCounter + 1);
				num_of_edge_ = (num_of_edge_*lidarCounter + scan_last_->cornerPointsLessSharpYZX_->points.size())/(lidarCounter + 1);
				num_of_surf_ = (num_of_surf_*lidarCounter + scan_last_->surfPointsLessFlatYZX_->points.size())/(lidarCounter + 1);
				lidarCounter++;
				
				
				
				cout << "Feature Extraction: time: " << duration_fea_  << endl;
	// 	    cout << "Feature Extraction: corners: " << scan_last_->cornerPointsLessSharpYZX_->points.size() << ", surfs: " << scan_last_->surfPointsLessFlatYZX_->points.size() << endl;
				cout << "Feature Extraction: corners: " << num_of_edge_ << ", surfs: " << num_of_surf_ << endl;
				cout << "State Estimation: time: " << duration_opt_ << endl;
		}
	    
	}
    
	
	void correctRollPitch(const double& roll, const double& pitch) {
		V3D rpy = math_utils::Q2rpy(globalState_.qbn_);
		Q4D quad = math_utils::rpy2Quat(V3D(roll, pitch, rpy[2]));
		
		
// 		globalState_.rn_ = quad * globalState_.qbn_.inverse() * globalState_.rn_;
		globalState_.qbn_ = quad;
		
	}

	void correctOrientation(const Q4D& quad) {
		// globalState_.rn_ = quad * globalState_.qbn_.inverse() * globalState_.rn_;
		// globalState_.vn_ = quad * globalState_.qbn_.inverse() * globalState_.vn_;
		
		globalState_.qbn_ = quad;
	}

	bool processFirstScan() {
	//cout << "processFirstScan" << endl;
		if (scan_new_->cornerPointsLessSharp_->points.size() < 10 || scan_new_->surfPointsLessFlat_->points.size() < 100) {
			ROS_WARN("Wait for more features for initialization...");
			scan_new_.reset(new Scan());
			return false;
		}
	
		Fk_.resize(GlobalState::DIM_OF_STATE_,GlobalState::DIM_OF_STATE_);
		Gk_.resize(GlobalState::DIM_OF_STATE_,GlobalState::DIM_OF_NOISE_);
		Pk_.resize(GlobalState::DIM_OF_STATE_,GlobalState::DIM_OF_STATE_);
		Qk_.resize(GlobalState::DIM_OF_NOISE_,GlobalState::DIM_OF_NOISE_);
		IKH_.resize(GlobalState::DIM_OF_STATE_,GlobalState::DIM_OF_STATE_);
		
		Fk_.setIdentity();
		Gk_.setZero();
		Pk_.setZero();
		Qk_.setZero();
	
		linState_.setIdentity();

		//preintegration_ = new integration::IntegrationBase(imu_last_.acc, imu_last_.gyr, V3D(0,0,0), INIT_BW);

		preintegration_ = new integration::IntegrationBase(imu_last_.acc, imu_last_.gyr, INIT_BA, INIT_BW);

		filter_->initialization(scan_new_->time_, V3D(0,0,0), V3D(0,0,0), V3D(0,0,0), V3D(0,0,0), imu_last_.acc, imu_last_.gyr);

		kdtreeCorner_->setInputCloud(scan_new_->cornerPointsLessSharp_);
		kdtreeSurf_->setInputCloud(scan_new_->surfPointsLessFlat_);

		pos_.setZero();
		vel_.setZero();
		quad_.setIdentity();

		scan_last_.swap(scan_new_);
		scan_new_.reset(new Scan());
	
		return true;
	}
    
	bool processSecondScan() {
		//cout << "processSecondScan" << endl;
		if (scan_new_->cornerPointsLessSharp_->points.size() < 10 || scan_new_->surfPointsLessFlat_->points.size() < 100) {
				ROS_WARN("Wait for more features for initialization...");
				scan_new_.reset(new Scan());
				return false;
		}
	
		V3D pl; Q4D ql; V3D v0, v1, ba0, bw0;
		ba0.setZero();
		ql = preintegration_->delta_q;
		pl = preintegration_->delta_p + 0.5*linState_.gn_*preintegration_->sum_dt*preintegration_->sum_dt - 0.5*ba0*preintegration_->sum_dt*preintegration_->sum_dt;
		estimateTransform(scan_last_, scan_new_, pl ,ql);
		align4(pl ,ql, v0, v1, ba0, bw0);
		
		V3D r1 = pl;
		filter_->initialization(scan_new_->time_, r1, v1, ba0, bw0, imu_last_.acc, imu_last_.gyr);
		
					double roll_init, pitch_init, yaw_init = deg2rad(0.0);
		StatePredictor::calcRP(imu_last_.acc-ba0, roll_init, pitch_init);
		globalState_ = GlobalState(r1, v1, rpy2Quat(V3D(roll_init, pitch_init, yaw_init)), ba0, bw0);
		
		updatePointCloud(); 
		
		scan_last_.swap(scan_new_);
		scan_new_.reset(new Scan());
	
		return true;
	}

	bool processScan() {
		//cout << "processScan" << endl;
		if (scan_new_->cornerPointsLessSharp_->points.size() <= 5 || scan_new_->surfPointsLessFlat_->points.size() <= 10) {
				ROS_WARN("Insufficient features...State estimation fails.");
	// 	    linState_ = filter_->state_;
	// 	    integrateTransformation();
	// 	    filter_->reset();
	// 	    updatePointCloud();
	// 	    scan_last_.swap(scan_new_);
	// 	    scan_new_.reset(new Scan());
				return false;
		} 
	
		performIEKF();
		integrateTransformation();
		filter_->reset(1);
		
		double roll, pitch;
		calcRP(filter_->state_.gn_, roll, pitch);
		correctRollPitch(roll, pitch);
		
		
		updatePointCloud();
		
		scan_last_.swap(scan_new_);
		scan_new_.reset(new Scan());
		
		return true;
	}
        
        
    
	void processPcl(double time, pcl::PointCloud<PointType>::Ptr distortedPointCloud, cloud_msgs::cloud_info cloudInfo, pcl::PointCloud<PointType>::Ptr outlierPointCloud) {

		if (filter_->isInitialized() == false)
				return;
		
		TicToc ts_fea;
		scan_new_->setPcl(time, distortedPointCloud, cloudInfo, outlierPointCloud);
		double time_fea = ts_fea.toc();

		TicToc ts_ext;
		undistortPcl(scan_new_);
		calculateSmoothness(scan_new_);
		markOccludedPoints(scan_new_);
		extractFeatures(scan_new_);
		double time_ext = ts_ext.toc();
		if (scan_last_->cornerPointsLessSharp_->points.size() <= 5 || scan_last_->surfPointsLessFlat_->points.size() <= 20) {
			cout << "Insufficient features...State estimation fails." << endl;
			linState_ = filter_->state_;
			integrateTransformation();
			filter_->reset();
			updatePointCloud();
			scan_last_.swap(scan_new_);
			scan_new_.reset(new Scan());
			return;
		} else {
				
		}


		TicToc ts_kf;
		TicToc ts_1;
		performIEKF();
		double time_1 = ts_1.toc();
		// cout << "time_1: " << time_1 << endl;

		TicToc ts_2;    
		integrateTransformation();
		double time_2 = ts_2.toc();
		// cout << "time_2: " << time_2 << endl;	
			

		TicToc ts_3; 
		filter_->reset();
		updatePointCloud();
		double time_3 = ts_3.toc();
		// cout << "time_3: " << time_3 << endl;

		double time_kf = ts_kf.toc();

		scan_last_.swap(scan_new_);
	}


// TicToc ts_fea;
// double time_fea = ts_fea.toc();

	void performIEKF() {

		Pk_ = filter_->covariance_;
		GlobalState filterState = filter_->state_;
					linState_ = filterState;

		double residualNorm = 1e6;
		bool hasConverged = false;
		bool hasDiverged = false;
		const unsigned int DIM_OF_STATE = GlobalState::DIM_OF_STATE_;
		for (int iter = 0; iter < NUM_ITER && !hasConverged && !hasDiverged; iter++) {
	    keypointSurfs_->clear();
	    jacobianCoffSurfs->clear();
	    keypointCorns_->clear();
	    jacobianCoffCorns->clear();  

	    findCorrespondingSurfFeatures(scan_last_, scan_new_, keypointSurfs_, jacobianCoffSurfs, iter);
	    if (keypointSurfs_->points.size() < 10) {
				if (VERBOSE) {
					cout << "Insufficient matched surfs..." << endl;
				}
			}    
	    findCorrespondingCornerFeatures(scan_last_, scan_new_, keypointCorns_, jacobianCoffCorns, iter);
	    if (keypointCorns_->points.size() < 5) {
				if (VERBOSE) {
					cout << "Insufficient matched corners..." << endl;
				}
	    }

	    keypoints_->clear();
	    jacobians_->clear();
	    (*keypoints_) += (*keypointSurfs_);
	    (*keypoints_) += (*keypointCorns_);
	    (*jacobians_) += (*jacobianCoffSurfs); 
	    (*jacobians_) += (*jacobianCoffCorns); 
	    
	    const unsigned int DIM_OF_MEAS = keypoints_->points.size();
	    //cout << "DIM_OF_MEAS: " << DIM_OF_MEAS << endl;
	    residual_.resize(DIM_OF_MEAS);
	    Hk_.resize(DIM_OF_MEAS, DIM_OF_STATE);
	    Rk_.resize(DIM_OF_MEAS, DIM_OF_MEAS);
	    Kk_.resize(DIM_OF_STATE,DIM_OF_MEAS);
	    Py_.resize(DIM_OF_MEAS, DIM_OF_MEAS);
	    Pyinv_.resize(DIM_OF_MEAS, DIM_OF_MEAS);
	    
	    Hk_.setZero();
	    V3D axis = Quat2axis(linState_.qbn_);
			for (int i=0; i<DIM_OF_MEAS; ++i) {
				V3D P2xyz(keypoints_->points[i].x, keypoints_->points[i].y, keypoints_->points[i].z);
				V3D coff_xyz(jacobians_->points[i].x, jacobians_->points[i].y, jacobians_->points[i].z);
				residual_(i) = LIDAR_SCALE * jacobians_->points[i].intensity;
				// Hk_.block<1, 3>(i, GlobalState::att_) = coff_xyz.transpose() * (-linState_.qbn_.toRotationMatrix() * skew(P2xyz));
				// Hk_.block<1, 3>(i, GlobalState::pos_) = coff_xyz.transpose() * M3D::Identity();

				Hk_.block<1, 3>(i, GlobalState::att_) = coff_xyz.transpose() *  (-linState_.qbn_.toRotationMatrix() * skew(P2xyz)) * Rinvleft(-axis);
				Hk_.block<1, 3>(i, GlobalState::pos_) = coff_xyz.transpose() *  M3D::Identity();
			}

	    VXD cov = VXD::Zero(DIM_OF_MEAS);
	    for (int i=0; i<DIM_OF_MEAS; ++i) {
				cov[i] = LIDAR_STD*LIDAR_STD;
	    }
	    Rk_ = cov.asDiagonal();
	    
	    Py_ = Hk_*Pk_*Hk_.transpose() + Rk_; // S = H * P * H.transpose() + R;
	    Pyinv_.setIdentity(); // solve Ax=B
	    Py_.llt().solveInPlace(Pyinv_);
	    Kk_ = Pk_*Hk_.transpose()*Pyinv_; // K = P*H.transpose()*S.inverse()

	    filterState.boxMinus(linState_, difVecLinInv_);
	    updateVec_ = -Kk_*(residual_+Hk_*difVecLinInv_)+difVecLinInv_;

	    bool hasNaN = false;
	    for(int i=0; i<updateVec_.size(); i++){
				if(isnan(updateVec_[i])) {
						updateVec_[i]=0;
						hasNaN = true;
				} 
	    }
	    if (hasNaN == true) {
				ROS_WARN("System diverges Because of NaN!");
				cout << iter << " iterations. System diverges because of NaN." << endl;
				hasDiverged = true;
				break;
	    } 
	    
	    if (residual_.norm() > residualNorm*10) {
				ROS_WARN("System diverges!!!");
				hasDiverged = true;
				break;
	    }
	    
	    linState_.boxPlus(updateVec_,linState_);
	    
	    updateVecNorm_ = updateVec_.norm();
	    //cout << "---" << iter << ", residualNorm: " << residualNorm << endl;
	    if (updateVecNorm_ <= 1e-2) {
		//cout << iter << " iterations. System converges!" << endl;
				hasConverged = true;
	    }
	    
	    residualNorm = residual_.norm();
		}
		
		if (hasDiverged == true) {
			ROS_WARN("======Using ICP Method======");
			V3D t = filterState.rn_; 
			Q4D q = filterState.qbn_; 
			estimateTransform(scan_last_, scan_new_, t, q);  
			filterState.rn_ = t;
			filterState.qbn_ = q;
			filter_->update(filterState, Pk_);  

		} else {
	    IKH_ = Eigen::Matrix<double, 18, 18>::Identity() - Kk_*Hk_;
	    Pk_ = IKH_*Pk_*IKH_.transpose() + Kk_*Rk_*Kk_.transpose();
	    enforceSymmetry(Pk_);
	    filter_->update(linState_, Pk_);	    
		}
	}
   
//     // ICP Method
//     V3D t = linState_.rn_; 
//     Q4D q = linState_.qbn_; 
//     estimateTransform(scan_last_, scan_new_, t, q);  
//     filterState.rn_ = t;
//     filterState.qbn_ = q;
//     linState_ = filterState;
//     filter_->update(filterState, Pk_);  
	
void calcRP(const V3D& fbib, double& roll, double& pitch) {
    pitch = -sign(fbib.z()) * asin(fbib.x()/ G0);
    roll = sign(fbib.z()) * asin(fbib.y() / G0);
}

	void integrateTransformation() {
		GlobalState filterState = filter_->state_;
		globalState_.rn_ = globalState_.qbn_ * filterState.rn_ + globalState_.rn_;
		globalState_.qbn_  = globalState_.qbn_ * filterState.qbn_;
		globalState_.vn_ = globalState_.qbn_ * filterState.qbn_.inverse() * filterState.vn_;
		globalState_.ba_ = filterState.ba_;
		globalState_.bw_ = filterState.bw_;
		globalState_.gn_ = globalState_.qbn_ * filterState.gn_;
		

	}

	void undistortPcl(ScanPtr scan) {
		bool halfPassed = false;
		scan->undistPointCloud_->clear();
		pcl::PointCloud<PointType>::Ptr distPointCloud = scan->distPointCloud_;
		cloud_msgs::cloud_info::Ptr segInfo = scan->cloudInfo_;
		int size = distPointCloud->points.size();
		PointType point;
		for (int i = 0; i < size; i++) {
			rotatePoint(&distPointCloud->points[i], &point);    

			double ori = -atan2(point.y, point.x);
			if (!halfPassed) {
				if (ori < segInfo->startOrientation - M_PI / 2)
					ori += 2 * M_PI;
				else if (ori > segInfo->startOrientation + M_PI * 3 / 2)
					ori -= 2 * M_PI;

			if (ori - segInfo->startOrientation > M_PI)
				halfPassed = true;
			} else {
				ori += 2 * M_PI;

				if (ori < segInfo->endOrientation - M_PI * 3 / 2)
					ori += 2 * M_PI;
				else if (ori > segInfo->endOrientation + M_PI / 2)
					ori -= 2 * M_PI;
			}
			double relTime = (ori - segInfo->startOrientation) / segInfo->orientationDiff; 
			point.intensity = int(distPointCloud->points[i].intensity) + SCAN_PERIOD * relTime;
			
			scan->undistPointCloud_->push_back(point);
		}
	}
    
	void calculateSmoothness(ScanPtr scan) {
		int cloudSize = scan->undistPointCloud_->points.size();
		cloud_msgs::cloud_info::Ptr segInfo = scan->cloudInfo_;
		for (int i = 5; i < cloudSize - 5; i++) {
	    double diffRange = segInfo->segmentedCloudRange[i-5] + segInfo->segmentedCloudRange[i-4]
			    + segInfo->segmentedCloudRange[i-3] + segInfo->segmentedCloudRange[i-2]
			    + segInfo->segmentedCloudRange[i-1] - segInfo->segmentedCloudRange[i] * 10
			    + segInfo->segmentedCloudRange[i+1] + segInfo->segmentedCloudRange[i+2]
			    + segInfo->segmentedCloudRange[i+3] + segInfo->segmentedCloudRange[i+4]
			    + segInfo->segmentedCloudRange[i+5];            
	    scan->cloudCurvature_[i] = diffRange*diffRange;

	    scan->cloudNeighborPicked_[i] = 0;
	    scan->cloudLabel_[i] = 0;
	    scan->cloudSmoothness_[i].value = scan->cloudCurvature_[i];
	    scan->cloudSmoothness_[i].ind = i;
		}
	}
    
	void markOccludedPoints(ScanPtr scan) {
		int cloudSize = scan->undistPointCloud_->points.size();
		cloud_msgs::cloud_info::Ptr segInfo = scan->cloudInfo_;
		for (int i = 5; i < cloudSize - 6; ++i){
				float depth1 = segInfo->segmentedCloudRange[i];
				float depth2 = segInfo->segmentedCloudRange[i+1];
				int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[i+1] - segInfo->segmentedCloudColInd[i]));
				if (columnDiff < 10){
					if (depth1 - depth2 > 0.3){
						scan->cloudNeighborPicked_[i - 5] = 1;
						scan->cloudNeighborPicked_[i - 4] = 1;
						scan->cloudNeighborPicked_[i - 3] = 1;
						scan->cloudNeighborPicked_[i - 2] = 1;
						scan->cloudNeighborPicked_[i - 1] = 1;
						scan->cloudNeighborPicked_[i] = 1;
					} else if (depth2 - depth1 > 0.3){
						scan->cloudNeighborPicked_[i + 1] = 1;
						scan->cloudNeighborPicked_[i + 2] = 1;
						scan->cloudNeighborPicked_[i + 3] = 1;
						scan->cloudNeighborPicked_[i + 4] = 1;
						scan->cloudNeighborPicked_[i + 5] = 1;
						scan->cloudNeighborPicked_[i + 6] = 1;
					}
				}
				float diff1 = std::abs(segInfo->segmentedCloudRange[i-1] - segInfo->segmentedCloudRange[i]);
				float diff2 = std::abs(segInfo->segmentedCloudRange[i+1] - segInfo->segmentedCloudRange[i]);
				if (diff1 > 0.02 * segInfo->segmentedCloudRange[i] && diff2 > 0.02 * segInfo->segmentedCloudRange[i])
					scan->cloudNeighborPicked_[i] = 1;
		}
	}
    
    
	pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
	pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;
	void extractFeatures(ScanPtr scan) {
		cloud_msgs::cloud_info::Ptr segInfo = scan->cloudInfo_;
	
		scan->cornerPointsSharp_->clear();
		scan->cornerPointsLessSharp_->clear();
		scan->surfPointsFlat_->clear();
		scan->surfPointsLessFlat_->clear();

		for (int i = 0; i < LINE_NUM; i++) {

	    surfPointsLessFlatScan->clear();

	    for (int j = 0; j < 6; j++) {

				int sp = (segInfo->startRingIndex[i] * (6 - j)    + segInfo->endRingIndex[i] * j) / 6;
				int ep = (segInfo->startRingIndex[i] * (5 - j)    + segInfo->endRingIndex[i] * (j + 1)) / 6 - 1;

				if (sp >= ep)
					continue;

				std::sort(scan->cloudSmoothness_.begin()+sp, scan->cloudSmoothness_.begin()+ep, byValue());

				int largestPickedNum = 0;
				for (int k = ep; k >= sp; k--) {
					int ind = scan->cloudSmoothness_[k].ind;
					if (scan->cloudNeighborPicked_[ind] == 0 &&
							scan->cloudCurvature_[ind] > EDGE_THRESHOLD &&
							segInfo->segmentedCloudGroundFlag[ind] == false) {
					
						largestPickedNum++;
						if (largestPickedNum <= 2) {
							scan->cloudLabel_[ind] = 2;
							scan->cornerPointsSharp_->push_back(scan->undistPointCloud_->points[ind]);
							scan->cornerPointsLessSharp_->push_back(scan->undistPointCloud_->points[ind]);
						} else if (largestPickedNum <= 20) {
							scan->cloudLabel_[ind] = 1;
							scan->cornerPointsLessSharp_->push_back(scan->undistPointCloud_->points[ind]);
						} else {
							break;
						}

						scan->cloudNeighborPicked_[ind] = 1;
						for (int l = 1; l <= 5; l++) {
							int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[ind + l] - segInfo->segmentedCloudColInd[ind + l - 1]));
							if (columnDiff > 10)
								break;
							scan->cloudNeighborPicked_[ind + l] = 1;
						}
						for (int l = -1; l >= -5; l--) {
							int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[ind + l] - segInfo->segmentedCloudColInd[ind + l + 1]));
							if (columnDiff > 10)
								break;
							scan->cloudNeighborPicked_[ind + l] = 1;
						}
					}
			}

		int smallestPickedNum = 0;
		for (int k = sp; k <= ep; k++) {
		    int ind = scan->cloudSmoothness_[k].ind;
		    if (scan->cloudNeighborPicked_[ind] == 0 &&
			scan->cloudCurvature_[ind] < SURF_THRESHOLD &&
			segInfo->segmentedCloudGroundFlag[ind] == true) {
			scan->cloudLabel_[ind] = -1;
			scan->surfPointsFlat_->push_back(scan->undistPointCloud_->points[ind]);
			smallestPickedNum++;
			if (smallestPickedNum >= 4) {
			    break;
			}

			scan->cloudNeighborPicked_[ind] = 1;
			for (int l = 1; l <= 5; l++) {

			    int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[ind + l] - segInfo->segmentedCloudColInd[ind + l - 1]));
			    if (columnDiff > 10)
				break;

			    scan->cloudNeighborPicked_[ind + l] = 1;
			}
			for (int l = -1; l >= -5; l--) {

			    int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[ind + l] - segInfo->segmentedCloudColInd[ind + l + 1]));
			    if (columnDiff > 10)
				break;

			    scan->cloudNeighborPicked_[ind + l] = 1;
			}
		    }
		}

		for (int k = sp; k <= ep; k++) {
		    if (scan->cloudLabel_[k] <= 0) {
			surfPointsLessFlatScan->push_back(scan->undistPointCloud_->points[k]);
		    }
		}
	    }
	    surfPointsLessFlatScanDS->clear();
	    downSizeFilter_.setInputCloud(surfPointsLessFlatScan);
	    downSizeFilter_.filter(*surfPointsLessFlatScanDS);
	    *(scan->surfPointsLessFlat_) += *surfPointsLessFlatScanDS;
	}
    }; 
    
    void findCorrespondingSurfFeatures(ScanPtr lastScan, ScanPtr newScan, pcl::PointCloud< PointType >::Ptr keypoints, pcl::PointCloud< PointType >::Ptr jacobianCoff, int iterCount) {
	int surfPointsFlatNum = newScan->surfPointsFlat_->points.size();

	for (int i = 0; i < surfPointsFlatNum; i++) {

	    PointType pointSel;
	    PointType coeff, tripod1, tripod2, tripod3;
	    
	    transformToStart(&newScan->surfPointsFlat_->points[i], &pointSel);
	    
	    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast = lastScan->surfPointsLessFlat_;
	    
	    if (iterCount % ICP_FREQ == 0) {
		std::vector<int> pointSearchInd;
		std::vector<float> pointSearchSqDis;
		kdtreeSurf_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
		int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

		if (pointSearchSqDis[0] < NEAREST_FEATURE_SEARCH_SQ_DIST) {
		    closestPointInd = pointSearchInd[0];
		    int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

		    float pointSqDis, minPointSqDis2 = NEAREST_FEATURE_SEARCH_SQ_DIST, minPointSqDis3 = NEAREST_FEATURE_SEARCH_SQ_DIST;
		    
		    for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
			if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
			    break;
			}
			
			pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
					(laserCloudSurfLast->points[j].x - pointSel.x) + 
					(laserCloudSurfLast->points[j].y - pointSel.y) * 
					(laserCloudSurfLast->points[j].y - pointSel.y) + 
					(laserCloudSurfLast->points[j].z - pointSel.z) * 
					(laserCloudSurfLast->points[j].z - pointSel.z);
			if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
			    if (pointSqDis < minPointSqDis2) {
				minPointSqDis2 = pointSqDis;
				minPointInd2 = j;
			    }
			} else {
			    if (pointSqDis < minPointSqDis3) {
				minPointSqDis3 = pointSqDis;
				minPointInd3 = j;
			    }
			}
		    }
		    
		    for (int j = closestPointInd - 1; j >= 0; j--) {
			if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
			    break;
			}

			pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
					(laserCloudSurfLast->points[j].x - pointSel.x) + 
					(laserCloudSurfLast->points[j].y - pointSel.y) * 
					(laserCloudSurfLast->points[j].y - pointSel.y) + 
					(laserCloudSurfLast->points[j].z - pointSel.z) * 
					(laserCloudSurfLast->points[j].z - pointSel.z);

			if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
			    if (pointSqDis < minPointSqDis2) {
				minPointSqDis2 = pointSqDis;
				minPointInd2 = j;
			    }
			} else {
			    if (pointSqDis < minPointSqDis3) {
				minPointSqDis3 = pointSqDis;
				minPointInd3 = j;
			    }
			}
		    }
		}
		pointSearchSurfInd1[i] = closestPointInd;
		pointSearchSurfInd2[i] = minPointInd2;
		pointSearchSurfInd3[i] = minPointInd3;
	    }

	    if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
		
		tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
		tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
		tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];


		V3D P0xyz(pointSel.x, pointSel.y, pointSel.z);
		V3D P1xyz(tripod1.x, tripod1.y, tripod1.z);
		V3D P2xyz(tripod2.x, tripod2.y, tripod2.z);
		V3D P3xyz(tripod3.x, tripod3.y, tripod3.z);
		
		V3D M = math_utils::skew(P1xyz-P2xyz) * (P1xyz-P3xyz);
		double r = (P0xyz - P1xyz).transpose() * M;
		double m = M.norm();
		float res = r / m;
		
		V3D jacxyz = M.transpose() / (m);
		
		float s = 1;
		if (iterCount >= ICP_FREQ) {
		    s = 1 - 1.8 * fabs(res) / sqrt(sqrt(pointSel.x * pointSel.x
			    + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
		}

		if (s > 0.1 && res != 0) {
                    coeff.x = s * jacxyz(0); 
                    coeff.y = s * jacxyz(1);
                    coeff.z = s * jacxyz(2);
                    coeff.intensity = s * res;	
		    
		    keypoints->push_back(newScan->surfPointsFlat_->points[i]);
		    jacobianCoff->push_back(coeff);
		}
	    }
	}
    }

    
	void findCorrespondingCornerFeatures(ScanPtr lastScan, ScanPtr newScan, pcl::PointCloud< PointType >::Ptr keypoints, pcl::PointCloud< PointType >::Ptr jacobianCoff, int iterCount) {
		int cornerPointsSharpNum = newScan->cornerPointsSharp_->points.size();
	
	for (int i = 0; i < cornerPointsSharpNum; i++) {
	    PointType pointSel;
	    PointType coeff, tripod1, tripod2;
	    
	    transformToStart(&newScan->cornerPointsSharp_->points[i], &pointSel);
	    
	    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast = lastScan->cornerPointsLessSharp_;
	    
	    if (iterCount % ICP_FREQ == 0) {
		std::vector<int> pointSearchInd;
		std::vector<float> pointSearchSqDis;
		kdtreeCorner_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
		int closestPointInd = -1, minPointInd2 = -1;
		
		if (pointSearchSqDis[0] < NEAREST_FEATURE_SEARCH_SQ_DIST) {
		    closestPointInd = pointSearchInd[0];
		    int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

		    float pointSqDis, minPointSqDis2 = NEAREST_FEATURE_SEARCH_SQ_DIST;
		    for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
			if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
			    break;
			}

			pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
					(laserCloudCornerLast->points[j].x - pointSel.x) + 
					(laserCloudCornerLast->points[j].y - pointSel.y) * 
					(laserCloudCornerLast->points[j].y - pointSel.y) + 
					(laserCloudCornerLast->points[j].z - pointSel.z) * 
					(laserCloudCornerLast->points[j].z - pointSel.z);

			if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
			    if (pointSqDis < minPointSqDis2) {
				minPointSqDis2 = pointSqDis;
				minPointInd2 = j;
			    }
			}
		    }
		    for (int j = closestPointInd - 1; j >= 0; j--) {
			if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
			    break;
			}

			pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
					(laserCloudCornerLast->points[j].x - pointSel.x) + 
					(laserCloudCornerLast->points[j].y - pointSel.y) * 
					(laserCloudCornerLast->points[j].y - pointSel.y) + 
					(laserCloudCornerLast->points[j].z - pointSel.z) * 
					(laserCloudCornerLast->points[j].z - pointSel.z);

			if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
			    if (pointSqDis < minPointSqDis2) {
				minPointSqDis2 = pointSqDis;
				minPointInd2 = j;
			    }
			}
		    }
		}

		pointSearchCornerInd1[i] = closestPointInd;
		pointSearchCornerInd2[i] = minPointInd2;
	    }

	    if (pointSearchCornerInd2[i] >= 0) {

		tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
		tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];
 
		V3D P0xyz(pointSel.x, pointSel.y, pointSel.z);
		V3D P1xyz(tripod1.x, tripod1.y, tripod1.z);
		V3D P2xyz(tripod2.x, tripod2.y, tripod2.z);
		
		V3D P =  math_utils::skew(P0xyz-P1xyz)*(P0xyz-P2xyz);
		float r = P.norm();
		float d12 = (P1xyz - P2xyz).norm();
		float res = r / d12;
		
		V3D jacxyz = P.transpose() * math_utils::skew(P2xyz - P1xyz) / (d12 * r);
		
		float s = 1;
		if (iterCount >= ICP_FREQ) {
		    s = 1 - 1.8 * fabs(res);
		}

		if (s > 0.1 && res != 0) {
                    coeff.x = s * jacxyz(0); 
                    coeff.y = s * jacxyz(1);
                    coeff.z = s * jacxyz(2);
		    coeff.intensity = s * res;
		    
		    keypoints->push_back(newScan->cornerPointsSharp_->points[i]);
		    jacobianCoff->push_back(coeff);
		}	
	    }
	}
    }
    
	void transformToStart(PointType const * const pi, PointType * const po) {
		double s = (1.f / SCAN_PERIOD) * (pi->intensity - int(pi->intensity));
		
		V3D P2xyz(pi->x, pi->y, pi->z);
		V3D phi = Quat2axis(linState_.qbn_);
		Q4D R21xyz = axis2Quat(s * phi); 
		R21xyz.normalized();
		V3D T112xyz = s * linState_.rn_;
		V3D P1xyz = R21xyz * P2xyz + T112xyz;
	
		po->x = P1xyz.x();
		po->y = P1xyz.y();
		po->z = P1xyz.z();
		po->intensity = pi->intensity;
	};
    
    
	void transformToEnd(PointType const * const pi, PointType * const po) {
		double s = (1.f / SCAN_PERIOD) * (pi->intensity - int(pi->intensity));

		V3D P2xyz(pi->x, pi->y, pi->z);
		V3D phi = Quat2axis(linState_.qbn_);
		Q4D R21xyz = axis2Quat(s * phi); 
		R21xyz.normalized();
		V3D T112xyz = s * linState_.rn_;
		V3D P1xyz = R21xyz * P2xyz + T112xyz;

		R21xyz = linState_.qbn_;
		T112xyz = linState_.rn_;
		P2xyz = R21xyz.inverse() * (P1xyz - T112xyz);

		po->x = P2xyz.x();
		po->y = P2xyz.y();
		po->z = P2xyz.z();
		po->intensity = pi->intensity;

	};
    

	void rotatePoint(PointType const * const pi, PointType * const po) {
		V3D rpy;
		rpy << deg2rad(0.0), deg2rad(0.0), deg2rad(IMU_LIDAR_EXTRINSIC_ANGLE);
		M3D R = rpy2R(rpy);
		V3D Pi(pi->x, pi->y, pi->z);
		V3D Po = R * Pi;
		po->x = Po.x();
		po->y = Po.y();
		po->z = Po.z();
		po->intensity = pi->intensity;
	}	
    

    
	void updatePointCloud() {
		scan_new_->cornerPointsLessSharpYZX_->clear();
		scan_new_->surfPointsLessFlatYZX_->clear();
		scan_new_->outlierPointCloudYZX_->clear();
	
		PointType point;
		for (int i = 0; i < scan_new_->cornerPointsLessSharp_->points.size(); i++) {
			transformToEnd(&scan_new_->cornerPointsLessSharp_->points[i], &scan_new_->cornerPointsLessSharp_->points[i]);
						point.x = scan_new_->cornerPointsLessSharp_->points[i].y;
						point.y = scan_new_->cornerPointsLessSharp_->points[i].z;
						point.z = scan_new_->cornerPointsLessSharp_->points[i].x; 
						point.intensity = scan_new_->cornerPointsLessSharp_->points[i].intensity;
			scan_new_->cornerPointsLessSharpYZX_->push_back(point);
		}
		for (int i = 0; i < scan_new_->surfPointsLessFlat_->points.size(); i++) {
			transformToEnd(&scan_new_->surfPointsLessFlat_->points[i], &scan_new_->surfPointsLessFlat_->points[i]);
						point.x = scan_new_->surfPointsLessFlat_->points[i].y;
						point.y = scan_new_->surfPointsLessFlat_->points[i].z;
						point.z = scan_new_->surfPointsLessFlat_->points[i].x; 
						point.intensity = scan_new_->surfPointsLessFlat_->points[i].intensity;   
			scan_new_->surfPointsLessFlatYZX_->push_back(point);
		}
		for (int i = 0; i < scan_new_->outlierPointCloud_->points.size(); i++) {
			//transformToEnd(&scan_new_->outlierPointCloud_->points[i], &scan_new_->outlierPointCloud_->points[i]);
						point.x = scan_new_->outlierPointCloud_->points[i].y;
						point.y = scan_new_->outlierPointCloud_->points[i].z;
						point.z = scan_new_->outlierPointCloud_->points[i].x; 
						point.intensity = scan_new_->outlierPointCloud_->points[i].intensity;   
			scan_new_->outlierPointCloudYZX_->push_back(point);
		}
	
		globalStateYZX_.rn_ = Q_xyz_to_yzx * globalState_.rn_;
		globalStateYZX_.qbn_ = Q_xyz_to_yzx * globalState_.qbn_ * Q_xyz_to_yzx.inverse();
	
	
		if (scan_new_->cornerPointsLessSharp_->points.size() >= 5 && scan_new_->surfPointsLessFlat_->points.size() >= 20) {
			kdtreeCorner_->setInputCloud(scan_new_->cornerPointsLessSharp_);
			kdtreeSurf_->setInputCloud(scan_new_->surfPointsLessFlat_);
		}
	}
    
	void estimateTransform(ScanPtr lastScan, ScanPtr newScan, V3D& t, Q4D& q) {
		double sum_dt = preintegration_->sum_dt;
		linState_.rn_ = t;
		linState_.qbn_ = q;
		for (int iter = 0; iter < NUM_ITER; iter++) {
	    keypointSurfs_->clear();
	    jacobianCoffSurfs->clear();
	    keypointCorns_->clear();
	    jacobianCoffCorns->clear();  
	    
	    findCorrespondingSurfFeatures(lastScan, newScan, keypointSurfs_, jacobianCoffSurfs, iter);
	    if (keypointSurfs_->points.size() < 10) {
				cout << "Insufficient matched surfs..." << endl;
				continue;
	    }    
	    findCorrespondingCornerFeatures(lastScan, newScan, keypointCorns_, jacobianCoffCorns, iter);
	    if (keypointCorns_->points.size() < 5) {
				cout << "Insufficient matched corners..." << endl;
				continue;
	    }
  
	    if (calculateTransformation(lastScan, newScan, keypointCorns_, jacobianCoffCorns, keypointSurfs_, jacobianCoffSurfs, iter)) {
				cout << iter << " iteration. System Converges." << endl;
				break;
	    }
		}
	
		t = linState_.rn_;
		q = linState_.qbn_;
	}     
        
    bool calculateTransformation(ScanPtr lastScan, ScanPtr newScan, pcl::PointCloud< PointType >::Ptr corners, pcl::PointCloud< PointType >::Ptr jacoCornersCoff, pcl::PointCloud< PointType >::Ptr surfs, pcl::PointCloud< PointType >::Ptr jacoSurfsCoff, int iterCount) {
	keypoints_->clear();
	jacobians_->clear();
	(*keypoints_) += (*surfs);
	(*keypoints_) += (*corners);
	(*jacobians_) += (*jacoSurfsCoff); 
	(*jacobians_) += (*jacoCornersCoff); 
	
	const int stateNum = 6;
	const int pointNum = keypoints_->points.size();
	const int imuNum = 0;
	const int row = pointNum + imuNum;
	Eigen::Matrix<double, Eigen::Dynamic, stateNum> J(row, stateNum);
	Eigen::Matrix<double, stateNum, Eigen::Dynamic> JT(stateNum, row);
	Eigen::Matrix<double, stateNum, stateNum> JTJ;
	Eigen::VectorXd b(row);
	Eigen::Matrix<double, stateNum, 1> JTb;
	Eigen::Matrix<double, stateNum, 1> x;
	J.setZero(); JT.setZero(); JTJ.setZero();
	b.setZero(); JTb.setZero();
	x.setZero();
	
	for (int i=0; i<pointNum; ++i) {
	    const PointType& keypoint = keypoints_->points[i];
	    const PointType& coeff = jacobians_->points[i];

	    V3D P2xyz(keypoint.x, keypoint.y, keypoint.z);
	    V3D coff_xyz(coeff.x, coeff.y, coeff.z);
	    
	    double s = (1.f / SCAN_PERIOD) * (keypoint.intensity - int(keypoint.intensity));
	    
	    V3D phi = Quat2axis(linState_.qbn_);
	    Q4D R21xyz = axis2Quat(s * phi); 
	    R21xyz.normalized();
	    V3D T112xyz = s * linState_.rn_;
	
	    V3D jacobian1xyz = coff_xyz.transpose() * (-R21xyz.toRotationMatrix() * skew(P2xyz)); // rot
	    V3D jacobian2xyz = coff_xyz.transpose() * M3D::Identity(); // trans
	    double residual = coeff.intensity;
	    
	    J.block<1, 3>(i, O_R) = jacobian1xyz;
	    J.block<1, 3>(i, O_P) = jacobian2xyz;

	    b(i) = -0.05 * residual;
	}
	
	JT = J.transpose();
	JTJ = JT * J;
	JTb = JT * b;
	x = JTJ.colPivHouseholderQr().solve(JTb);

	bool isDegenerate = false;
	Eigen::Matrix<double, stateNum, stateNum> matP;
	if (iterCount == 0) {
	    Eigen::Matrix<double, 1, stateNum> matE;
	    Eigen::Matrix<double, stateNum, stateNum> matV;
	    Eigen::Matrix<double, stateNum, stateNum> matV2;

	    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<double, stateNum, stateNum> > esolver(JTJ);
	    matE = esolver.eigenvalues().real();
	    matV = esolver.eigenvectors().real();

	    matV2 = matV;

	    isDegenerate = false;
	    std::vector<double> eignThre(stateNum, 10.);
	    for (int i = 0; i < stateNum; i++)
	    {
		// if eigenvalue is less than 10, set the corresponding eigenvector to 0 vector
		if (matE(0, i) < eignThre[i]) {
		    for (int j = 0; j < stateNum; j++) {
			matV2(i, j) = 0;
		    }
		    isDegenerate = true;
		} else {
		    break;
		}
	    }
	    matP = matV.inverse() * matV2;
	}
	
	if (isDegenerate) {
	    cout << "System is Degenerate." << endl;
	    Eigen::Matrix<double, stateNum, 1> matX2(x);
	    x = matP * matX2;
	}
	
	// Update
	Q4D dq = rpy2Quat(x.segment<3>(O_R));
	linState_.qbn_ =  (linState_.qbn_ * dq).normalized();
	linState_.rn_ += x.segment<3>(O_P);
	
	// Stop flag
        V3D rpy_rad = x.segment<3>(O_R);
	V3D rpy_deg = math_utils::rad2deg(rpy_rad);
	double deltaR = rpy_deg.norm();
	
	V3D trans = 100 * x.segment<3>(O_P);
	double deltaT = trans.norm();

        if (deltaR < 0.1 && deltaT < 0.1) {
            return true;
        }

	return false;
    } 
        
	void align(const V3D& p, const Q4D& q, V3D& v0, V3D& v1, V3D& ba, V3D& bw) {
		ba = INIT_BA;
		bw = INIT_BW;
		
		solveGyroscopeBias(q, bw);
		
		double sum_dt = preintegration_->sum_dt;
		v0 = (p - 0.5*linState_.gn_*sum_dt*sum_dt - preintegration_->delta_p)/sum_dt;
		v1 = v0 + sum_dt*linState_.gn_ + preintegration_->delta_v;

		cout << "v0: " << v0.transpose() << endl;
		cout << "v1: " << v1.transpose() << endl;
		cout << "ba0: " << INIT_BA.transpose() << endl;
		cout << "bw0: " << INIT_BW.transpose() << endl;
		cout << "bw0: " << bw.transpose() << endl;
		
	}
    
    
    void align2(const V3D& p, const Q4D& q, V3D& v0, V3D& v1, V3D& ba, V3D& bw) {

	const int DIM_OF_STATE = 1+1+3;
	const int DIM_OF_MEAS = 3+3;
	Eigen::Matrix<double, Eigen::Dynamic, DIM_OF_STATE> J(DIM_OF_MEAS, DIM_OF_STATE);
	Eigen::Matrix<double, DIM_OF_STATE, Eigen::Dynamic> JT(DIM_OF_STATE, DIM_OF_MEAS);
	Eigen::Matrix<double, DIM_OF_STATE, DIM_OF_STATE> JTJ;
	Eigen::VectorXd b(DIM_OF_MEAS);
	Eigen::Matrix<double, DIM_OF_STATE, 1> JTb;
	Eigen::Matrix<double, DIM_OF_STATE, 1> x;
	
	J.setZero(); JT.setZero(); JTJ.setZero();
	b.setZero(); JTb.setZero();
	x.setZero();
	
	double sum_dt = preintegration_->sum_dt;
	b.block<3, 1>(0,0) = (p-preintegration_->delta_p)/sum_dt - 0.5*sum_dt*linState_.gn_;
	b.block<3, 1>(3,0) = sum_dt*linState_.gn_ + preintegration_->delta_v;
	
	V3D L(1,0,0);
	J.block<3, 1>(0,0) = L;
	J.block<3, 3>(0,2) = -0.5*sum_dt*M3D::Identity();
	J.block<3, 1>(3,0) = -L;
	J.block<3, 1>(3,1) = L;
	J.block<3, 3>(3,2) = sum_dt*M3D::Identity();
	
	JT = J.transpose();
	JTJ = JT * J;
	JTb = JT * b;

	
	x = JTJ.colPivHouseholderQr().solve(JTb);
	v0 = x(0) * L;
	v1 = x(1) * L;
	ba = INIT_BA;
	bw = INIT_BW;	
	
	V3D test_ba = linState_.gn_ + preintegration_->delta_v/sum_dt;
	cout << "test_ba: " << test_ba.transpose() << endl;
    }
    
    void align3(const V3D& p, const Q4D& q, V3D& v0, V3D& v1, V3D& ba, V3D& bw) {
	double sum_dt = preintegration_->sum_dt;
	V3D v = p / sum_dt;
	// v * sum_dt = (p - 0.5*linState_.gn_*sum_dt*sum_dt - preintegration_->delta_p + 0.5*ba*sum_dt*sum_dt);
	ba = (v * sum_dt - p + 0.5*linState_.gn_*sum_dt*sum_dt + preintegration_->delta_p)*2*(1.0/sum_dt*sum_dt);
	
	solveGyroscopeBias(q, bw);
	
	v0 = v;
	v1 = v;	
	cout << "v0: " << v0.transpose() << endl;
	cout << "v1: " << v1.transpose() << endl;
	cout << "ba0: " << ba.transpose() << endl;
	cout << "bw0: " << bw.transpose() << endl;
	
    }
    
	void align4(const V3D& p, const Q4D& q, V3D& v0, V3D& v1, V3D& ba, V3D& bw) {
		double sum_dt = preintegration_->sum_dt;
		V3D v = p / sum_dt;
		v0 = v;
		v1 = v;	
		ba = INIT_BA;
		bw = INIT_BW;
	}
    
	void solveGyroscopeBias(const Q4D& q, V3D& bw) {
		Matrix3d A;
		Vector3d b;
		Vector3d delta_bg;
		A.setZero();
		b.setZero();

		MatrixXd tmp_A(3, 3);
		tmp_A.setZero();
		VectorXd tmp_b(3);
		tmp_b.setZero();
		Eigen::Quaterniond q_ij = q;
		tmp_A = preintegration_->jacobian.template block<3, 3>(GlobalState::att_, GlobalState::gyr_);
		tmp_b = 2 * (preintegration_->delta_q.inverse() * q_ij).vec();
		A += tmp_A.transpose() * tmp_A;
		b += tmp_A.transpose() * tmp_b;

		delta_bg = A.ldlt().solve(b);
		ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());

		bw += delta_bg;
	}
    
	FusionStatus status_;
	
	ScanPtr scan_new_;
	ScanPtr scan_last_;
	
	pcl::VoxelGrid<PointType> downSizeFilter_;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeCorner_;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurf_;
	
	std::vector<int> pointSelCornerInd;
	std::vector<double> pointSearchCornerInd1;
	std::vector<double> pointSearchCornerInd2;
	std::vector<int> pointSelSurfInd;
	std::vector<double> pointSearchSurfInd1;
	std::vector<double> pointSearchSurfInd2;
	std::vector<double> pointSearchSurfInd3;

	
	pcl::PointCloud< PointType >::Ptr keypoints_;
	pcl::PointCloud< PointType >::Ptr jacobians_;
		
	pcl::PointCloud<PointType>::Ptr keypointCorns_;
	pcl::PointCloud<PointType>::Ptr keypointSurfs_;
	pcl::PointCloud<PointType>::Ptr jacobianCoffCorns;
	pcl::PointCloud<PointType>::Ptr jacobianCoffSurfs;
	
	GlobalState globalState_;
	GlobalState linState_;
	Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, 1> difVecLinInv_;
	Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, 1> updateVec_;
	double updateVecNorm_ = 0.0;
	
	VXD residual_;
	MXD Fk_;
	MXD Gk_;
	MXD Pk_;
	MXD Qk_;
	MXD Rk_;
	MXD Hk_;
	MXD Jk_;
	MXD Kk_;
	MXD IKH_; 
	MXD Py_;
	MXD Pyinv_;
	
	Gps gpsRef_;
	Imu imu_last_;
	
	StatePredictor* filter_; 
	
	integration::IntegrationBase* preintegration_;
	
	Eigen::Matrix3d R_yzx_to_xyz;
	Eigen::Matrix3d R_xyz_to_yzx;
	Eigen::Quaterniond Q_yzx_to_xyz;
	Eigen::Quaterniond Q_xyz_to_yzx;
	GlobalState globalStateYZX_;
	
	V3D gravity_feedback;
};	
    


}

#endif