#ifndef SENSOR_UTILS_HPP_
#define SENSOR_UTILS_HPP_

#include <iostream>
#include <map>
#include <math_utils.h>
#include <parameters.h>

using namespace std;
using namespace math_utils;
using namespace parameter;

namespace sensor_utils {

   
//Sensor measurement class
class Measurement {
public:
    Measurement() {};
    virtual ~Measurement(){};      
};

class Imu : public Measurement {
public:
    Imu() {};
    Imu(double time, const V3D& acc, const V3D& gyr):
	time(time),
	acc(acc),
	gyr(gyr){};
    ~Imu(){};
    double time;
    V3D acc;         // accelerometer measurement (m^2/sec)
    V3D gyr;         // gyroscope measurement (rad/s)
};

class Gps : public Measurement {
public:
    Gps() {
	time = 0.0;
	lat = 0.0;
	lon = 0.0;
	alt = 0.0;
    };
    Gps(double time, double lat, double lon, double alt) :
	time(time),
	lat(lat),
	lon(lon),
	alt(alt){};
    ~Gps(){};
    int status;
    double time;
    double lat; 
    double lon;	
    double alt; 
    inline V3D pn() const { return V3D(lat, lon, alt); }
};

class Odometry : public Measurement {
public:
    Odometry() {
	time = 0.0;
	rotation.setIdentity();
	translation.setZero();
    };
    Odometry(double time, const Q4D& rotation, const V3D& translation):
	time(time),
	rotation(rotation),
	translation(translation) {};
    ~Odometry(){};
    
    Odometry inverse(const Odometry& odom) {
	Odometry inv;
	inv.time = odom.time;
	inv.rotation = odom.rotation.inverse();
	inv.translation = -inv.rotation.toRotationMatrix() * odom.translation;
	return inv;
    }
    
    void boxPlus(const Odometry& increment) {
	translation = rotation.toRotationMatrix() * increment.translation + translation;
	rotation = rotation * increment.rotation;
    }
    
    Odometry boxMinus(const Odometry& odom) {
	Odometry res;
// 	Odometry inv = inverse(odom);
// 	res.translation = inv.rotation.toRotationMatrix() * translation + inv.translation;
// 	res.rotation = inv.rotation * rotation;
	
	res.translation = odom.rotation.inverse().toRotationMatrix() * translation - odom.rotation.inverse().toRotationMatrix()*odom.translation;
	res.rotation = odom.rotation.inverse() * rotation;
	return res;
    }
    
    double time;
    Q4D rotation;
    V3D translation;
    

};
 


class EarthParams{
public:  
    EarthParams() {}
    ~EarthParams() {}
    
    static M3D getDrp(const V3D& pn) {
	M3D Drp = M3D::Zero();
	double latitude = pn(0); 
	double longitude = pn(1); 
	double height = pn(2);  
	double sq = 1 - EeEe*sin(latitude)*sin(latitude);
	double RNh = Re / sqrt(sq) + height; // (2.5)
	double RMh = RNh * (1 - EeEe) / sq  + height; // (2.4)
	Drp <<  0, 1.0/RMh, 0,
		1.0/(RNh*cos(latitude)), 0, 0,
		0, 0, 1;
	return Drp;
    };
    
    static M3D getDpr(const V3D& pn) {
	M3D Drp = getDrp(pn);
	return Drp.inverse();
	
    };
    
    /*
    void updateEarthParams(const V3D& pos, const V3D& vn) {
	double sinLat, cosLat, sinLon, cosLon, tanLat, sinLat2, sinLat4;
	double sq;
	
	lat = pos(0);
	lon = pos(1); 
	height = pos(2);
	
	sinLat = sin(lat); 
	cosLat = cos(lat); 
	sinLon = sin(lon); 
	cosLon = cos(lon); 
	
	tanLat = sinLat / cosLat;
	sinLat2 = sinLat * sinLat; 
	sinLat4 = sinLat2 * sinLat2;
	sq = 1 - EeEe*sinLat*sinLat; 
    
	RNh = Re / sqrt(sq) + height; // (2.5)
	RMh = RNh * (1 - EeEe) / sq  + height; // (2.4)
	
	wnie << 0.0, Wie*cosLat, Wie*sinLat;
	wnen << -vn(1)/RMh, vn(0)/RNh, vn(0)*tanLat/RNh;
	wnin = wnie + wnen;
	gn << 0.0, 0.0, -(g0*(1 + 5.27094e-3*sinLat2 + 2.32718e-5*sinLat4) - 3.086e-6*pos(2));
	
	M3D Cne_DCM;
	Cne_DCM << -sinLon, -cosLon*sinLat, cosLon*cosLat,
	        cosLon, -sinLon*sinLat, sinLon*cosLat,
	        0     ,  cosLat       , sinLat;
	Cne = Cne_DCM;
	
	// from rn(ENU) to pos(lat,lon,h)
	Mrp <<  0, 1.0/RMh, 0,
		1.0/(RNh*cos(lat)), 0, 0,
		0, 0, 1;
			 
		
	Mpr = Mrp.inverse();		 
    }*/
};




}


#endif


	