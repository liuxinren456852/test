#include <parameters.h>
#include <math_utils.h>
#include <point_type.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/vector_average.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>

using namespace std;
using namespace parameter;

namespace pcl {
    class ScanImage : public pcl::PointCloud<pcl::PointXYZIWithRange> {
    public:  
	// =====TYPEDEFS=====
	typedef pcl::PointCloud<pcl::PointXYZIWithRange> BaseClass;
	typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > VectorOfEigenVector3d;
	typedef boost::shared_ptr<ScanImage> Ptr;
	typedef boost::shared_ptr<const ScanImage> ConstPtr;
	using BaseClass::points;
	using BaseClass::width;
	using BaseClass::height;
	
	PCL_EXPORTS ScanImage() {}
	PCL_EXPORTS virtual ~ScanImage() {}
	
	PCL_EXPORTS void reset () {}
	PCL_EXPORTS double* getRangesArray () const {
	    int arraySize = width * height;
	    double* ranges = new double[arraySize];
	    for (int i=0; i<arraySize; ++i)
		ranges[i] = points[i].range;
	    return ranges;
	};
	template <typename PointCloudType>
	void createFromPointCloud (const PointCloudType& point_cloud,
                                  double angular_resolution_x, double angular_resolution_y,
                                  double max_angle_width, double max_angle_height,
                                  double noise_level, double min_range, int border_size) {
	    
	    setAngularResolution (angular_resolution_x, angular_resolution_y);
	    width  = static_cast<uint32_t> (pcl_lrint (floor (max_angle_width*angular_resolution_x_reciprocal_)));
	    height = static_cast<uint32_t> (pcl_lrint (floor (max_angle_height*angular_resolution_y_reciprocal_)));
	    
	    int full_width  = static_cast<int> (pcl_lrint (floor (pcl::deg2rad (360.0f)*angular_resolution_x_reciprocal_))),
		full_height = static_cast<int> (pcl_lrint (floor (pcl::deg2rad (180.0f)*angular_resolution_y_reciprocal_)));
	    image_offset_x_ = (full_width -static_cast<int> (width) )/2;
	    image_offset_y_ = (full_height-static_cast<int> (height))/2;
	    
	    unsigned int size = width*height;
	    points.clear ();
	    points.resize (size, unobserved_point);
	}
	
	template <typename PointCloudType>
	void createFromPointCloud (const PointCloudType& point_cloud,
				  uint32_t input_width, uint32_t input_height, 
                                  double max_hor_angle, double min_hor_angle,
                                  double max_ver_angle, double min_ver_angle,
                                  double noise_level, double min_range, int border_size) {
	    
	    width = input_width;
	    height = input_height;
	    size_ = width * height;
	    max_hor_angle_ = max_hor_angle;
	    min_hor_angle_ = min_hor_angle;
	    max_ver_angle_ = max_ver_angle;
	    min_ver_angle_ = min_ver_angle;  
	    ver_factor_ =  (height - 1) / (max_ver_angle - min_ver_angle);
	    hor_factor_ =  (width - 1) / (max_hor_angle - min_hor_angle);
	    image_ = cv::Mat(height, width, CV_32FC(1), cv::Scalar(0)); 
	    
	    points.clear ();
	    points.resize (size_, unobserved_point);
	    
	    int top=height, right=-1, bottom=-1, left=width;
	    doZBuffer (point_cloud, noise_level, min_range);
	    
	}
	
	template <typename PointCloudType>
	void createFromPointCloud (const PointCloudType& point_cloud, double angular_resolution=pcl::deg2rad (0.5f),
	    double max_angle_width=pcl::deg2rad (360.0f), double max_angle_height=pcl::deg2rad (180.0f),
	    const Eigen::Affine3f& sensor_pose = Eigen::Affine3f::Identity (),
	    double noise_level=0.0f, double min_range=0.0f, int border_size=0) {
	    
	    createFromPointCloud (point_cloud, angular_resolution, angular_resolution, max_angle_width, max_angle_height,
				    noise_level, min_range, border_size);
	};
	
	template <typename PointCloudType> 
	void doZBuffer (const PointCloudType& point_cloud, double noise_level, double min_range) {
	    typedef typename PointCloudType::PointType PointType2;
	    const typename pcl::PointCloud<PointType2>::VectorType &points2 = point_cloud.points;
	    cout << points2.size() << " points in total." << endl;
	    int addedPointCounter = 0;
	    
	    unsigned int size = width*height;
	    int* counters = new int[size];
	    ERASE_ARRAY (counters, size);
	    
	    double x_real, y_real, range_of_current_point;
	    int x, y;
	    for (typename pcl::PointCloud<PointType2>::VectorType::const_iterator it=points2.begin (); it!=points2.end (); ++it)
	    {
		PointType2 current_point = (*it);
		if (!isFinite (current_point))  // Check for NAN etc
		    continue;

		Eigen::Vector3d current_vector(current_point.x, current_point.y, current_point.z);
		double current_intensity = current_point.intensity;
		
		this->getImagePoint (current_vector, x_real, y_real, range_of_current_point);
		this->real2DToInt2D (x_real, y_real, x, y);
		
		//cout << range_of_current_point << endl;
		if (range_of_current_point < min_range|| !isInImage (x, y)) {
		    //cout << "Point is not in the image." << endl;
		    continue;
		}
		    
		
		int arrayPos = y*width + x;
		PointXYZIWithRange& point = points[arrayPos];
		double& range_at_image_point = image_.at<double>(y, x);
		int& counter = counters[arrayPos];
		bool addCurrentPoint=false, replace_with_current_point=false;
		
		if (counter==0) {
		    replace_with_current_point = true;
		} else {
		    if (range_of_current_point < range_at_image_point-noise_level) {
			replace_with_current_point = true;
		    }
		    else if (fabs (range_of_current_point-range_at_image_point)<=noise_level) {
			addCurrentPoint = true;
		    }
		}
    
		if (replace_with_current_point) {
		    counter = 1;
		    point.x = current_point.x;
		    point.y = current_point.y;
		    point.z = current_point.z;
		    point.intensity = current_intensity;
		    point.range = range_of_current_point;
		    point.label = 1;
		    
		    range_at_image_point = range_of_current_point;
		    counter++;
		    addedPointCounter++;
		   
		} else if (addCurrentPoint) {
		    ++counter;
		    range_at_image_point += (range_of_current_point-range_at_image_point)/counter;
		    std::cout << "===Overlapping point==="<<x<<","<<y<<"\n";
		}
	    }
	    cout << addedPointCounter << " points are added successfully." << endl;
	    delete[] counters;
	    
	}
	
	
	
	template <typename PointCloudType> 
	void doZBuffer (const PointCloudType& point_cloud, double noise_level, double min_range, int& top, int& right, int& bottom, int& left) {
	    
	    typedef typename PointCloudType::PointType PointType2;
	    const typename pcl::PointCloud<PointType2>::VectorType &points2 = point_cloud.points;
	    cout << points2.size() << " points in total." << endl;
	    int addedPointCounter = 0;
	    
	    unsigned int size = width*height;
	    int* counters = new int[size];
	    ERASE_ARRAY (counters, size);
	    
	    double x_real, y_real, range_of_current_point;
	    int x, y;
	    for (typename pcl::PointCloud<PointType2>::VectorType::const_iterator it=points2.begin (); it!=points2.end (); ++it)
	    {
		PointType2 current_point = (*it);
		if (!isFinite (current_point))  // Check for NAN etc
		    continue;

		Eigen::Vector3d current_vector(current_point.x, current_point.y, current_point.z);
		double current_intensity = current_point.intensity;
		
		this->getImagePoint (current_vector, x_real, y_real, range_of_current_point);
		this->real2DToInt2D (x_real, y_real, x, y);
		
		if (range_of_current_point < min_range|| !isInImage (x, y)) 
		    continue;
		
		  
// 		int floor_x = pcl_lrint (floor (x_real)), floor_y = pcl_lrint (floor (y_real)),
// 		    ceil_x  = pcl_lrint (ceil (x_real)),  ceil_y  = pcl_lrint (ceil (y_real));
// 		
// 		int neighbor_x[4], neighbor_y[4];
// 		neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
// 		neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
// 		neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
// 		neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;
// 		//std::cout << x_real<<","<<y_real<<": ";
// 		
// 		for (int i=0; i<4; ++i) {
// 		    int n_x=neighbor_x[i], n_y=neighbor_y[i];
// 		    //std::cout << n_x<<","<<n_y<<" ";
// 		    if (n_x==x && n_y==y)
// 			continue;
// 		    if (isInImage (n_x, n_y)) {
// 			int neighbor_array_pos = n_y*width + n_x;
// 			if (counters[neighbor_array_pos]==0) {
// 			    PointXYZIWithRange& neighbor_point = points[neighbor_array_pos];
// 			    double& neighbor_range = neighbor_point.range;
// 			    if (pcl_isinf (neighbor_range)) {
// 				neighbor_point.x = current_point.x;
// 				neighbor_point.y = current_point.y;
// 				neighbor_point.z = current_point.z;
// 				neighbor_point.intensity = current_intensity;
// 				neighbor_point.range = range_of_current_point;
// 				neighbor_point.label = 1;
// 				image_.at<double>(n_y, n_x) = range_of_current_point;
// 			    } else {
// 				
// 			    }
// 			    //neighbor_range = (pcl_isinf (neighbor_range) ? range_of_current_point : (std::min) (neighbor_range, range_of_current_point));
// 	
// 			    top= (std::min) (top, n_y); right= (std::max) (right, n_x); bottom= (std::max) (bottom, n_y); left= (std::min) (left, n_x);
// 			}
// 		    }
// 		}
		  
		
		int arrayPos = y*width + x;
		PointXYZIWithRange& point = points[arrayPos];
		double& range_at_image_point = image_.at<double>(y, x);
		int& counter = counters[arrayPos];
		bool addCurrentPoint=false, replace_with_current_point=false;
		
		if (counter==0) {
		    replace_with_current_point = true;
		} else {
		    if (range_of_current_point < range_at_image_point-noise_level) {
			replace_with_current_point = true;
		    }
		    else if (fabs (range_of_current_point-range_at_image_point)<=noise_level) {
			addCurrentPoint = true;
		    }
		}
    
		if (replace_with_current_point) {
		    counter = 1;
		    point.x = current_point.x;
		    point.y = current_point.y;
		    point.z = current_point.z;
		    point.intensity = current_intensity;
		    point.range = range_of_current_point;
		    point.label = 1;
		    
		    range_at_image_point = range_of_current_point;
		    counter++;
		    addedPointCounter++;
		   
		} else if (addCurrentPoint) {
		    ++counter;
		    range_at_image_point += (range_of_current_point-range_at_image_point)/counter;
		    std::cout << "===Overlapping point==="<<x<<","<<y<<"\n";
		}
	    }
	    cout << addedPointCounter << " points are added successfully." << endl;
	    delete[] counters;
	    
// 	    typedef typename PointCloudType::PointType PointType2;
// 	    const typename pcl::PointCloud<PointType2>::VectorType &points2 = point_cloud.points;
// 	    
// 	    unsigned int size = width*height;
// 	    int* counters = new int[size];
// 	    ERASE_ARRAY (counters, size);
// 	    
// 	    top=height; right=-1; bottom=-1; left=width;
// 	    
// 	    float x_real, y_real, range_of_current_point;
// 	    int x, y;
// 	    for (typename pcl::PointCloud<PointType2>::VectorType::const_iterator it=points2.begin (); it!=points2.end (); ++it)
// 	    {
// 		if (!isFinite (*it))  // Check for NAN etc
// 		continue;
// 		Vector3fMapConst current_point = it->getVector3fMap ();
// 		
// 		this->getImagePoint (current_point, x_real, y_real, range_of_current_point);
// 		this->real2DToInt2D (x_real, y_real, x, y);
// 		
// 		if (range_of_current_point < min_range|| !isInImage (x, y))
// 		continue;
// 		//std::cout << " ("<<current_point[0]<<", "<<current_point[1]<<", "<<current_point[2]<<") falls into pixel "<<x<<","<<y<<".\n";
// 		
// 		// Do some minor interpolation by checking the three closest neighbors to the point, that are not filled yet.
// 		int floor_x = pcl_lrint (floor (x_real)), floor_y = pcl_lrint (floor (y_real)),
// 		    ceil_x  = pcl_lrint (ceil (x_real)),  ceil_y  = pcl_lrint (ceil (y_real));
// 		
// 		int neighbor_x[4], neighbor_y[4];
// 		neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
// 		neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
// 		neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
// 		neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;
// 		//std::cout << x_real<<","<<y_real<<": ";
// 		
// 		for (int i=0; i<4; ++i)
// 		{
// 		int n_x=neighbor_x[i], n_y=neighbor_y[i];
// 		//std::cout << n_x<<","<<n_y<<" ";
// 		if (n_x==x && n_y==y)
// 		    continue;
// 		if (isInImage (n_x, n_y))
// 		{
// 		    int neighbor_array_pos = n_y*width + n_x;
// 		    if (counters[neighbor_array_pos]==0)
// 		    {
// 		    float& neighbor_range = points[neighbor_array_pos].range;
// 		    neighbor_range = (pcl_isinf (neighbor_range) ? range_of_current_point : (std::min) (neighbor_range, range_of_current_point));
// 		    top= (std::min) (top, n_y); right= (std::max) (right, n_x); bottom= (std::max) (bottom, n_y); left= (std::min) (left, n_x);
// 		    }
// 		}
// 		}
// 		//std::cout <<std::endl;
// 		
// 		// The point itself
// 		int arrayPos = y*width + x;
// 		float& range_at_image_point = points[arrayPos].range;
// 		int& counter = counters[arrayPos];
// 		bool addCurrentPoint=false, replace_with_current_point=false;
// 		
// 		if (counter==0)
// 		{
// 		replace_with_current_point = true;
// 		}
// 		else
// 		{
// 		if (range_of_current_point < range_at_image_point-noise_level)
// 		{
// 		    replace_with_current_point = true;
// 		}
// 		else if (fabs (range_of_current_point-range_at_image_point)<=noise_level)
// 		{
// 		    addCurrentPoint = true;
// 		}
// 		}
// 		
// 		if (replace_with_current_point)
// 		{
// 		counter = 1;
// 		range_at_image_point = range_of_current_point;
// 		top= (std::min) (top, y); right= (std::max) (right, x); bottom= (std::max) (bottom, y); left= (std::min) (left, x);
// 		//std::cout << "Adding point "<<x<<","<<y<<"\n";
// 		}
// 		else if (addCurrentPoint)
// 		{
// 		++counter;
// 		range_at_image_point += (range_of_current_point-range_at_image_point)/counter;
// 		}
// 	    }
// 	    
// 	    delete[] counters;
	}
	
	void getMinMaxRanges (double& min_range, double& max_range) const {
	    min_range = std::numeric_limits<double>::infinity ();
	    max_range = -std::numeric_limits<double>::infinity ();
	    for (unsigned int i=0; i<points.size (); ++i) {
		if (points[i].label == 0)
		    continue;
		double range = points[i].range;
		if (!pcl_isfinite (range))
		continue;
		min_range = (std::min) (min_range, range);
		max_range = (std::max) (max_range, range);
	    }
	};
	
	void getAverageRange (double& ave_range) {
	    double sum_range = 0;
	    int counter = 0;
	    for (unsigned int i=0; i<points.size (); ++i) {
		if (points[i].label == 0)
		    continue;
		double range = points[i].range;
		if (!pcl_isfinite (range))
		    continue;
		
		sum_range += range;
		counter++;
	    }    
	    ave_range = sum_range / counter;
	}
	unsigned char computeGreyScale(const float& range, const float& average, const float& ratio = 0.5) {
	    if (range < 0.0) {
		return 255;
	    } else if (range > average * 2) {
		return 255.0 / (1.0 + std::exp(ratio*(-range + average)));
	    } else {
		return range * 127.5 / average;
	    }	
	};  

	void visualize(cv::Mat& image)
	{
	    image = cv::Mat(image_.rows, image_.cols, CV_8UC1, cv::Scalar(255));
	    
	    double average_range = 0;
	    getAverageRange(average_range);
	    cout << "average_range: " << average_range << endl;
	    
	    int count = 0;
	    for (size_t r = 0; r < image_.rows; r++) {
		for (size_t c = 0; c < image_.cols; c++) {
		    if (isLabeled(c, r)) {
			float range = image_.at<float>(r, c);
			image.at<unsigned char>(r, c) = computeGreyScale(range, average_range);
			count++;
		    }
		}
	    }
	    
	    cout << "count: " << count << endl;
// 	    
// 	    cv::Mat filterdImage = cv::Mat(image_.rows, image_.cols, CV_8UC1, cv::Scalar(255));
// 	    cv::bilateralFilter(image, filterdImage, 3, 25.0f / average_range, 6.0f);
// 	    image = filterdImage;
// 	    
	}

	
	inline void real2DToInt2D (double x, double y, int& xInt, int& yInt) const {
	    xInt = static_cast<int> (pcl_lrintf (x)); 
	    yInt = static_cast<int> (pcl_lrintf (y));
	};
	inline Ptr makeShared () { return Ptr (new ScanImage (*this)); } 
	inline void setAngularResolution (double angular_resolution_x, double angular_resolution_y) {
	    angular_resolution_x_ = angular_resolution_x;
	    angular_resolution_x_reciprocal_ = 1.0f / angular_resolution_x_;
	    angular_resolution_y_ = angular_resolution_y;
	    angular_resolution_y_reciprocal_ = 1.0f / angular_resolution_y_;
	}
	inline void setImageOffsets (int offset_x, int offset_y) { image_offset_x_=offset_x; image_offset_y_=offset_y;}
	inline double getAngularResolutionX () const { return angular_resolution_x_;}
	inline double getAngularResolutionY () const { return angular_resolution_y_;}
	inline int getImageOffsetX () const { return image_offset_x_;}
	inline int getImageOffsetY () const { return image_offset_y_;}
	
	/** Return the 3d point with range at the given image position */
	inline const pcl::PointXYZIWithRange& getPoint (int image_x, int image_y) const {
	    if (!isInImage (image_x, image_y))
		return unobserved_point;
	    return points[image_y*width + image_x];
	}
	/** Return the 3d point with range at the given index (whereas index=y*width+x) */
	inline const pcl::PointXYZIWithRange& getPoint (int index) const { return points[index]; }
	inline const Eigen::Vector3d getEigenVector3d (int x, int y) const { return Eigen::Vector3d(getPoint (x, y).x, getPoint (x, y).y, getPoint (x, y).z); }
	inline const Eigen::Vector3d getEigenVector3d (int index) const { return Eigen::Vector3d(getPoint (index).x, getPoint (index).y, getPoint (index).z); }
	/** Calculate the 3D point according to the given image point and range */
	inline void calculate3DPoint (double image_x, double image_y, double range, Eigen::Vector3f& point) const {
	    double angle_x, angle_y;
	    //std::cout << image_x<<","<<image_y<<","<<range;
	    getAnglesFromImagePoint (image_x, image_y, angle_x, angle_y);
	    double cosY = cos (angle_y);
	    point = Eigen::Vector3f (range * sin (angle_x) * cosY, range * sin (angle_y), range * cos (angle_x)*cosY);
	}
	inline void calculate3DPoint (double image_x, double image_y, Eigen::Vector3f& point) const {
	    const pcl::PointXYZIWithRange& point_in_image = getPoint (image_x, image_y);
	    calculate3DPoint (image_x, image_y, point_in_image.range, point);
	}
	inline void calculate3DPoint (double image_x, double image_y, double range, pcl::PointXYZIWithRange& point) const {
	    point.range = range;
	    Eigen::Vector3f tmp_point;
	    calculate3DPoint (image_x, image_y, range, tmp_point);
	    point.x=tmp_point[0];  point.y=tmp_point[1];  point.z=tmp_point[2];
	}; 
	
	inline void getImagePointFromAngles (double angle_x, double angle_y, double& image_x, double& image_y) const {
	    // pcl form
	    // image_x = (angle_x*cos (angle_y) + static_cast<double> (M_PI))*angular_resolution_x_reciprocal_ - static_cast<double> (image_offset_x_);
	    // image_y = (angle_y + 0.5f*static_cast<double> (M_PI))*angular_resolution_y_reciprocal_ - static_cast<double> (image_offset_y_);
	    
	    image_x = (angle_x + static_cast<double> (M_PI) - min_hor_angle_) * hor_factor_;
	    image_y = (max_ver_angle_ - angle_y) * ver_factor_;
	}
	inline void getAnglesFromImagePoint (double image_x, double image_y, double& angle_x, double& angle_y) const {
	    angle_y = (image_y+static_cast<double> (image_offset_y_))*angular_resolution_y_ - 0.5f*static_cast<double> (M_PI);
	    double cos_angle_y = cos (angle_y);
	    angle_x = (cos_angle_y==0.0f ? 0.0f : ( (image_x+ static_cast<double> (image_offset_x_))*angular_resolution_x_ - static_cast<double> (M_PI))/cos_angle_y);   
	};
	inline void getImagePoint (const Eigen::Vector3d& point, double& image_x, double& image_y, double& range) const  {
	    range = point.norm ();
	    
	    // pcl form
	    // double angle_x = atan2 (point[0], point[2]),
	    //	    angle_y = sin (point[1]/range);

	    double angle_x = -atan2(point.y(), point.x()),
		  angle_y = atan(point.z() / sqrt(point.y()*point.y()+point.x()*point.x()));
		  
	    getImagePointFromAngles (angle_x, angle_y, image_x, image_y);
	}
	
	/** Calculates the curvature in a point using pca */
	inline float getCurvature (int x, int y, int radius, int step_size) const {
	    VectorAverage3f vector_average;
	    for (int y2=y-radius; y2<=y+radius; y2+=step_size) {
		for (int x2=x-radius; x2<=x+radius; x2+=step_size) {
		    if (!isInImage (x2, y2))
			continue;
		    const pcl::PointXYZIWithRange& point = getPoint (x2, y2);
		    if (!pcl_isfinite (point.range))
			continue;
		    vector_average.add (Eigen::Vector3f (point.x, point.y, point.z));
		}
	    }
	    if (vector_average.getNoOfSamples () < 3)
		return false;
	    Eigen::Vector3f eigen_values;
	    vector_average.doPCA (eigen_values);
	    return eigen_values[0]/eigen_values.sum ();
	};

	
	/** Check if a point is inside of the image */
	inline bool isInImage (int x, int y) const { return (x >= 0 && x < static_cast<int> (width) && y >= 0 && y < static_cast<int> (height)); }
	inline bool isValid (int x, int y) const { return isInImage (x,y) && pcl_isfinite (getPoint (x,y).range); }
	inline bool isValid (int index) const { return pcl_isfinite (getPoint (index).range); }
	inline bool isLabeled (int x, int y) const { return isInImage (x,y) && getPoint (x,y).label != 0; }
	inline bool isLabeled (int index) const { return getPoint (index).label != 0; }	
	inline bool isMaxRange (int x, int y) const {
	    double range = getPoint (x,y).range;
	    return pcl_isinf (range) && range>0.0f;
	};
    protected:
	// =====MEMBER VARIABLES=====
	// BaseClass:
	//   roslib::Header header;
	//   std::vector<PointT> points;
	//   uint32_t width;
	//   uint32_t height;
	//   bool is_dense;
	cv::Mat image_;
	double max_hor_angle_;
	double min_hor_angle_;
	double max_ver_angle_;
	double min_ver_angle_;  
	double ver_factor_;
	double hor_factor_;
	uint32_t size_;
	
	double angular_resolution_x_;            
	double angular_resolution_y_;             
	double angular_resolution_x_reciprocal_;                 
	double angular_resolution_y_reciprocal_; 
	int image_offset_x_, image_offset_y_;
	pcl::PointXYZIWithRange unobserved_point;        

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

class FeatureHandler{
public:
    FeatureHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh):
        nh_(nh),
	pnh_(pnh) {

        subPointCloud_ = pnh_.subscribe<sensor_msgs::PointCloud2>(LIDAR_TOPIC, 1, &FeatureHandler::lidarCallback, this);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    
    ~FeatureHandler(){}
    
    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
    }

    void resetParameters(){
        laserCloudIn->clear();
    }

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
    }
    
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
	TicToc ts_total;
        copyPointCloud(laserCloudMsg);
	//TODO Range imgae
	process();
        //publishCloud();
        resetParameters();
	double time_total = ts_total.toc();
	if (VERBOSE) {
	    cout << "Segmentation: time: " << time_total << endl;
	}
    }
    
    void process() {
	double noise_level = 0.0;      
	double min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::ScanImage> range_image_ptr (new pcl::ScanImage);
	pcl::ScanImage& range_image = *range_image_ptr;  
	
	double max_ver_angle = std::numeric_limits<double>::infinity ();
	double min_ver_angle = -std::numeric_limits<double>::infinity ();
	V3D point;
	int hor_point_counter = 0;
	for (size_t i=0; i<laserCloudIn->size(); ++i) { 
	    point << laserCloudIn->points[i].x,laserCloudIn->points[i].y,laserCloudIn->points[i].z;
	    
	    if (!pcl_isfinite(point.x()) ||
		!pcl_isfinite(point.y()) ||
		!pcl_isfinite(point.z())) {
		continue;
	    }
	
	    
	    double hor_angle = -atan2(point.y(), point.x());
	    double ver_angle = atan(point.z() / sqrt(point.y() * point.y() + point.x() * point.x()));
	    double factor((16 - 1) / (15.0f - (-15.0f)));
	    int scanID = int(((hor_angle * 180 / M_PI) - (-15.0f)) * factor + 0.5);
	    
	    if (scanID == 10) {
		hor_point_counter++;
	    }
	    
	    max_ver_angle = (std::min) (max_ver_angle, ver_angle);
	    min_ver_angle = (std::max) (min_ver_angle, ver_angle);
	}
	
// 	max_ver_angle = pcl::deg2rad(15.0f);
// 	min_ver_angle = pcl::deg2rad(-15.0f);
// 	cout << "hor_point_counter: " << hor_point_counter << endl;
// 	cout << "max_ver_angle : " << math_utils::rad2deg(max_ver_angle);
// 	cout << ", min_ver_angle: " << math_utils::rad2deg(min_ver_angle) << endl;
	
	range_image.createFromPointCloud (*laserCloudIn, 800, 16, 
					pcl::deg2rad (360.0f), pcl::deg2rad (0.0f),
					max_ver_angle, min_ver_angle,
					noise_level, min_range, border_size);
	cv::Mat grey_image;
	range_image.visualize(grey_image);
	cv::namedWindow("grey_image", CV_WINDOW_NORMAL); 
	cv::imshow("grey_image",grey_image);
	cv::waitKey(5);
	
    }
    
protected:
    ros::NodeHandle nh_;			 
    ros::NodeHandle pnh_;

    ros::Subscriber subPointCloud_;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;

    PointType nanPoint;
    std_msgs::Header cloudHeader;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "range_image_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    parameter::readParameters(pnh);
	
    FeatureHandler featureHandler(nh, pnh);
    
    ROS_INFO("\033[1;32m---->\033[0m Range Image Module Started.");

    ros::spin();
    return 0;
}
