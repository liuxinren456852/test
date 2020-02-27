#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl {
struct PointXYZIWithRange {	
    PCL_ADD_POINT4D;                  
    double intensity;
    double range;
    int label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;   
}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZIWithRange,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, intensity, intensity)
    (double, range, range)
    (int, label, label)
)