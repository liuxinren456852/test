#include <ros/ros.h>

#include <parameters.h>
#include <Estimator.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "liom_fusion_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("\033[1;32m---->\033[0m Feature IMU Association Started.");

    parameter::readParameters(pnh);
    
    
    fusion::Liom liom(nh, pnh);
    liom.run();
    /*
    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
	liom.process();

        rate.sleep();
    }*/
    
    ros::spin();
    return 0;
}
