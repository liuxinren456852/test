#include <ros/ros.h>

#include <parameters.h>
#include <DriftEvaluator.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drift_evaluation_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("\033[1;32m---->\033[0m Drift Evaluation Started.");
    parameter::readParameters(pnh);
    evaluation::Evaluator evaluator(nh, pnh);
    evaluator.run();
    ros::Rate rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
	evaluator.process();
        rate.sleep();
    }
    
    ros::spin();
    return 0;
}
