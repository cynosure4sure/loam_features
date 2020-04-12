#include "loam_features.hpp"

//main()
int main(int argc, char **argv)
{
    //ROS init
    ros::init(argc, argv, "loam_features");
    ROS_WARN("---------- LOAM FEATURE NODE STARTED ----------");

    //Node handle
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~"); //private namespace for parameters

    //LOAM Features
    LoamFeatures loam_features(nh, nhp);

    ros::spin();

    return 0;
}
