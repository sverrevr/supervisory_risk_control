

#include "mavros_msgs/DebugValue.h"
#include <ros/ros.h>

class SupervisoryRiskControl{
    ros::NodeHandle nh;
    ros::NodeHandle nhp{"~"};

    ros::Subscriber debug_value_subscriber = nh.subscribe("/mavros/debug_value/debug_float_array", 1, &SupervisoryRiskControl::debug_value_CB, this);

    void debug_value_CB(mavros_msgs::DebugValueConstPtr msg){
        ROS_INFO("%f", msg->data[0]);
    }
public:
    SupervisoryRiskControl(){
        ros::spin();
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "supervisory_risk_control");
    SupervisoryRiskControl node;

    return 0;
}