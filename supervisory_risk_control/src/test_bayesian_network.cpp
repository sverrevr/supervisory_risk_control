
#include <supervisory_risk_control/bayesian_network.h>
#include <ros/ros.h>
#include <string>
#include <ros/package.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "supervisory_risk_control");
    ros::NodeHandle nh;

    std::string file_path = ros::package::getPath("supervisory_risk_control") + "/include/supervisory_risk_control/test.xdsl";
    std::vector<std::string> temporal_leaf_nodes = {"temporal_leaf"};
    BayesianNetwork net{file_path,temporal_leaf_nodes,2};
    
    net.setEvidence("output","true");
    net.incrementTime();

    auto res1 = net.evaluateStates({"output"});

    net.setEvidence("output","false");
    net.incrementTime();

    auto res2 = net.evaluateStates({"output"});

    net.setEvidence("output","true");
    net.incrementTime();

    auto res3 = net.evaluateStates({"output"});

    net.save_network(ros::package::getPath("supervisory_risk_control") + "/include/supervisory_risk_control/test-updated.xdsl");

    return 0;
}