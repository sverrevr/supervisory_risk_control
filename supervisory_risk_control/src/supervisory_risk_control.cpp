#include "mavros_msgs/DebugValue.h"
#include "supervisory_risk_control/bayesian_network.h"
#include "supervisory_risk_control/classifiers.hpp"
#include "supervisory_risk_control/debug_array_id.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <cmath>
#include <sstream>

class SupervisoryRiskControl
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp{"~"};

    //const std::vector<std::string> temporal_leaf_nodes = {"obstacle_missing_from_lidar_scan","turbulence_geometry_følsomhet", "other_disturbances", "velocity_deviation_distance_scaling_factor", "distance_deviation_when_loss_of_control", "motorbruk_fra_slitasje_eller_tether", "velocity_deviation_when_loss_of_control", "other_reason_for_roll_pitchrate_devitaiton"};
    const std::vector<std::string> temporal_leaf_nodes = {"varying_motor_use", "mean_motor_use"};
    BayesianNetwork net{ros::package::getPath("supervisory_risk_control") + "/include/supervisory_risk_control/2feb_simplified Discretized.xdsl",
                        temporal_leaf_nodes, 2};

    ros::Subscriber debug_value_subscriber = nh.subscribe("/mavros/debug_value/debug_float_array", 1, &SupervisoryRiskControl::debug_value_CB, this);

    void debug_value_CB(mavros_msgs::DebugValueConstPtr msg)
    {
        ROS_INFO("\n\n\nNew measurement");
        // put all values as evidence on measurement nodes
        net.setEvidence("motor_use", motor_use_clasifier(msg->data[topic_indices::motor_max]));
        ROS_INFO("%s: %.2f, %i", "motor_use",msg->data[topic_indices::motor_max], motor_use_clasifier(msg->data[topic_indices::motor_max]));

        net.setEvidence("js_derivat", js_derivative_clasifier(msg->data[topic_indices::js_derivative]));
        ROS_INFO("%s: %.2f, %i", "js_derivat",msg->data[topic_indices::js_derivative], js_derivative_clasifier(msg->data[topic_indices::js_derivative]));

        /*net.setEvidence("loss_of_control_authority", loss_of_control_authority_classifier(msg->data[LOSS_OF_CONTROL_AUTHORITY]));
        ROS_INFO("%s: %.2f, %i", "loss_of_control_authority",msg->data[LOSS_OF_CONTROL_AUTHORITY], loss_of_control_authority_classifier(msg->data[LOSS_OF_CONTROL_AUTHORITY]));

        net.setEvidence("velocity_deviation", velocity_deviation_classifier(msg->data[VELOCITY_DEVIATION]));
        ROS_INFO("%s: %.2f, %i", "velocity_deviation",msg->data[VELOCITY_DEVIATION], velocity_deviation_classifier(msg->data[VELOCITY_DEVIATION]));

        net.setEvidence("collision_speed", collision_speed_classifier(msg->data[COLLISION_SPEED]));
        ROS_INFO("%s: %.2f, %i", "collision_speed",msg->data[COLLISION_SPEED], collision_speed_classifier(msg->data[COLLISION_SPEED]));

        net.setEvidence("roll_pitchrate_deviation", rp_rate_deviation_classifier(msg->data[RP_RATE_DEVIATION]));
        ROS_INFO("%s: %.2f, %i", "roll_pitchrate_deviation",msg->data[RP_RATE_DEVIATION], rp_rate_deviation_classifier(msg->data[RP_RATE_DEVIATION]));

        net.setEvidence("is_close_obstacle", is_close_to_obstacle_classifier(msg->data[IS_CLOSE_TO_OBSTACLE]));
        ROS_INFO("%s: %.2f, %i", "is_close_obstacle",msg->data[IS_CLOSE_TO_OBSTACLE], is_close_to_obstacle_classifier(msg->data[IS_CLOSE_TO_OBSTACLE]));

        net.setEvidence("turbulence_geometry", 2);
        ROS_INFO("%s: %.2f, %i", "turbulence_geometry", NAN, 2);

        net.setEvidence("current_safety_margin", 3);
        ROS_INFO("%s: %.2f, %i", "current_safety_margin", NAN, 3);*/

        // evalaute different actions
        const std::string output_node = "S3__Loss_of_control_authority_causes_contact";
        //const std::string output_node = "probability_that_we_will_contact";

        double lowest_cost = INFINITY;
        int best_margin = -1;
        int best_acceleration = -1;
        //double best_speed = -1;
        double best_output_probability = -1;

        for(int safety_margin = 0; safety_margin < 5; ++safety_margin){
            net.setEvidence("safety_margin", safety_margin);
            for(int max_acceleration = 0; max_acceleration < 5; ++max_acceleration){
                net.setEvidence("motorbruk_til_maks_akselerasjon", max_acceleration);
                double output_prob = net.evaluateStates({output_node}).at(output_node).at(output_node);
                //for(double speed = 0.2; speed < 1.5; speed+=0.3){
                    static constexpr double safety_margin_penalty = 1;
                    static constexpr double max_acceleration_penalty = 1;
                    //static constexpr double speed_penalty = 1;
                    //static constexpr double contact_speed_gain_penalty = 3;
                    static constexpr double contact_constant_penalty = 1;
                    const double safety_margin_cost = std::pow(safety_margin/4.0,2.0)*safety_margin_penalty;
                    const double max_acc_cost = std::pow(1-max_acceleration/4.0,2.0)*max_acceleration_penalty;
                    //const double speed_cost = std::pow(1-speed/1.4,2.0)*speed_penalty;
                    //const double risk_cost = contact_prob*(contact_constant_penalty+std::pow(speed,2.0)*contact_speed_gain_penalty);
                    const double cost = safety_margin_cost+max_acc_cost+contact_constant_penalty*output_prob;//speed_cost+risk_cost;
                    if(cost<lowest_cost){
                        lowest_cost = cost;
                        best_margin = safety_margin;
                        best_acceleration = max_acceleration;
                        //best_speed = speed;
                        best_output_probability = output_prob;
                    }
                //}
            }
        }

        //ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %.2f. Cost: %.2f, Contact prob: %.2f", best_margin, best_acceleration, best_speed, lowest_cost, best_contact_prob);
        ROS_INFO("Optimal action - Margin: %i, Acc: %i. Cost: %.2f, Output prob: %.2f", best_margin, best_acceleration, lowest_cost, best_output_probability);

        // increment time
        net.incrementTime();

        ROS_INFO("States:");
        auto temporal_leaf_node_states = net.getTemporalLeafNodeSate();
        std::stringstream print_text;
        for(const auto [node, state] : temporal_leaf_node_states){
            print_text.clear();
            print_text.str(std::string());
            print_text << std::fixed;
            print_text << std::setprecision(2);
            print_text << node << " = [";
            for(const auto [state_name, state_value] : state){
                print_text << state_value << ", ";
            }
            print_text << "]";
            ROS_INFO("%s", print_text.str().c_str());
        }
    }

public:
    SupervisoryRiskControl()
    {
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisory_risk_control");
    SupervisoryRiskControl node;

    return 0;
}