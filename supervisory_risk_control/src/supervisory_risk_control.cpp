#include "mavros_msgs/DebugValue.h"
#include "supervisory_risk_control/bayesian_network.h"
#include "supervisory_risk_control/classifiers.hpp"
#include "supervisory_risk_control/debug_array_id.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <cmath>

class SupervisoryRiskControl
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp{"~"};

    const std::vector<std::string> temporal_leaf_nodes = {"obstacle_missing_from_lidar_scan","turbulence_geometry_følsomhet", "other_disturbances", "velocity_deviation_distance_scaling_factor", "distance_deviation_when_loss_of_control", "motorbruk_fra_slitasje_eller_tether", "velocity_deviation_when_loss_of_control", "other_reason_for_roll_pitchrate_devitaiton"};
    BayesianNetwork net{ros::package::getPath("supervisory_risk_control") + "/include/supervisory_risk_control/2feb_simplified Discretized.xdsl",
                        temporal_leaf_nodes, 2};

    ros::Subscriber debug_value_subscriber = nh.subscribe("/mavros/debug_value/debug_float_array", 1, &SupervisoryRiskControl::debug_value_CB, this);

    void debug_value_CB(mavros_msgs::DebugValueConstPtr msg)
    {
        printf("\033c");
        ROS_INFO("%f", msg->data[0]);

        // put all values as evidence on measurement nodes
        net.setEvidence("motor_overhead", motor_overhead_clasifier(msg->data[MOTOR_OVERHEAD]));
        printf("%s: %d, %i", "motor_overhead",msg->data[MOTOR_OVERHEAD], motor_overhead_clasifier(msg->data[MOTOR_OVERHEAD]));

        net.setEvidence("js_derivat", js_derivative_clasifier(msg->data[JS_DERIVATIVE]));
        printf("%s: %d, %i", "js_derivat",msg->data[JS_DERIVATIVE], js_derivative_clasifier(msg->data[JS_DERIVATIVE]));

        net.setEvidence("loss_of_control_authority", loss_of_control_authority_classifier(msg->data[LOSS_OF_CONTROL_AUTHORITY]));
        printf("%s: %d, %i", "loss_of_control_authority",msg->data[LOSS_OF_CONTROL_AUTHORITY], loss_of_control_authority_classifier(msg->data[LOSS_OF_CONTROL_AUTHORITY]));

        net.setEvidence("velocity_deviation", velocity_deviation_classifier(msg->data[VELOCITY_DEVIATION]));
        printf("%s: %d, %i", "velocity_deviation",msg->data[VELOCITY_DEVIATION], velocity_deviation_classifier(msg->data[VELOCITY_DEVIATION]));

        net.setEvidence("collision_speed", collision_speed_classifier(msg->data[COLLISION_SPEED]));
        printf("%s: %d, %i", "collision_speed",msg->data[COLLISION_SPEED], collision_speed_classifier(msg->data[COLLISION_SPEED]));

        net.setEvidence("roll_pitchrate_deviation", rp_rate_deviation_classifier(msg->data[RP_RATE_DEVIATION]));
        printf("%s: %d, %i", "roll_pitchrate_deviation",msg->data[RP_RATE_DEVIATION], rp_rate_deviation_classifier(msg->data[RP_RATE_DEVIATION]));

        net.setEvidence("is_close_obstacle", is_close_to_obstacle_classifier(msg->data[IS_CLOSE_TO_OBSTACLE]));
        printf("%s: %d, %i", "is_close_obstacle",msg->data[IS_CLOSE_TO_OBSTACLE], is_close_to_obstacle_classifier(msg->data[IS_CLOSE_TO_OBSTACLE]));

        net.setEvidence("turbulence_geometry", 2);
        printf("%s: %d, %i", "turbulence_geometry", 2, 2);

        // evalaute different actions
        double lowest_cost = INFINITY;
        int best_margin = 0;
        int best_acceleration = 0;
        double best_speed = 0;
        for(int safety_margin = 0; safety_margin < 10; safety_margin+=2){
            net.setEvidence("safety_margin", safety_margin);
            for(int max_acceleration = 0; max_acceleration < 10; max_acceleration+=2){
                net.setEvidence("motorbruk_til_maks_akselerasjon", max_acceleration);
                auto contact_prob = net.evaluateStates({"probability_that_we_will_contact"}).at("probability_that_we_will_contact");
                for(double speed = 0.1; speed < 1.5; speed+=0.3){
                    static constexpr double safety_margin_penalty = 1;
                    static constexpr double max_acceleration_penalty = 1;
                    static constexpr double speed_penalty = 1;
                    static constexpr double risk_penalty = 10;
                    double cost = std::pow(1-safety_margin/10,2)*1 + std::pow(1-max_acceleration/10,1)*1+std::pow(1-speed/1.5,2)*1 + contact_prob.at("true")*std::pow(speed,2);
                    if(cost<lowest_cost){
                        lowest_cost = cost;
                        best_margin = safety_margin;
                        best_acceleration = max_acceleration;
                        best_speed = speed;
                    }
                }
            }
        }

        printf("Optimal action - Margin: %i, Acc: %i, Speed: %f. Cost: %f", best_margin,best_acceleration,best_speed, lowest_cost);

        // increment time
        net.incrementTime();
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