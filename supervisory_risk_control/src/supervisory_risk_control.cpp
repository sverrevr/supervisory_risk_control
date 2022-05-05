#include "mavros_msgs/DebugValue.h"
#include "supervisory_risk_control/bayesian_network.h"
#include "supervisory_risk_control/classifiers.hpp"
#include "supervisory_risk_control/debug_array_id.hpp"
#include <supervisory_risk_control_msgs/debug_display.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <cmath>
#include <sstream>
#include "mavros_msgs/ParamSet.h"
#include <stdexcept>
#include "supervisory_risk_control/utils.h"

class SupervisoryRiskControl
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp{"~"};

    //const std::vector<std::string> temporal_leaf_nodes = {"obstacle_missing_from_lidar_scan","turbulence_geometry_følsomhet", "other_disturbances", "velocity_deviation_distance_scaling_factor", "distance_deviation_when_loss_of_control", "motorbruk_fra_slitasje_eller_tether", "velocity_deviation_when_loss_of_control", "other_reason_for_roll_pitchrate_devitaiton"};
    //const std::vector<std::string> temporal_leaf_nodes = {"varying_motor_use", "mean_motor_use"};
    const std::vector<std::string> temporal_leaf_nodes = {"Turbulence_geometry_suceptability","Other_disturbance_causes","Tether_sway_suceptability", "Tether_friction", "motor_propeller_wear"};
    BayesianNetwork net{ros::package::getPath("supervisory_risk_control") + "/include/supervisory_risk_control/net.xdsl",
                        temporal_leaf_nodes, 2};

    ros::Subscriber debug_value_subscriber = nh.subscribe("/mavros/debug_value/debug_float_array", 1, &SupervisoryRiskControl::debug_value_CB, this);
    ros::Publisher debug_display_publisher = nh.advertise<supervisory_risk_control_msgs::debug_display>("supervisory_risk_controller/debug_msg",1);
    supervisory_risk_control_msgs::debug_display debug_display;

    const std::vector<std::string> output_node_names = {"H__Loss_of_control"};
    const std::vector<std::string> intermediate_estimation_node_names = {"turbulence","tether_sway","tether_drag"};
    const std::vector<std::string> intermediate_prediction_node_names = {"S__Disturbance_causes_deviation_beyond_safety_margin", "S__loss_of_control_authority_causes_deviation_beyond_safety_margin", "H__Contact_with_obstacle", "S__Contact_with_obstacle_causes_loss_of_control", "S__Disturbance_causes_loss_of_control"};
    const std::vector<std::string> all_estimate_node_names = [=]{
        auto ret = temporal_leaf_nodes; 
        ret.insert(ret.end(),intermediate_estimation_node_names.begin(), intermediate_estimation_node_names.end()); 
        return ret;}();

    const std::vector<std::string> all_prediction_node_names = [=]{
        auto ret = output_node_names; 
        ret.insert(ret.end(),intermediate_prediction_node_names.begin(), intermediate_prediction_node_names.end()); 
        return ret;}();

    void set_observations(mavros_msgs::DebugValueConstPtr msg){
        auto motor_use_state = motor_use_clasifier(msg->data[topic_indices::motor_max]);
        net.setEvidence("Measured_motor_use", motor_use_state);
        ROS_INFO("%s: %.2f, %i", "Measured_motor_use",msg->data[topic_indices::motor_max], motor_use_state);
        debug_display.measured_motor_use = motor_use_state;

        auto js_derivative_state = js_derivative_clasifier(msg->data[topic_indices::js_derivative]);
        net.setEvidence("current_change_in_joystick_input", js_derivative_state);
        ROS_INFO("%s: %.2f, %i", "current_change_in_joystick_input",msg->data[topic_indices::js_derivative], js_derivative_state);
        debug_display.current_joystick_input = js_derivative_state;

        auto loss_of_control_authority_state = loss_of_control_authority_classifier(msg->data[topic_indices::yawrate_integral]);
        net.setEvidence("Has_currently_lost_control_authority", loss_of_control_authority_state);
        ROS_INFO("%s: %.2f, %i", "Has_currently_lost_control_authority",msg->data[topic_indices::yawrate_integral], loss_of_control_authority_state);
        debug_display.has_currently_lost_control_authority = loss_of_control_authority_state;

        auto velocity_deviation_state = velocity_deviation_classifier(msg->data[topic_indices::vxvy_error_component]);
        net.setEvidence("Velocity_deviation", velocity_deviation_state);
        ROS_INFO("%s: %.2f, %i", "Velocity_deviation",msg->data[topic_indices::vxvy_error_component], velocity_deviation_state);
        debug_display.velocity_deviation = velocity_deviation_state;

        net.setEvidence("Turublence_geometry", 0);
        ROS_INFO("%s: %.2f, %i", "Turublence_geometry", NAN, 0);
        debug_display.turbulence_geometry = 0;

        //TODO! Fiks denne, det er ikke mye jobb
        net.setEvidence("height_over_ground", 0);
        ROS_INFO("%s: %.2f, %i", "height_over_ground", NAN, 0);
        debug_display.height_over_ground = 0;
    }

    void debug_value_CB(mavros_msgs::DebugValueConstPtr msg)
    {
        ROS_INFO("\n\n\nNew measurement");
        set_observations(msg);

        {
        auto output = net.evaluateStates(all_estimate_node_names);
        debug_display.turbulence_geometry_suceptability_mean = mean(output.at("Turbulence_geometry_suceptability"));
        debug_display.turbulence_mean = mean(output.at("turbulence"));
        debug_display.other_disturbance_mean = mean(output.at("Other_disturbance_causes"));
        debug_display.tether_sway_suceptability_mean = mean(output.at("Tether_sway_suceptability"));
        debug_display.tether_sway_mean = mean(output.at("tether_sway"));
        debug_display.tether_drag_mean = mean(output.at("tether_drag"));
        debug_display.tether_friction_mean = mean(output.at("Tether_friction"));
        debug_display.motor_wear_mean = mean(output.at("motor_propeller_wear"));
        }

        double lowest_cost = INFINITY;
        int best_margin = -1;
        int best_acceleration_limit = -1;
        int best_max_speed = -1;
        double best_output_probability = -1;

        std::map<std::string, int> action;
        for(int safety_margin = 0; safety_margin < 5; ++safety_margin){
            action["safety_margin"] =  safety_margin;
            for(int acceleration_limiter = 0; acceleration_limiter < 5; ++acceleration_limiter){
                action["acceleration_limiter"] =  acceleration_limiter;
                for(int max_speed = 0; max_speed < 5; ++max_speed){
                    action["max_speed"] =  max_speed;

                    net.setEvidence(action);
                    auto output = net.evaluateStates(output_node_names);
                    auto loss_of_control_prob = output.at(output_node_names[0]).at("State1");

                    static constexpr double safety_margin_penalty = 1;
                    static constexpr double max_acceleration_penalty = 1;
                    static constexpr double speed_penalty = 1;
                    static constexpr double loss_of_control_cost = 10;

                    const double safety_margin_cost = std::pow(safety_margin/4.0,3.0)*safety_margin_penalty;
                    const double max_acc_cost = std::pow(acceleration_limiter/4.0,3.0)*max_acceleration_penalty;
                    const double speed_cost = std::pow(1-(max_speed/4),3.0)*speed_penalty;
                    const double risk_cost = loss_of_control_prob*loss_of_control_cost;

                    const double cost = safety_margin_cost+max_acc_cost+speed_cost+risk_cost;

                    if(cost<lowest_cost){
                        lowest_cost = cost;
                        best_margin = safety_margin;
                        best_acceleration_limit = acceleration_limiter;
                        best_max_speed = max_speed;
                        best_output_probability = loss_of_control_prob;
                    }
                }
            }
        }
        debug_display.optimal_cost = lowest_cost;
        debug_display.optimal_safety_margin = best_margin;
        debug_display.optimal_max_acc = best_acceleration_limit;
        debug_display.optimal_speed = best_max_speed;
        action["safety_margin"] =  best_margin;
        action["acceleration_limiter"] =  best_acceleration_limit;
        action["max_speed"] =  best_max_speed;
        {
        net.setEvidence(action);
        auto output = net.evaluateStates(all_prediction_node_names);
        debug_display.loss_of_control = output.at("H__Loss_of_control").at("State1");
        debug_display.disturbance_causes_loss_of_control = output.at("S__Disturbance_causes_loss_of_control").at("State1");
        debug_display.contact_with_obstacle_causes_loss_of_control = output.at("S__Contact_with_obstacle_causes_loss_of_control").at("State1");
        debug_display.contact_with_obstacle = output.at("H__Contact_with_obstacle").at("State1");
        debug_display.disturbance_causes_deviation_beyond_safety_margin = output.at("S__Disturbance_causes_deviation_beyond_safety_margin").at("State1");
        debug_display.loss_of_control_authortiy_causes_deviation_beyond_safety_margin = output.at("S__loss_of_control_authority_causes_deviation_beyond_safety_margin").at("State1");
        }

        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Cost: %.1f, Loss prob: %0.2f", best_margin, best_acceleration_limit, best_max_speed, lowest_cost,best_output_probability);
        //ROS_INFO("Optimal action - Margin: %i, Acc: %i. Cost: %.2f, Output prob: %.2f", best_margin, best_acceleration_limit, lowest_cost);
        //Publiser på en ros topic ønkset hastighet, margin, etc, så man kan se på dem i plotjuggler 

        /*auto param_set = mavros_msgs::ParamSet{};
        param_set.request.param_id = "SA_DISTANCE";
        param_set.request.value.real = best_margin;

        if (!ros::service::call("/mavros/param/set", param_set) ||
        (param_set.response.success == 0u))
        throw "Failed to write PX4 parameter " + param_set.request.param_id;*/

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

        debug_display_publisher.publish(debug_display);
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