#include "mavros_msgs/DebugValue.h"
#include "supervisory_risk_control/bayesian_network.h"
#include "supervisory_risk_control/debug_array_id.hpp"
#include <supervisory_risk_control_msgs/debug_display.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <cmath>
#include <sstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <stdexcept>
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/ParamGet.h"
#include <stdexcept>
#include "supervisory_risk_control/utils.h"
#include <scout_msgs/DistanceDown.h>

class SupervisoryRiskControl
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp{"~"};

    //Data
    double height_over_ground = 0;
    double motor_use = 0;
    double yaw_moment = 0;
    double velocity_deviation = 0;
    int number_of_filtered_points = -1;
    double camera_noise = 0;
    double drone_tilt; 


    static constexpr double risk_limit = 0.01;

    const std::vector<std::string> causal_node_names = {"tether_tension_motor_use_scaling_factor", "drone_tilt_to_counteract_tether","motor_wear","motoruse_for_whirlewind","turbulence","dust","enviornment_observability"};
    BayesianNetwork net{ros::package::getPath("supervisory_risk_control") + "/include/supervisory_risk_control/net.xdsl", causal_node_names, 2};

    ros::Subscriber debug_value_subscriber = nh.subscribe<mavros_msgs::DebugValue>("/mavros/debug_value/debug_float_array", 1, [&](mavros_msgs::DebugValueConstPtr msg){
        motor_use = msg->data[topic_indices::motor_max];
        height_over_ground = msg->data[topic_indices::height_over_ground]; 
        yaw_moment = msg->data[topic_indices::yaw_moment];
        velocity_deviation = msg->data[topic_indices::turbulence];
        drone_tilt = msg->data[topic_indices::drone_roll_pitch];
    });
    ros::Subscriber number_of_filtered_points_subscriber = nh.subscribe<std_msgs::Int32>("/cloud_filter/number_of_removed_points", 1, [&](std_msgs::Int32ConstPtr msg){ 
        number_of_filtered_points=0.9*number_of_filtered_points + 0.1*msg->data;
    });
    ros::Subscriber noise_level_subscriber = nh.subscribe<std_msgs::Float32>("/supervisor/noise_level", 1, [&](std_msgs::Float32ConstPtr msg){ 
        camera_noise=msg->data;
    });

    ros::Publisher debug_display_publisher = nh.advertise<supervisory_risk_control_msgs::debug_display>("supervisory_risk_controller/debug_msg",1);
    supervisory_risk_control_msgs::debug_display debug_display;
    double height_over_ground;

    std::map<std::string, int> previous_action;

    const std::vector<std::string> output_node_names = {"frequency_of_motor_saturation", "frequency_of_loss_of_control_due_to_motor_wear", "frequency_of_exceeding_safety_marign_due_to_turbulence", "frequency_of_contact_with_unobservable_obstacle"};
    const std::vector<std::string> intermediate_estimation_node_names = {"motoruse_for_tether",};
    const std::vector<std::string> intermediate_binary_prediction_node_names = {};
    const std::vector<std::string> intermediate_linear_prediction_node_names = {};
    const std::vector<std::string> intermediate_log_prediction_node_names = {};
    const std::vector<std::string> all_estimate_node_names = [=]{
        auto ret = causal_node_names; 
        ret.insert(ret.end(),intermediate_estimation_node_names.begin(), intermediate_estimation_node_names.end()); 
        return ret;}();

    const std::vector<std::string> all_prediction_node_names = [=]{
        auto ret = output_node_names; 
        ret.insert(ret.end(),intermediate_linear_prediction_node_names.begin(), intermediate_linear_prediction_node_names.end()); 
        ret.insert(ret.end(),intermediate_log_prediction_node_names.begin(), intermediate_log_prediction_node_names.end()); 
        ret.insert(ret.end(),intermediate_binary_prediction_node_names.begin(), intermediate_binary_prediction_node_names.end()); 
        return ret;}();

    void set_observations(){
        //values are normalized to be within [0-10)

        {
            //motor saturation is at 1949 
        auto motor_use_state = std::clamp((int)std::floor(motor_use*10/1949),0,9);
        net.setEvidence("motor", motor_use_state);
        ROS_INFO("%s: %.2f, %i", "motor", motor_use, motor_use_state);
        debug_display.measured_motoruse = motor_use_state;
        }
        {
            //Max roll(pitch angle is 20 degrees)
        auto drone_tilt_state = std::clamp((int)std::floor(drone_tilt*10/20),0,9);
        net.setEvidence("measured_drone_roll_pitch", drone_tilt_state);
        ROS_INFO("%s: %.2f, %i", "measured_drone_roll_pitch", drone_tilt, drone_tilt_state);
        debug_display.measured_drone_roll_pitch = drone_tilt_state;
        }
        {
            //TODO!
        auto yaw_moment_state = std::clamp((int)std::floor(yaw_moment*10),0,9);
        net.setEvidence("yaw_moment", yaw_moment_state);
        ROS_INFO("%s: %.2f, %i", "yaw_moment", yaw_moment, yaw_moment_state);
        debug_display.measured_yaw_moment = yaw_moment_state;
        }
        {
            //max height is 40m
        auto height_over_ground_state = std::clamp((int)std::floor(height_over_ground*10/40),0,9);
        net.setEvidence("height_over_ground", height_over_ground_state);
        ROS_INFO("%s: %.2f, %i", "height_over_ground", height_over_ground, height_over_ground_state);
        debug_display.measured_height_over_ground = height_over_ground_state;
        }
        {
            //High turbulence area has measurements of 0.004-0.007, define max to be slightly higher as there might be ven worse cases.
        auto velocity_deviation_state = std::clamp((int)std::floor(velocity_deviation*10/0.008),0,9);
        net.setEvidence("velocity_deviation", velocity_deviation_state);
        ROS_INFO("%s: %.2f, %i", "velocity_deviation", velocity_deviation, velocity_deviation_state);
        debug_display.measured_velocity_deviation = velocity_deviation_state;
        }
        {
            //Human input of value between 0 and 1
        auto camera_noise_state = std::clamp((int)std::floor(camera_noise*10),0,9);
        net.setEvidence("camera_noise", camera_noise_state);
        ROS_INFO("%s: %.2f, %i", "camera_noise", camera_noise, camera_noise_state);
        debug_display.measured_camera_noise = camera_noise_state;
        }
        {
                //Not a clear connection between worse conditions and increased number of points. 0 is defineatly good, and is 0 most of the time. 10 seems like a safe place to define as there being an undetectable obstacle close by
        auto number_of_filtered_points_state = std::clamp((int)std::floor(number_of_filtered_points),0,9);
        net.setEvidence("lidar_flicker", number_of_filtered_points_state);
        ROS_INFO("%s: %.2f, %i", "lidar_flicker", number_of_filtered_points, number_of_filtered_points_state);
        debug_display.measured_lidar_flicker = number_of_filtered_points_state;
        }        
    }

    double evaluate_risk(const std::map<std::string,std::map<std::string,double>>& output, const std::map<std::string, int>& actions, bool do_print=false) const{
        double max_speed = actions.at("max_speed_setpoint")/9.0;
        double safety_margin = actions.at("safety_margin")/9.0;

        double mean_frequency_of_motor_saturation = 1.2*log_mean(output.at("frequency_of_motor_saturation"));
        double mean_frequency_of_motor_saturation_deviating_beyond_safety_margin = mean_frequency_of_motor_saturation*(0.3+0.7*safety_margin);
        double mean_frequency_of_turbulence_causing_deviation_beyond_safety_margin = log_mean(output.at("frequency_of_exceeding_safety_marign_due_to_turbulence"));
        double mean_frequency_of_contact_with_unobservable_obstacle = log_mean(output.at("frequency_of_contact_with_unobservable_obstacle"));
        
        double mean_frequency_of_contact_causing_loss_of_control =
                mean_frequency_of_motor_saturation_deviating_beyond_safety_margin*(0.1+0.2*std::pow(max_speed,2)) 
                + mean_frequency_of_turbulence_causing_deviation_beyond_safety_margin*(0.05+0.2*std::pow(max_speed,2))
                + mean_frequency_of_contact_with_unobservable_obstacle*(0.01+0.2*std::pow(max_speed,2));

        double mean_frequency_of_motor_wear_causing_loss_of_control = log_mean(output.at("frequency_of_loss_of_control_due_to_motor_wear"));

        double mean_frequency_of_loss_of_control = mean_frequency_of_contact_causing_loss_of_control+mean_frequency_of_motor_wear_causing_loss_of_control;

        double mean_risk_damage_to_drone = mean_frequency_of_loss_of_control*(0.3+0.7*height_over_ground);
        double mean_risk_loss_of_mission= mean_frequency_of_loss_of_control;
        double total_risk = 0.8*mean_risk_damage_to_drone + 0.2*mean_risk_loss_of_mission;

        if(do_print){
            std::cout << "Mean freq of motor saturation: " << mean_frequency_of_motor_saturation 
                      << "\nMean freq of motor sat deviating beyond: " << mean_frequency_of_motor_saturation_deviating_beyond_safety_margin
                      << "\nMean freq of turbulence deviation beyond: " << mean_frequency_of_turbulence_causing_deviation_beyond_safety_margin
                      << "\nMean freq of contact with unobs obst: " << mean_frequency_of_contact_with_unobservable_obstacle
                      << "\nMean freq of contact causing loss of control: " << mean_frequency_of_contact_causing_loss_of_control
                      << "\nMean freq of motor wear causing loss of control: " << mean_frequency_of_motor_wear_causing_loss_of_control
                      << "\nMean freq of loss of control: " << mean_frequency_of_loss_of_control
                      << "\nRisk: " << total_risk <<std::endl;
        }

        return total_risk;
    }

    bool evaluate_action_feasibility(const std::map<std::string, int>& actions){
        net.setEvidence(actions);
        auto output = net.evaluateStates(output_node_names);
        return evaluate_risk(output, actions) < risk_limit;
    }

    auto heuristic_search(){
        std::map<std::string, int> action;
        action["safety_margin"]  = 0; //OBS denne er definert feil vei, finn ut hva å gjøre med det
        action["acceleration_limit"]  = 0;
        action["max_speed_setpoint"]  = 0;       
        
        std::vector<std::string> active_parameters = {"safety_margin", "acceleration_limit", "max_speed_setpoint"};
        std::map<std::string, int> parameter_max;
        parameter_max["safety_margin"]  = 9;
        parameter_max["acceleration_limit"]  = 4;
        parameter_max["max_speed_setpoint"]  = 9; 
        bool search_failed = false;
        bool search_completed = false;
        while (active_parameters.size()>0)
        {
            if(!evaluate_action_feasibility(action)){
                //If not feasible, go back to previous feasible value
                for(const auto& param : active_parameters){
                    action.at(param)--;
                    if(action.at(param)<0){
                        //there is not feasable solution, end the search;
                        action.at(param)=0;
                        search_failed = true;
                        break;
                    }
                }
                if(search_failed) break;
                //Find a parameter that cannot be improved
                bool did_erase_something = false;
                for(auto param = active_parameters.begin(); param !=  active_parameters.end(); ++param){
                    action.at(*param)++;
                    bool feasible = evaluate_action_feasibility(action);
                    action.at(*param)--;
                    if(feasible == false){
                        active_parameters.erase(param);
                        //then continue the search without modifying this parameter
                        did_erase_something = true;
                        break;
                    }
                }
                if(!did_erase_something){
                    active_parameters.pop_back();
                }
            }
            for(const auto& param : active_parameters){
                action.at(param)++;
                    if(action.at(param)>parameter_max.at(param)){
                        search_completed=true;
                        action.at(param)=parameter_max.at(param);
                        break;
                    }
            }
            if(search_completed) break;
        }
        return action;
    }
    
    auto exact_search(bool close_mode){
        std::map<std::string, int> optimal_action;
        optimal_action["safety_margin"]  = 0; //OBS denne er definert feil vei, finn ut hva å gjøre med det
        optimal_action["acceleration_limit"]  = 0;
        optimal_action["max_speed_setpoint"]  = 0; 
        std::map<std::string, int> action;
        double optimal_cost = INFINITY;

        double min_safety_margin = 0;
        double max_speed = 10;
        if(close_mode){
            max_speed = 5;
        }
        else{
            min_safety_margin = 5;
        }

        for(int safety_margin =min_safety_margin; safety_margin<10; ++safety_margin){
            action["safety_margin"]  = safety_margin; //OBS denne er definert feil vei, finn ut hva å gjøre med det
            for(int acceleration_limit=0; acceleration_limit<5; ++acceleration_limit){
                action["acceleration_limit"]  = acceleration_limit;
                //max speed does not affect the network
                net.setEvidence(action);
                auto output = net.evaluateStates(output_node_names);
                for(int max_speed=0; max_speed<max_speed; ++max_speed){
                    action["max_speed_setpoint"]  = max_speed; 
                    bool is_feasible = evaluate_risk(output, action) < risk_limit;
                    double parameter_cost = std::pow(9-safety_margin,1)+0.6*std::pow(4-acceleration_limit,1)+std::pow(9-max_speed,1);

                    double change_cost = std::pow(safety_margin-previous_action["safety_margin"],2)+std::pow(acceleration_limit-previous_action["acceleration_limit"],2)+std::pow(max_speed-previous_action["max_speed_setpoint"],2);

                    double total_cost = parameter_cost+0.3*change_cost;

                    if(is_feasible && total_cost<optimal_cost){
                        optimal_action = action;
                        optimal_cost = total_cost;
                    }
                }
            }
        }
        return optimal_action;
    }

    void run()
    {
        try{

        auto param_get = mavros_msgs::ParamGet{};
        param_get.request.param_id = 

        if (!ros::service::call("/mavros/param/set", param_set) ||
        (param_set.response.success == 0u))
        throw "Failed to write PX4 parameter " + param_set.request.param_id;*/
        set_observations();
        {
        auto output = net.evaluateStates(all_estimate_node_names);
        debug_display.mean_tether_tension_motor_use_scaling_factor = mean(output.at("tether_tension_motor_use_scaling_factor"));
        debug_display.mean_drone_roll_pitch_to_counteract_tether = mean(output.at("drone_tilt_to_counteract_tether"));
        debug_display.mean_motor_wear = mean(output.at("motor_wear"));
        debug_display.mean_motoruse_for_whirlewind = mean(output.at("motoruse_for_whirlewind"));
        debug_display.mean_motoruse_for_tether = mean(output.at("motoruse_for_tether"));
        }

        //Worst case:
        {
        std::map<std::string, int> action;
        action["safety_margin"]  = 9; //OBS denne er definert feil vei, finn ut hva å gjøre med det
        action["acceleration_limit"]  = 4;
        action["max_speed_setpoint"]  = 9; 
        net.setEvidence(action);
        auto output = net.evaluateStates(all_prediction_node_names);
        double risk_cost = evaluate_risk(output,action,true);
        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f", action.at("safety_margin"), action.at("acceleration_limit"), action.at("max_speed_setpoint"), risk_cost);
        }
       
        //auto action = heuristic_search(); 
        {      
        auto action = exact_search();       
        net.setEvidence(action);
        auto output = net.evaluateStates(all_prediction_node_names);
        debug_display.risk_cost = evaluate_risk(output,action,true);
        debug_display.optimal_safety_margin = action.at("safety_margin");
        debug_display.optimal_max_acc = action.at("acceleration_limit");
        debug_display.optimal_speed = action.at("max_speed_setpoint");
        debug_display.found_feasible_option = debug_display.risk_cost<risk_limit;
        
        debug_display.mean_frequency_of_motor_saturation = log_mean(output.at("frequency_of_motor_saturation"));
        debug_display.mean_frequency_of_loss_of_control_due_to_motor_wear = log_mean(output.at("frequency_of_loss_of_control_due_to_motor_wear"));
        debug_display.mean_frequency_of_exceeding_safety_marign_due_to_turbulence = log_mean(output.at("frequency_of_exceeding_safety_marign_due_to_turbulence"));
        debug_display.mean_frequency_of_contact_with_unobservable_obstacle = log_mean(output.at("frequency_of_contact_with_unobservable_obstacle"));
        debug_display_publisher.publish(debug_display);

        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f", action.at("safety_margin"), action.at("acceleration_limit"), action.at("max_speed_setpoint"), debug_display.risk_cost);
        previous_action = action;
        }
        /*auto exact_action = exact_search(); 
        net.setEvidence(exact_action);
        output = net.evaluateStates(all_prediction_node_names);
        double risk_cost = evaluate_risk(output,exact_action,true);
        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f", exact_action.at("safety_margin"), exact_action.at("acceleration_limit"), exact_action.at("max_speed_setpoint"), risk_cost);
        */

        auto param_set = mavros_msgs::ParamSet{};
        //
        param_set.request.param_id = "SA_DISTANCE";
        param_set.request.value.real = best_margin;

        if (!ros::service::call("/mavros/param/set", param_set) ||
        (param_set.response.success == 0u))
        throw "Failed to write PX4 parameter " + param_set.request.param_id;


        net.incrementTime();
        }
        catch ( const std::out_of_range& oor){
            std::cout << "ERROR attempted to acces non-existing element, what: " << oor.what() <<std::endl;
        }
    }

public:
    SupervisoryRiskControl()
    {
        ros::spinOnce();
        run();
        ros::Duration(1).sleep();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisory_risk_control");
    SupervisoryRiskControl node;

    return 0;
}