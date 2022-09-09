#include "mavros_msgs/DebugValue.h"
#include "supervisory_risk_control/bayesian_network.h"
#include "supervisory_risk_control/debug_array_id_real.hpp"
#include <supervisory_risk_control_msgs/debug_display.h>
#include <supervisory_risk_control_msgs/actions.h>
#include <supervisory_risk_control_msgs/measurements.h>
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

class SupervisoryRiskControl
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp{"~"};

    // Data
    double height_over_ground = 0;
    double motor_use = 0;
    double yaw_moment = 0;
    double roll_pitch_deviation = 0;
    double number_of_filtered_points = 0;
    double camera_noise = 0;
    double drone_tilt = 0;
    int max_speed_target = 9;
    int safety_margin_target = 0;
    enum class last_set_target_type {none, speed, margin} last_set_target = last_set_target_type::none;
    bool new_data = true;
    supervisory_risk_control_msgs::measurements measurement_msg;

    double previous_safety_margin=0.0;
    double previous_max_speed=9.0;
    double previous_max_acc=4.0;
    bool do_abort = false;

    struct{
        bool dynamic, modify_parameters, only_update_on_measurements, spam_states, no_delay;
        double saving_factor, max_risk;
        struct{
            double max_yaw_moment, max_turbulence, min_turbulence, max_number_of_filtered_points, motor_max, motor_min, max_tilt, min_tilt, filtered_point_lowpass_factor;
        } measurement_conversion;
        struct{
            struct {
                double constant, scale;
            } motor_saturation, turbulence, breaking_distance, damage;
            struct{
               double damage_to_drone, loss_of_mission;
            } total;
        } risk;

        struct{
            double target_cost, non_target_cost, acceleration_cost, parameter_change_cost, parameter_change_delay_factor;
        } cost_function_parameters;
    } pars;

    const std::vector<std::string> causal_node_names = {"environment_observability", "motor_wear", "random_disturbance", "turbulence"};
    BayesianNetwork net;

    ros::Subscriber debug_value_subscriber = nh.subscribe<mavros_msgs::DebugValue>("/mavros/debug_value/debug_float_array", 1, [&](mavros_msgs::DebugValueConstPtr msg)
                                                                                   {
        motor_use = msg->data[topic_indices::motor_max];
        height_over_ground = msg->data[topic_indices::height_over_ground];
        yaw_moment = abs(msg->data[topic_indices::yaw_moment]);
        roll_pitch_deviation = msg->data[topic_indices::roll_pitch_ref_error];
        drone_tilt = msg->data[topic_indices::roll_pitch_js_error]; 
        new_data=true;});
    ros::Subscriber number_of_filtered_points_subscriber = nh.subscribe<std_msgs::Int32>("/cloud_filter/number_of_removed_points", 1, [&](std_msgs::Int32ConstPtr msg)
                                                                { number_of_filtered_points = pars.measurement_conversion.filtered_point_lowpass_factor * number_of_filtered_points 
                                                                                            + (1-pars.measurement_conversion.filtered_point_lowpass_factor) * msg->data;});
    ros::Subscriber noise_level_subscriber = nh.subscribe<std_msgs::Float32>("/supervisor/noise_level", 1, [&](std_msgs::Float32ConstPtr msg)
                                                                             { camera_noise = msg->data; });
    ros::Subscriber max_speed_target_subscriber = nh.subscribe<std_msgs::Float32>("/supervisor/max_speed_target", 1, [&](std_msgs::Float32ConstPtr msg)
                                                                             { max_speed_target = std::clamp((int)(msg->data*10),0,9); 
                                                                                last_set_target = last_set_target_type::speed;
                                                                                ROS_INFO("Setting max-speed target to: %d",max_speed_target);});
    ros::Subscriber safety_margin_target_subscriber = nh.subscribe<std_msgs::Float32>("/supervisor/safety_margin_target", 1, [&](std_msgs::Float32ConstPtr msg)
                                                                             { safety_margin_target = std::clamp((int)(msg->data*10),0,9); 
                                                                                last_set_target = last_set_target_type::margin;
                                                                                ROS_INFO("Setting safety-margin_target to %d", safety_margin_target);});
    ros::Subscriber disable_target_subscriber = nh.subscribe<std_msgs::Float32>("/supervisor/disable_target", 1, [&](std_msgs::Float32ConstPtr msg)
                                                                             { last_set_target = last_set_target_type::none;
                                                                             ROS_INFO("Disabling target.");});

    ros::Subscriber abort_subscriber = nh.subscribe<std_msgs::Float32>("/supervisor/abort", 1, [&](std_msgs::Float32ConstPtr msg)
            {  
            auto param_set = mavros_msgs::ParamSet{};
            param_set.request.param_id = "SA_DISTANCE";
            param_set.request.value.real = 1.2;
            if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
                throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};

            param_set.request.param_id = "MPC_VEL_MANUAL";
            param_set.request.value.real = 1;
            if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
                throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};

            param_set.request.param_id = "MPC_ACC_DOWN_MAX";
            param_set.request.value.real = 2;
            if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
                throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};
            param_set.request.param_id = "MPC_ACC_UP_MAX";
            if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
                throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};
            do_abort = true;});

    ros::Subscriber set_measurements_subscriber = nh.subscribe<supervisory_risk_control_msgs::measurements>("/supervisor/set_measurements", 1, [&](supervisory_risk_control_msgs::measurementsConstPtr msg)
    {
        measurement_msg = *msg;
        new_data = true;
    });

    ros::Subscriber disable_direct_measurements = nh.subscribe<std_msgs::Int32>("/suervisor/disable_direct_measurements",1,[&](std_msgs::Int32ConstPtr msg){
        measurement_msg.camera_noise = -1;
        measurement_msg.drone_tilt = -1;
        measurement_msg.height_over_ground = -1;
        measurement_msg.motor_mean = -1;
        measurement_msg.number_of_filtered_away_points = -1;
        measurement_msg.roll_pitch_deviation = -1;
    });

    ros::Publisher debug_display_publisher = nh.advertise<supervisory_risk_control_msgs::debug_display>("supervisory_risk_controller/debug_msg", 1);
    supervisory_risk_control_msgs::debug_display debug_display;

    map_stringkey<int> previous_action{std::map<std::string,int>{{"max_speed",9}, {"safety_margin",0}, {"max_upwards_acceleration",4}}};

    const std::vector<std::string> output_node_names = {"frequency_of_motor_saturation_deviating_beyond_safety_margin", "frequency_of_loss_of_control_due_to_motor_wear", "frequency_of_exceeding_safety_margin_due_to_turbulence", "frequency_of_breaking_distance_exceeding_safety_margin", "frequency_of_loss_of_control_due_to_turbulence","effective_safety_margin"};
    const std::vector<std::string> intermediate_estimation_node_names = {
        "motoruse_for_tether", "presence_of_unobservable_obstacle", "effective_safety_margin", "filtered_motor_use"
    };
    const std::vector<std::string> intermediate_binary_prediction_node_names = {};
    const std::vector<std::string> intermediate_linear_prediction_node_names = {};
    const std::vector<std::string> intermediate_log_prediction_node_names = {};
    const std::vector<std::string> all_estimate_node_names = [=]
    {
        auto ret = causal_node_names;
        ret.insert(ret.end(),intermediate_estimation_node_names.begin(), intermediate_estimation_node_names.end());
        return ret; }();

    const std::vector<std::string> all_prediction_node_names = [=]
    {
        auto ret = output_node_names;
        ret.insert(ret.end(),intermediate_linear_prediction_node_names.begin(), intermediate_linear_prediction_node_names.end());
        ret.insert(ret.end(),intermediate_log_prediction_node_names.begin(), intermediate_log_prediction_node_names.end());
        ret.insert(ret.end(),intermediate_binary_prediction_node_names.begin(), intermediate_binary_prediction_node_names.end());
        return ret; }();

    void set_observations()
    {
        // values are normalized to be within [0-10)

        {
            if(measurement_msg.motor_mean<0){
            // motor saturation is at 1949
                auto motor_use_state = std::clamp((int)std::floor(10*(motor_use-pars.measurement_conversion.motor_min)/(pars.measurement_conversion.motor_max-pars.measurement_conversion.motor_min)), 0, 9);
                net.setEvidence("mean_motor", motor_use_state);
                ROS_INFO("%s: %.2f, %i", "mean_motor", motor_use, motor_use_state);
                debug_display.measured_motoruse = motor_use_state/10.0;
            }
            else{
                net.setEvidence("mean_motor", measurement_msg.motor_mean);
                ROS_INFO("%s: %i", "mean_motor", measurement_msg.motor_mean);
                debug_display.measured_motoruse = measurement_msg.motor_mean/10.0;
            }
        }
        {
            if(measurement_msg.drone_tilt<0){
            // Max roll/pitch
            auto drone_tilt_state = std::clamp((int)std::floor((drone_tilt-pars.measurement_conversion.min_tilt) * 10 / pars.measurement_conversion.max_tilt), 0, 9);
            net.setEvidence("drone_tilt", drone_tilt_state);
            ROS_INFO("%s: %.2f, %i", "drone_tilt", drone_tilt, drone_tilt_state);
            debug_display.measured_drone_roll_pitch = drone_tilt_state/10.0;
            }
            else{
                net.setEvidence("drone_tilt", measurement_msg.drone_tilt);
            ROS_INFO("%s: %i", "drone_tilt", measurement_msg.drone_tilt);
            debug_display.measured_drone_roll_pitch = measurement_msg.drone_tilt/10.0;
            }
        }
        {
            /*auto yaw_moment_state = std::clamp((int)std::floor(yaw_moment * 10/pars.measurement_conversion.max_yaw_moment), 0, 9);
            net.setEvidence("yaw_moment", yaw_moment_state);
            ROS_INFO("%s: %.2f, %i", "yaw_moment", yaw_moment, yaw_moment_state);
            debug_display.measured_yaw_moment = yaw_moment_state/10.0;*/
        }
        {
            if(measurement_msg.height_over_ground<0){
            // max height is 40m
            auto height_over_ground_state = std::clamp((int)std::floor(height_over_ground * 10 / 40), 0, 9);
            net.setEvidence("height_over_ground", height_over_ground_state);
            ROS_INFO("%s: %.2f, %i", "height_over_ground", height_over_ground, height_over_ground_state);
            debug_display.measured_height_over_ground = height_over_ground_state/10.0;
            }
            else{
            net.setEvidence("height_over_ground", measurement_msg.height_over_ground);
            ROS_INFO("%s: %i", "height_over_ground", measurement_msg.height_over_ground);
            debug_display.measured_height_over_ground = measurement_msg.height_over_ground/10.0;
            height_over_ground = measurement_msg.height_over_ground*40.0/9.0;
            }
        }
        {
            if(measurement_msg.roll_pitch_deviation<0){
            // High turbulence area has measurements of 0.004-0.007, define max to be slightly higher as there might be even worse cases.
            auto velocity_deviation_state = std::clamp((int)std::floor(10*(roll_pitch_deviation-pars.measurement_conversion.min_turbulence)/(pars.measurement_conversion.max_turbulence-pars.measurement_conversion.min_turbulence)), 0, 9);
            net.setEvidence("roll_pitch_deviation", velocity_deviation_state);
            ROS_INFO("%s: %.2f, %i", "roll_pitch_deviation", roll_pitch_deviation, velocity_deviation_state);
            debug_display.measured_velocity_deviation = velocity_deviation_state/10.0;
            }
            else{
                net.setEvidence("roll_pitch_deviation", measurement_msg.roll_pitch_deviation);
            ROS_INFO("%s: %i", "roll_pitch_deviation", measurement_msg.roll_pitch_deviation);
            debug_display.measured_velocity_deviation = measurement_msg.roll_pitch_deviation/10.0;    
            }
        }
        {
            if(measurement_msg.camera_noise<0){
            // Human input of value between 0 and 1
            auto camera_noise_state = std::clamp((int)std::floor(camera_noise * 10), 0, 9);
            net.setEvidence("camera_noise", camera_noise_state);
            ROS_INFO("%s: %.2f, %i", "camera_noise", camera_noise, camera_noise_state);
            debug_display.measured_camera_noise = camera_noise_state/10.0;
            }
            else{
                net.setEvidence("camera_noise", measurement_msg.camera_noise);
            ROS_INFO("%s: %i", "camera_noise", measurement_msg.camera_noise);
            debug_display.measured_camera_noise = measurement_msg.camera_noise/10.0;
            }
        }
        {
            if(measurement_msg.number_of_filtered_away_points<0){
            // Not a clear connection between worse conditions and increased number of points. 0 is defineatly good, and is 0 most of the time. 10 seems like a safe place to define as there being an undetectable obstacle close by
            auto number_of_filtered_points_state = std::clamp((int)std::floor(number_of_filtered_points * 10 / pars.measurement_conversion.max_number_of_filtered_points), 0, 9);
            net.setEvidence("number_of_filtered_away_points", number_of_filtered_points_state);
            ROS_INFO("%s: %i, %i", "number_of_filtered_away_points", number_of_filtered_points, number_of_filtered_points_state);
            debug_display.measured_lidar_flicker = number_of_filtered_points_state/10.0;
            }
            else{
                net.setEvidence("number_of_filtered_away_points", measurement_msg.number_of_filtered_away_points);
            ROS_INFO("%s: %i", "number_of_filtered_away_points", measurement_msg.number_of_filtered_away_points);
            debug_display.measured_lidar_flicker = measurement_msg.number_of_filtered_away_points/10.0;
            }
        }
    }

    double evaluate_risk(const map_stringkey<map_stringkey<double>> &output, const map_stringkey<int> &actions, double probability_of_unobservable_obs, bool do_print = false) const
    {
        double max_speed = actions.at("max_speed") / 9.0;
        double safety_margin = actions.at("safety_margin") / 9.0;

        double mean_frequency_of_motor_saturation_deviating_beyond_safety_margin = log_mean(output.at("frequency_of_motor_saturation_deviating_beyond_safety_margin"));
        double mean_frequency_of_turbulence_causing_deviation_beyond_safety_margin = log_mean(output.at("frequency_of_exceeding_safety_margin_due_to_turbulence"));
        double mean_frequency_of_breaking_distance_exceeding_safety_margin = log_mean(output.at("frequency_of_breaking_distance_exceeding_safety_margin"));

        double mean_frequency_of_contact_causing_loss_of_control =
            mean_frequency_of_motor_saturation_deviating_beyond_safety_margin * (pars.risk.motor_saturation.constant+ pars.risk.motor_saturation.scale * std::pow(max_speed, 2)) 
            + mean_frequency_of_turbulence_causing_deviation_beyond_safety_margin * (pars.risk.turbulence.constant + pars.risk.turbulence.scale * std::pow(max_speed, 2)) 
            + mean_frequency_of_breaking_distance_exceeding_safety_margin * (pars.risk.breaking_distance.constant 
                        + pars.risk.breaking_distance.scale * ((1-probability_of_unobservable_obs)*std::pow(max_speed-safety_margin/0.9, 2)
                        +probability_of_unobservable_obs*std::pow(max_speed, 2)));

        double mean_frequency_of_motor_wear_causing_loss_of_control = log_mean(output.at("frequency_of_loss_of_control_due_to_motor_wear"));
        double mean_frequency_of_loss_of_control_due_to_turbulence = log_mean(output.at("frequency_of_loss_of_control_due_to_turbulence"));

        double mean_frequency_of_loss_of_control = mean_frequency_of_contact_causing_loss_of_control + mean_frequency_of_motor_wear_causing_loss_of_control + mean_frequency_of_loss_of_control_due_to_turbulence;

        double mean_risk_damage_to_drone = mean_frequency_of_loss_of_control * (pars.risk.damage.constant + pars.risk.damage.scale * (height_over_ground/40));
        double mean_risk_loss_of_mission = mean_frequency_of_loss_of_control;
        double total_risk = pars.risk.total.damage_to_drone * mean_risk_damage_to_drone + pars.risk.total.loss_of_mission * mean_risk_loss_of_mission;

        if (do_print)
        {
            std::cout << "Mean freq of motor saturation deviating beyond: " << mean_frequency_of_motor_saturation_deviating_beyond_safety_margin
                      << "\nMean freq of turbulence deviation beyond: " << mean_frequency_of_turbulence_causing_deviation_beyond_safety_margin
                      << "\nMean freq of breaking dist exceeding: " << mean_frequency_of_breaking_distance_exceeding_safety_margin
                      << "\nMean freq of contact causing loss of control: " << mean_frequency_of_contact_causing_loss_of_control
                      << "\nMean freq of motor wear causing loss of control: " << mean_frequency_of_motor_wear_causing_loss_of_control
                      << "\nMean freq of turbulence causing loss of control: " << mean_frequency_of_loss_of_control_due_to_turbulence
                      << "\nMean freq of loss of control: " << mean_frequency_of_loss_of_control
                      << "\nRisk: " << total_risk << std::endl;
        }

        return total_risk;
    }

    /*bool evaluate_action_feasibility(const map_stringkey<int> &actions)
    {
        net.setEvidence(actions);
        auto output = net.evaluateStates(output_node_names);
        return evaluate_risk(output, actions) < pars.max_risk;
    }*/

    /*auto heuristic_search()
    {
        map_stringkey<int> action;
        action["safety_margin"] = 0; // OBS denne er definert feil vei, finn ut hva å gjøre med det
        action["max_upwards_acceleration"] = 0;
        action["max_speed"] = 0;

        std::vector<std::string> active_parameters = {"safety_margin", "max_upwards_acceleration", "max_speed"};
        map_stringkey<int> parameter_max;
        parameter_max["safety_margin"] = 9;
        parameter_max["max_upwards_acceleration"] = 4;
        parameter_max["max_speed"] = 9;
        bool search_failed = false;
        bool search_completed = false;
        while (active_parameters.size() > 0)
        {
            if (!evaluate_action_feasibility(action))
            {
                // If not feasible, go back to previous feasible value
                for (const auto &param : active_parameters)
                {
                    action.at(param)--;
                    if (action.at(param) < 0)
                    {
                        // there is not feasable solution, end the search;
                        action.at(param) = 0;
                        search_failed = true;
                        break;
                    }
                }
                if (search_failed)
                    break;
                // Find a parameter that cannot be improved
                bool did_erase_something = false;
                for (auto param = active_parameters.begin(); param != active_parameters.end(); ++param)
                {
                    action.at(*param)++;
                    bool feasible = evaluate_action_feasibility(action);
                    action.at(*param)--;
                    if (feasible == false)
                    {
                        active_parameters.erase(param);
                        // then continue the search without modifying this parameter
                        did_erase_something = true;
                        break;
                    }
                }
                if (!did_erase_something)
                {
                    active_parameters.pop_back();
                }
            }
            for (const auto &param : active_parameters)
            {
                action.at(param)++;
                if (action.at(param) > parameter_max.at(param))
                {
                    search_completed = true;
                    action.at(param) = parameter_max.at(param);
                    break;
                }
            }
            if (search_completed)
                break;
        }
        return action;
    }*/

    auto exact_search(double probability_of_unobservable_obs)
    {
        struct{
            map_stringkey<int> optimal_action;
            double found_feasable_solution=false;
        }ret;
        ret.optimal_action["safety_margin"] = 9;
        ret.optimal_action["max_upwards_acceleration"] = 0;
        ret.optimal_action["max_speed"] = 0;
        map_stringkey<int> action;
        double optimal_cost = INFINITY;

        double min_safety_margin = 0;
        double max_max_speed = 9;
        double max_speed_cost = 1;
        double safety_margin_cost = 1;
        if(last_set_target == last_set_target_type::speed){
            max_max_speed = max_speed_target;
            max_speed_cost = pars.cost_function_parameters.target_cost;
            safety_margin_cost = pars.cost_function_parameters.non_target_cost;
        }
        else if(last_set_target == last_set_target_type::margin){
            min_safety_margin = safety_margin_target;
            safety_margin_cost = pars.cost_function_parameters.target_cost;
            max_speed_cost = pars.cost_function_parameters.non_target_cost;
        }

        for (int safety_margin = min_safety_margin; safety_margin < 10; ++safety_margin)
        {
            action["safety_margin"] = safety_margin;
            for (int acceleration_limit = 0; acceleration_limit < 5; ++acceleration_limit)
            {
                action["max_upwards_acceleration"] = acceleration_limit;
                // max speed does not affect the network
                for (int max_speed = 0; max_speed <= max_max_speed; ++max_speed)
                {
                    action["max_speed"] = max_speed;
                    net.setEvidence(action);
                    auto output = net.evaluateStates(output_node_names);
                    double risk =  evaluate_risk(output, action,probability_of_unobservable_obs);
                    bool is_feasible = risk< pars.max_risk;
                    double parameter_cost = safety_margin_cost*std::pow(safety_margin, 1) 
                        + pars.cost_function_parameters.acceleration_cost * std::pow(4 - acceleration_limit, 1) 
                        + max_speed_cost * std::pow(9 - max_speed, 1);

                    double change_cost = std::pow(safety_margin - previous_action["safety_margin"], 2) + std::pow(acceleration_limit - previous_action["max_upwards_acceleration"], 2) + std::pow(max_speed - previous_action["max_speed"], 2);

                    double total_cost = parameter_cost + pars.cost_function_parameters.parameter_change_cost * change_cost;

                    if (is_feasible && total_cost < optimal_cost)
                    {
                        ret.optimal_action = action;
                        optimal_cost = total_cost;
                        ret.found_feasable_solution = true;
                    }
                }
            }
        }
        return ret;
    }

    void run()
    {
        try{

        set_observations();

        auto estimate_node_states = net.evaluateStates(all_estimate_node_names);
        {
            debug_display.mean_motor_wear = mean(estimate_node_states.at("motor_wear"));
            debug_display.mean_motoruse_for_whirlewind = mean(estimate_node_states.at("random_disturbance"));
            debug_display.mean_motoruse_for_tether = mean(estimate_node_states.at("motoruse_for_tether"));
            debug_display.mean_turbulence = mean(estimate_node_states.at("turbulence"));
            debug_display.mean_enviornment_observability = mean(estimate_node_states.at("environment_observability"));
            debug_display.presence_of_unobservable_obstacle = estimate_node_states.at("presence_of_unobservable_obstacle").at("State1");
            debug_display.mean_dust = mean(estimate_node_states.at("filtered_motor_use"));

            if(pars.spam_states){
                for(auto [node, states] : estimate_node_states){
                    std::cout <<  node << ": ";
                    for(auto [state, value] : states){
                        std::cout << state << ":" << value << ", ";
                    }
                    std::cout << std::endl;
                }
            }
        }

        // Worst case:
        /*{
        map_stringkey<int> action;
        action["safety_margin"]  = 9; //OBS denne er definert feil vei, finn ut hva å gjøre med det
        action["max_upwards_acceleration"]  = 4;
        action["max_speed"]  = 9;
        net.setEvidence(action);
        auto output = net.evaluateStates(all_prediction_node_names);
        double risk_cost = evaluate_risk(output,action,true);
        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f", action.at("safety_margin"), action.at("max_upwards_acceleration"), action.at("max_speed"), risk_cost);
        }*/

        // auto action = heuristic_search();
        auto search_result = exact_search(estimate_node_states.at("presence_of_unobservable_obstacle").at("State1"));

        {
            double a  = pars.cost_function_parameters.parameter_change_delay_factor;
            if(previous_max_speed < search_result.optimal_action.at("max_speed")){
                previous_max_speed = (1-a)*previous_max_speed+a*search_result.optimal_action.at("max_speed");
                search_result.optimal_action.at("max_speed") = (int)round(previous_max_speed);  
            }
            else
                previous_max_speed = search_result.optimal_action.at("max_speed");

            if(previous_safety_margin > search_result.optimal_action.at("safety_margin")){
                previous_safety_margin = (1-a)*previous_safety_margin+a*search_result.optimal_action.at("safety_margin");
                search_result.optimal_action.at("safety_margin") = (int)round(previous_safety_margin);  
            }
            else
                previous_safety_margin = search_result.optimal_action.at("safety_margin");

            if(previous_max_acc < search_result.optimal_action.at("max_upwards_acceleration")){
                previous_max_acc = (1-a)*previous_max_acc+a*search_result.optimal_action.at("max_upwards_acceleration");
                search_result.optimal_action.at("max_upwards_acceleration") = (int)round(previous_max_acc);  
            }
            else
                previous_max_acc = search_result.optimal_action.at("max_upwards_acceleration");
        }

        {
            net.setEvidence(search_result.optimal_action);
            auto output = net.evaluateStates(all_prediction_node_names);
            debug_display.risk_cost = evaluate_risk(output, search_result.optimal_action, true);
            debug_display.optimal_safety_margin = search_result.optimal_action.at("safety_margin");
            debug_display.optimal_max_acc = search_result.optimal_action.at("max_upwards_acceleration");
            debug_display.optimal_speed = search_result.optimal_action.at("max_speed");
            debug_display.found_feasible_option = search_result.found_feasable_solution;

            debug_display.mean_frequency_of_motor_saturation_deviating_beyond_safety_margin = log_mean(output.at("frequency_of_motor_saturation_deviating_beyond_safety_margin"));
            debug_display.mean_frequency_of_loss_of_control_due_to_motor_wear = log_mean(output.at("frequency_of_loss_of_control_due_to_motor_wear"));
            debug_display.mean_frequency_of_loss_of_control_due_to_turbulence = log_mean(output.at("frequency_of_loss_of_control_due_to_turbulence"));
            debug_display.mean_frequency_of_exceeding_safety_margin_due_to_turbulence = log_mean(output.at("frequency_of_exceeding_safety_margin_due_to_turbulence"));
            debug_display.mean_frequency_of_breaking_distance_exceeding_safety_margin = log_mean(output.at("frequency_of_breaking_distance_exceeding_safety_margin"));

            
            debug_display_publisher.publish(debug_display);
        }

        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f", search_result.optimal_action.at("safety_margin"), search_result.optimal_action.at("max_upwards_acceleration"), search_result.optimal_action.at("max_speed"), debug_display.risk_cost);
        
        previous_action = search_result.optimal_action;

        /*auto exact_action = exact_search();
        net.setEvidence(exact_action);
        output = net.evaluateStates(all_prediction_node_names);
        double risk_cost = evaluate_risk(output,exact_action,true);
        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f", exact_action.at("safety_margin"), exact_action.at("max_upwards_acceleration"), exact_action.at("max_speed"), risk_cost);
        */
       

        if(pars.modify_parameters){

            auto param_set = mavros_msgs::ParamSet{};
            //TODO: tune inn verdiene her
            param_set.request.param_id = "SA_DISTANCE";
            param_set.request.value.real = search_result.optimal_action.at("safety_margin")*(1.09-0.1)/9.0+(0.1+0.21)/2+0.31;
            if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
                throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};

            param_set.request.param_id = "MPC_VEL_MANUAL";
            param_set.request.value.real = search_result.optimal_action.at("max_speed")*(1.36-0.1)/9+(0.24+0.1)/2;
            if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
                throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};

            param_set.request.param_id = "MPC_ACC_DOWN_MAX";
            param_set.request.value.real = search_result.optimal_action.at("max_upwards_acceleration")*(1.7-0.5)/4+(0.5+0.8)/2;
            if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
                throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};
            param_set.request.param_id = "MPC_ACC_UP_MAX";
            if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
                throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};
        }


        if(pars.dynamic) net.incrementTime1_5DBN(causal_node_names, pars.saving_factor);

        }
        catch(const std::string& str)
        {
            ROS_ERROR("Crashed with string: %s", str.c_str());
        }
    }

    ros::Subscriber try_out_actions = nh.subscribe<supervisory_risk_control_msgs::actions>("/supervisor/test_action_set",1, [&](supervisory_risk_control_msgs::actionsConstPtr msg){
        std::map<std::string,int> action ={{"safety_margin", msg->safety_margin},{"max_upwards_acceleration", msg->max_acceleartion},{"max_speed",msg->max_speed}};
        net.setEvidence(action);
        auto output = net.evaluateStates(output_node_names);
        auto risk = evaluate_risk(output, action, true);
        ROS_INFO("Testing action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f", action.at("safety_margin"), action.at("max_upwards_acceleration"), action.at("max_speed"),  risk);
    });

public:
    SupervisoryRiskControl()
    {
        nhp.getParam("dynamic", pars.dynamic);
        nhp.getParam("modify_parameters", pars.modify_parameters);
        nhp.getParam("only_update_on_measurements", pars.only_update_on_measurements);
        nhp.getParam("saving_factor", pars.saving_factor);
        nhp.getParam("spam_states", pars.spam_states);
        nhp.getParam("no_delay", pars.no_delay);
        nhp.getParam("max_risk", pars.max_risk);
        nhp.getParam("measurement_conversion/max_yaw_moment", pars.measurement_conversion.max_yaw_moment);
        nhp.getParam("measurement_conversion/max_turbulence", pars.measurement_conversion.max_turbulence);
        nhp.getParam("measurement_conversion/min_turbulence", pars.measurement_conversion.min_turbulence);
        nhp.getParam("measurement_conversion/max_number_of_filtered_points", pars.measurement_conversion.max_number_of_filtered_points);
        nhp.getParam("measurement_conversion/max_tilt", pars.measurement_conversion.max_tilt);
        nhp.getParam("measurement_conversion/min_tilt", pars.measurement_conversion.min_tilt);
        nhp.getParam("measurement_conversion/motor_max", pars.measurement_conversion.motor_max);
        nhp.getParam("measurement_conversion/motor_min", pars.measurement_conversion.motor_min);
        nhp.getParam("measurement_conversion/filtered_point_lowpass_factor", pars.measurement_conversion.filtered_point_lowpass_factor);
        nhp.getParam("risk/motor_saturation/constant", pars.risk.motor_saturation.constant);
        nhp.getParam("risk/motor_saturation/scale", pars.risk.motor_saturation.scale);
        nhp.getParam("risk/turbulence/constant", pars.risk.turbulence.constant);
        nhp.getParam("risk/turbulence/scale", pars.risk.turbulence.scale);
        nhp.getParam("risk/breaking_distance/constant", pars.risk.breaking_distance.constant);
        nhp.getParam("risk/breaking_distance/scale", pars.risk.breaking_distance.scale);
        nhp.getParam("risk/damage/constant", pars.risk.damage.constant);
        nhp.getParam("risk/damage/scale", pars.risk.damage.scale);
        nhp.getParam("risk/total/damage_to_drone", pars.risk.total.damage_to_drone);
        nhp.getParam("risk/total/loss_of_mission", pars.risk.total.loss_of_mission);
        nhp.getParam("cost_function_parameters/target_cost", pars.cost_function_parameters.target_cost);
        nhp.getParam("cost_function_parameters/non_target_cost", pars.cost_function_parameters.non_target_cost);
        nhp.getParam("cost_function_parameters/acceleration_cost", pars.cost_function_parameters.acceleration_cost);
        nhp.getParam("cost_function_parameters/parameter_change_cost", pars.cost_function_parameters.parameter_change_cost);
        nhp.getParam("cost_function_parameters/parameter_change_delay_factor", pars.cost_function_parameters.parameter_change_delay_factor);
        

        measurement_msg.camera_noise = -1;
        measurement_msg.drone_tilt = -1;
        measurement_msg.height_over_ground = -1;
        measurement_msg.motor_mean = -1;
        measurement_msg.number_of_filtered_away_points = -1;
        measurement_msg.roll_pitch_deviation = -1;

        std::string filename = pars.dynamic ? "net_dynamic.xdsl" : "net.xdsl";
        net.init(ros::package::getPath("supervisory_risk_control") + "/include/supervisory_risk_control/"+filename);

        ros::Rate r(1);

        while(ros::ok()){
            ros::spinOnce();
            if(do_abort) return;
            if(!pars.only_update_on_measurements || new_data){
                run();
                new_data = false;
            }
            if(!pars.no_delay)
                r.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisory_risk_control");

    SupervisoryRiskControl node;

    return 0;
}