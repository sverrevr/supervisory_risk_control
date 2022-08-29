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
    int number_of_filtered_points = 0;
    double camera_noise = 0;
    double drone_tilt = 0;
    int max_speed_target = 9;
    int safety_margin_target = 0;
    enum class last_set_target_type {none, speed, margin} last_set_target = last_set_target_type::none;
    bool new_data = true;
    supervisory_risk_control_msgs::measurements measurement_msg;
    bool use_direct_messages = false;

    struct{
        bool dynamic;
        bool only_update_on_measurements;
        float saving_factor;
        double max_risk;
        struct{
            double max_yaw_moment, max_turbulence, max_number_of_filtered_points, motor_max, motor_min, max_tilt, min_tilt;
        } measurement_conversion;
        struct{
            struct {
                double constant, scale;
            } motor_saturation, turbulence, unobservable_obs, breaking_distance, damage;
            struct{
               double damage_to_drone, loss_of_mission;
            } total;
        } risk;

        struct{
            double target_cost, non_target_cost, acceleration_cost, parameter_change_cost;
        } cost_function_parameters;
    } pars;

    const std::vector<std::string> causal_node_names = {"tether_tension_motor_use_scaling_factor", "motor_wear", "random_disturbance", "turbulence", "dust", "enviornment_observability"};

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
                                                                                         { //number_of_filtered_points = 0.9 * number_of_filtered_points + 0.1 * msg->data; 
                                                                                         number_of_filtered_points=msg->data;});
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

    ros::Subscriber set_measurements_subscriber = nh.subscribe<supervisory_risk_control_msgs::measurements>("/supervisor/set_measurements", 1, [&](supervisory_risk_control_msgs::measurementsConstPtr msg)
    {
        measurement_msg = *msg;
        use_direct_messages = true;
        new_data = true;
    });

    ros::Subscriber disable_direct_measurements = nh.subscribe<std_msgs::Int32>("/suervisor/disable_direct_measurements",1,[&](std_msgs::Int32ConstPtr msg){
        use_direct_messages = false;
    });

    ros::Publisher debug_display_publisher = nh.advertise<supervisory_risk_control_msgs::debug_display>("supervisory_risk_controller/debug_msg", 1);
    supervisory_risk_control_msgs::debug_display debug_display;

    map_stringkey<int> previous_action{std::map<std::string,int>{{"max_speed",9}, {"safety_margin",0}, {"max_upwards_acceleration",4}}};

    const std::vector<std::string> output_node_names = {"frequency_of_motor_saturation_deviating_beyond_safety_margin", "frequency_of_loss_of_control_due_to_motor_wear", "frequency_of_exceeding_safety_margin_due_to_turbulence", "frequency_of_contact_with_unobservable_obstacle", "frequency_of_breaking_distance_exceeding_safety_margin", "frequency_of_loss_of_control_due_to_turbulence"};
    const std::vector<std::string> intermediate_estimation_node_names = {
        "motoruse_for_tether",
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
            // motor saturation is at 1949
            auto motor_use_state = std::clamp((int)std::floor(10*(motor_use-pars.measurement_conversion.motor_min)/(pars.measurement_conversion.motor_max-pars.measurement_conversion.motor_min)), 0, 9);
            net.setEvidence("mean_motor", motor_use_state);
            ROS_INFO("%s: %.2f, %i", "mean_motor", motor_use, motor_use_state);
            debug_display.measured_motoruse = motor_use_state/10.0;
        }
        {
            // Max roll/pitch
            auto drone_tilt_state = std::clamp((int)std::floor((drone_tilt-pars.measurement_conversion.min_tilt) * 10 / pars.measurement_conversion.max_tilt), 0, 9);
            net.setEvidence("drone_tilt", drone_tilt_state);
            ROS_INFO("%s: %.2f, %i", "drone_tilt", drone_tilt, drone_tilt_state);
            debug_display.measured_drone_roll_pitch = drone_tilt_state/10.0;
        }
        {
            /*auto yaw_moment_state = std::clamp((int)std::floor(yaw_moment * 10/pars.measurement_conversion.max_yaw_moment), 0, 9);
            net.setEvidence("yaw_moment", yaw_moment_state);
            ROS_INFO("%s: %.2f, %i", "yaw_moment", yaw_moment, yaw_moment_state);
            debug_display.measured_yaw_moment = yaw_moment_state/10.0;*/
        }
        {
            // max height is 40m
            auto height_over_ground_state = std::clamp((int)std::floor(height_over_ground * 10 / 40), 0, 9);
            net.setEvidence("height_over_ground", height_over_ground_state);
            ROS_INFO("%s: %.2f, %i", "height_over_ground", height_over_ground, height_over_ground_state);
            debug_display.measured_height_over_ground = height_over_ground_state/10.0;
        }
        {
            // High turbulence area has measurements of 0.004-0.007, define max to be slightly higher as there might be even worse cases.
            auto velocity_deviation_state = std::clamp((int)std::floor(roll_pitch_deviation * 10 / pars.measurement_conversion.max_turbulence), 0, 9);
            net.setEvidence("roll_pitch_deviation", velocity_deviation_state);
            ROS_INFO("%s: %.2f, %i", "roll_pitch_deviation", roll_pitch_deviation, velocity_deviation_state);
            debug_display.measured_velocity_deviation = velocity_deviation_state/10.0;
        }
        {
            // Human input of value between 0 and 1
            auto camera_noise_state = std::clamp((int)std::floor(camera_noise * 10), 0, 9);
            net.setEvidence("camera_noise", camera_noise_state);
            ROS_INFO("%s: %.2f, %i", "camera_noise", camera_noise, camera_noise_state);
            debug_display.measured_camera_noise = camera_noise_state/10.0;
        }
        {
            // Not a clear connection between worse conditions and increased number of points. 0 is defineatly good, and is 0 most of the time. 10 seems like a safe place to define as there being an undetectable obstacle close by
            auto number_of_filtered_points_state = std::clamp((int)std::floor(number_of_filtered_points * 10 / pars.measurement_conversion.max_number_of_filtered_points), 0, 9);
            net.setEvidence("number_of_filtered_away_points", number_of_filtered_points_state);
            ROS_INFO("%s: %i, %i", "number_of_filtered_away_points", number_of_filtered_points, number_of_filtered_points_state);
            debug_display.measured_lidar_flicker = number_of_filtered_points_state/10.0;
        }
    }

    void set_observations_from_direct_measurements(){
        {
            // motor saturation is at 1949
            net.setEvidence("mean_motor", measurement_msg.motor_mean);
            ROS_INFO("%s: %i", "mean_motor", measurement_msg.motor_mean);
            debug_display.measured_motoruse = measurement_msg.motor_mean/10.0;
        }
        {
            // Max roll/pitch
            net.setEvidence("drone_tilt", measurement_msg.drone_tilt);
            ROS_INFO("%s: %i", "drone_tilt", measurement_msg.drone_tilt);
            debug_display.measured_drone_roll_pitch = measurement_msg.drone_tilt/10.0;
        }
        {
            /*net.setEvidence("yaw_moment", measurement_msg.yaw_moment);
            ROS_INFO("%s: %i", "yaw_moment", measurement_msg.yaw_moment);
            debug_display.measured_yaw_moment = measurement_msg.yaw_moment/10.0;*/
        }
        {
            // max height is 40m
            net.setEvidence("height_over_ground", measurement_msg.height_over_ground);
            ROS_INFO("%s: %i", "height_over_ground", measurement_msg.height_over_ground);
            debug_display.measured_height_over_ground = measurement_msg.height_over_ground/10.0;
        }
        {
            // High turbulence area has measurements of 0.004-0.007, define max to be slightly higher as there might be even worse cases.
            net.setEvidence("roll_pitch_deviation", measurement_msg.roll_pitch_deviation);
            ROS_INFO("%s: %i", "roll_pitch_deviation", measurement_msg.roll_pitch_deviation);
            debug_display.measured_velocity_deviation = measurement_msg.roll_pitch_deviation/10.0;
        }
        {
            // Human input of value between 0 and 1
            net.setEvidence("camera_noise", measurement_msg.camera_noise);
            ROS_INFO("%s: %i", "camera_noise", measurement_msg.camera_noise);
            debug_display.measured_camera_noise = measurement_msg.camera_noise/10.0;
        }
        {
            // Not a clear connection between worse conditions and increased number of points. 0 is defineatly good, and is 0 most of the time. 10 seems like a safe place to define as there being an undetectable obstacle close by
            net.setEvidence("number_of_filtered_away_points", measurement_msg.number_of_filtered_away_points);
            ROS_INFO("%s: %i", "number_of_filtered_away_points", measurement_msg.number_of_filtered_away_points);
            debug_display.measured_lidar_flicker = measurement_msg.number_of_filtered_away_points/10.0;
        }
    }

    double evaluate_risk(const map_stringkey<map_stringkey<double>> &output, const map_stringkey<int> &actions, bool do_print = false) const
    {
        double max_speed = actions.at("max_speed") / 9.0;
        double safety_margin = actions.at("safety_margin") / 9.0;

        double mean_frequency_of_motor_saturation_deviating_beyond_safety_margin = log_mean(output.at("frequency_of_motor_saturation_deviating_beyond_safety_margin"));
        double mean_frequency_of_turbulence_causing_deviation_beyond_safety_margin = log_mean(output.at("frequency_of_exceeding_safety_margin_due_to_turbulence"));
        double mean_frequency_of_contact_with_unobservable_obstacle = log_mean(output.at("frequency_of_contact_with_unobservable_obstacle"));
        double mean_frequency_of_breaking_distance_exceeding_safety_margin = log_mean(output.at("frequency_of_breaking_distance_exceeding_safety_margin"));

        double mean_frequency_of_contact_causing_loss_of_control =
            mean_frequency_of_motor_saturation_deviating_beyond_safety_margin * (pars.risk.motor_saturation.constant+ pars.risk.motor_saturation.scale * std::pow(max_speed, 2)) 
            + mean_frequency_of_turbulence_causing_deviation_beyond_safety_margin * (pars.risk.turbulence.constant + pars.risk.turbulence.scale * std::pow(max_speed, 2)) 
            + mean_frequency_of_contact_with_unobservable_obstacle * (pars.risk.unobservable_obs.constant + pars.risk.unobservable_obs.scale * std::pow(max_speed, 2))
            + mean_frequency_of_breaking_distance_exceeding_safety_margin * (pars.risk.breaking_distance.constant + pars.risk.breaking_distance.scale * std::pow(max_speed, 2));

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
                      << "\nMean freq of contact with unobs obst: " << mean_frequency_of_contact_with_unobservable_obstacle
                      << "\nMean freq of breaking dist exceeding: " << mean_frequency_of_breaking_distance_exceeding_safety_margin
                      << "\nMean freq of contact causing loss of control: " << mean_frequency_of_contact_causing_loss_of_control
                      << "\nMean freq of motor wear causing loss of control: " << mean_frequency_of_motor_wear_causing_loss_of_control
                      << "\nMean freq of turbulence causing loss of control: " << mean_frequency_of_loss_of_control_due_to_turbulence
                      << "\nMean freq of loss of control: " << mean_frequency_of_loss_of_control
                      << "\nRisk: " << total_risk << std::endl;
        }

        return total_risk;
    }

    bool evaluate_action_feasibility(const map_stringkey<int> &actions)
    {
        net.setEvidence(actions);
        auto output = net.evaluateStates(output_node_names);
        return evaluate_risk(output, actions) < pars.max_risk;
    }

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

    auto exact_search()
    {
        map_stringkey<int> optimal_action;
        optimal_action["safety_margin"] = 9;
        optimal_action["max_upwards_acceleration"] = 0;
        optimal_action["max_speed"] = 0;
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
                    double risk =  evaluate_risk(output, action);
                    bool is_feasible = risk< pars.max_risk;
                    double parameter_cost = safety_margin_cost*std::pow(safety_margin, 1) 
                        + pars.cost_function_parameters.acceleration_cost * std::pow(4 - acceleration_limit, 1) 
                        + max_speed_cost * std::pow(9 - max_speed, 1);

                    double change_cost = std::pow(safety_margin - previous_action["safety_margin"], 2) + std::pow(acceleration_limit - previous_action["max_upwards_acceleration"], 2) + std::pow(max_speed - previous_action["max_speed"], 2);

                    double total_cost = parameter_cost + pars.cost_function_parameters.parameter_change_cost * change_cost;

                    if (is_feasible && total_cost < optimal_cost)
                    {
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
        if(use_direct_messages){
            set_observations_from_direct_measurements();
        }
        else{
            set_observations();
        }

        {
            auto output = net.evaluateStates(all_estimate_node_names);
            debug_display.mean_tether_tension_motor_use_scaling_factor = mean(output.at("tether_tension_motor_use_scaling_factor"));
            debug_display.mean_motor_wear = mean(output.at("motor_wear"));
            debug_display.mean_motoruse_for_whirlewind = mean(output.at("random_disturbance"));
            debug_display.mean_motoruse_for_tether = mean(output.at("motoruse_for_tether"));
            debug_display.mean_turbulence = mean(output.at("turbulence"));
            debug_display.mean_enviornment_observability = mean(output.at("enviornment_observability"));
            debug_display.mean_dust = mean(output.at("dust"));
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
        auto action = exact_search();
        
        {
            net.setEvidence(action);
            auto output = net.evaluateStates(all_prediction_node_names);
            debug_display.risk_cost = evaluate_risk(output, action, true);
            debug_display.optimal_safety_margin = action.at("safety_margin");
            debug_display.optimal_max_acc = action.at("max_upwards_acceleration");
            debug_display.optimal_speed = action.at("max_speed");
            debug_display.found_feasible_option = debug_display.risk_cost < pars.max_risk;

            debug_display.mean_frequency_of_motor_saturation_deviating_beyond_safety_margin = log_mean(output.at("frequency_of_motor_saturation_deviating_beyond_safety_margin"));
            debug_display.mean_frequency_of_loss_of_control_due_to_motor_wear = log_mean(output.at("frequency_of_loss_of_control_due_to_motor_wear"));
            debug_display.mean_frequency_of_loss_of_control_due_to_turbulence = log_mean(output.at("frequency_of_loss_of_control_due_to_turbulence"));
            debug_display.mean_frequency_of_exceeding_safety_margin_due_to_turbulence = log_mean(output.at("frequency_of_exceeding_safety_margin_due_to_turbulence"));
            debug_display.mean_frequency_of_contact_with_unobservable_obstacle = log_mean(output.at("frequency_of_contact_with_unobservable_obstacle"));
            debug_display.mean_frequency_of_breaking_distance_exceeding_safety_margin = log_mean(output.at("frequency_of_breaking_distance_exceeding_safety_margin"));
            
            debug_display_publisher.publish(debug_display);
        }

        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f", action.at("safety_margin"), action.at("max_upwards_acceleration"), action.at("max_speed"), debug_display.risk_cost);
        
        previous_action = action;

        /*auto exact_action = exact_search();
        net.setEvidence(exact_action);
        output = net.evaluateStates(all_prediction_node_names);
        double risk_cost = evaluate_risk(output,exact_action,true);
        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f", exact_action.at("safety_margin"), exact_action.at("max_upwards_acceleration"), exact_action.at("max_speed"), risk_cost);
        */

        /*auto param_set = mavros_msgs::ParamSet{};
        //TODO: tune inn verdiene her
        param_set.request.param_id = "SA_DISTANCE";
        param_set.request.value.real = action.at("safety_margin")*0.3+0.3;
        if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
            throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};

        param_set.request.param_id = "MPC_XY_VEL_MAX";
        param_set.request.value.real = action.at("max_speed")*0.29+0.1;
        if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
            throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};

        param_set.request.param_id = "MPC_ACC_DOWN_MAX";
        param_set.request.value.real = action.at("max_upwards_acceleration")*0.2+0.1;
        if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
            throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};
        param_set.request.param_id = "MPC_ACC_UP_MAX";
        param_set.request.value.real = action.at("max_upwards_acceleration")*0.2+0.1;
        if (!ros::service::call("/mavros/param/set", param_set) || (param_set.response.success == 0u))
            throw std::string{"Failed to write PX4 parameter " + param_set.request.param_id};*/

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
        nhp.getParam("only_update_on_measurements", pars.only_update_on_measurements);
        nhp.getParam("saving_factor", pars.saving_factor);
        nhp.getParam("max_risk", pars.max_risk);
        nhp.getParam("measurement_conversion/max_yaw_moment", pars.measurement_conversion.max_yaw_moment);
        nhp.getParam("measurement_conversion/max_turbulence", pars.measurement_conversion.max_turbulence);
        nhp.getParam("measurement_conversion/max_number_of_filtered_points", pars.measurement_conversion.max_number_of_filtered_points);
        nhp.getParam("measurement_conversion/max_tilt", pars.measurement_conversion.max_tilt);
        nhp.getParam("measurement_conversion/min_tilt", pars.measurement_conversion.min_tilt);
        nhp.getParam("measurement_conversion/motor_max", pars.measurement_conversion.motor_max);
        nhp.getParam("measurement_conversion/motor_min", pars.measurement_conversion.motor_min);
        nhp.getParam("risk/motor_saturation/constant", pars.risk.motor_saturation.constant);
        nhp.getParam("risk/motor_saturation/scale", pars.risk.motor_saturation.scale);
        nhp.getParam("risk/turbulence/constant", pars.risk.turbulence.constant);
        nhp.getParam("risk/turbulence/scale", pars.risk.turbulence.scale);
        nhp.getParam("risk/unobservable_obs/constant", pars.risk.unobservable_obs.constant);
        nhp.getParam("risk/unobservable_obs/scale", pars.risk.unobservable_obs.scale);
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

        std::string filename = pars.dynamic ? "net_dynamic.xdsl" : "net.xdsl";
        net.init(ros::package::getPath("supervisory_risk_control") + "/include/supervisory_risk_control/"+filename);

        while(ros::ok()){
            ros::spinOnce();
            if(!pars.only_update_on_measurements || new_data){
                run();
                new_data = false;
            }
            ros::Duration(1).sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisory_risk_control");

    SupervisoryRiskControl node;

    return 0;
}