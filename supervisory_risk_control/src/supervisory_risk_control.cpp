#include "mavros_msgs/DebugValue.h"
#include "supervisory_risk_control/bayesian_network.h"
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

    BayesianNetwork net{ros::package::getPath("supervisory_risk_control") + "/include/supervisory_risk_control/net.xdsl"};

    ros::Subscriber debug_value_subscriber = nh.subscribe("/mavros/debug_value/debug_float_array", 1, &SupervisoryRiskControl::debug_value_CB, this);
    ros::Publisher debug_display_publisher = nh.advertise<supervisory_risk_control_msgs::debug_display>("supervisory_risk_controller/debug_msg",1);
    supervisory_risk_control_msgs::debug_display debug_display;

    const std::vector<std::string> output_node_names = {"frequency_of_motor_saturation_causing_loss_of_control"};
    const std::vector<std::string> causal_node_names = {"friction", "other_causes","turbulence"};
    const std::vector<std::string> intermediate_estimation_node_names = {"current_drag","joystick_motor_use"};
    const std::vector<std::string> intermediate_binary_prediction_node_names = {"motor_saturation_caused_deviation_beyond_safety_margin","hitting_obstacle_causes_loss_of_control"};
    const std::vector<std::string> intermediate_linear_prediction_node_names = {"drag","compond_motoruse_badness","deviation_when_motor_saturate"};
    const std::vector<std::string> intermediate_log_prediction_node_names = {"expected_frequency_of_motor_saturation","frequency_of_motor_saturation_causing_loss_of_control","frequency_of_hitting_obstacles","frequency_of_motor_saturation_exceeding_safety_margin","expected_frequency_of_disturbance_causing_deviation_beyond_safety_margin"};
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

    void set_observations(mavros_msgs::DebugValueConstPtr msg){
        //TODO tune inn verdiene:

        auto& motor_use = msg->data[topic_indices::motor_max];
        auto motor_use_state = std::clamp((int)std::floor(motor_use*10),0,9);
        net.setEvidence("measured_motoruse", motor_use_state);
        ROS_INFO("%s: %.2f, %i", "measured_motoruse", motor_use, motor_use_state);
        debug_display.measured_motoruse = motor_use_state;

        auto & js_derivative = msg->data[topic_indices::js_derivative];
        auto js_derivative_state = std::clamp((int)std::floor(js_derivative*10),0,9);
        net.setEvidence("current_change_in_joystick_input", js_derivative_state);
        ROS_INFO("%s: %.2f, %i", "current_change_in_joystick_input", js_derivative, js_derivative_state);
        debug_display.measured_current_change_in_joystick_input = js_derivative_state;

        auto & acceleration_limit = msg->data[topic_indices::acceleration_limit];
        auto acceleration_limit_state = std::clamp((int)std::floor(acceleration_limit*10),0,9);
        net.setEvidence("current_acceleration_limit", js_derivative_state);
        ROS_INFO("%s: %.2f, %i", "current_acceleration_limit", acceleration_limit, acceleration_limit_state);
        debug_display.measured_current_acceleration_limit = acceleration_limit_state;

        auto & speed_setpoint = msg->data[topic_indices::speed_setpoint];
        auto speed_setpoint_state = std::clamp((int)std::floor(speed_setpoint*10),0,9);
        net.setEvidence("current_speed_setpoint", speed_setpoint_state);
        ROS_INFO("%s: %.2f, %i", "current_speed_setpoint",speed_setpoint, speed_setpoint_state);
        debug_display.measured_current_speed_setpoint = speed_setpoint_state;

        auto & velocity_deviation = msg->data[topic_indices::vxvy_error_component];
        auto velocity_deviation_state = std::clamp((int)std::floor(velocity_deviation*10),0,9);
        net.setEvidence("velocity_deviation", velocity_deviation_state);
        ROS_INFO("%s: %.2f, %i", "velocity_deviation", velocity_deviation, velocity_deviation_state);
        debug_display.measured_velocity_deviation = velocity_deviation_state;

        /*auto & height_over_ground = msg->data[topic_indices::height_over_ground];
        auto height_over_ground_state = std::clamp((int)std::floor(height_over_ground*10),0,9);
        net.setEvidence("height_over_ground", height_over_ground_state);
        ROS_INFO("%s: %.2f, %i", "height_over_ground", height_over_ground, height_over_ground_state);
        debug_display.measured_height_over_ground = height_over_ground_state;*/
    }

    void debug_value_CB(mavros_msgs::DebugValueConstPtr msg)
    {
        ROS_INFO("\n\n\nNew measurement");
        set_observations(msg);

        {
        auto output = net.evaluateStates(all_estimate_node_names);
        debug_display.mean_turbulence = mean(output.at("turbulence"));
        debug_display.mean_other_motoruse = mean(output.at("other_causes"));
        debug_display.mean_friction = mean(output.at("friction"));
        debug_display.mean_current_drag = mean(output.at("current_drag"));
        debug_display.mean_joystick_motor_use = mean(output.at("joystick_motor_use"));
        }

        double lowest_cost = INFINITY;
        int best_margin = -1;
        int best_acceleration_limit = -1;
        int best_max_speed = -1;
        double best_risk_cost = -1;

        std::map<std::string, int> action;
        for(int safety_margin = 0; safety_margin < 10; safety_margin+=1){
            action["safety_margin"] =  safety_margin;
            for(int acceleration_limit = 1; acceleration_limit < 10; ++acceleration_limit+=1){
                action["acceleration_limit"] =  acceleration_limit;
                for(int max_speed = 1; max_speed < 10; ++max_speed+=1){
                    action["max_speed_setpoint"] =  max_speed;

                    net.setEvidence(action);
                    auto output = net.evaluateStates(output_node_names);
                    auto risk_cost = 0;//output.at(output_node_names[0]).at("p1000min");
                    std::vector<std::string> state_names = {"p1000min","p0500min","p0200min","p0100min","p0050min","p0020min","p0010min","p0005min","p0002min","p0001min"};
                    std::vector<double> state_costs = {1,2,5,10,20,50,100,200,500,1000};
                    double consequence = 7+msg->data[topic_indices::height_over_ground];//consequence increases with height, but there is always a chance for large problems no matter how low
                    for(int i=0;i<10;++i){
                        risk_cost += consequence * state_costs[i] * output.at(output_node_names[0]).at(state_names[i]);
                    }

                    static constexpr double safety_margin_penalty = 1;
                    static constexpr double max_acceleration_penalty = 1;
                    static constexpr double speed_penalty = 1;
                    static constexpr double risk_limit = 400;

                    const double safety_margin_cost = std::pow(safety_margin,4.0)*safety_margin_penalty;
                    const double max_acc_cost = std::pow(9-acceleration_limit,4.0)*max_acceleration_penalty;
                    const double speed_cost = std::pow(9-max_speed,4.0)*speed_penalty;

                    const double cost = safety_margin_cost+max_acc_cost+speed_cost;

                    if(cost<lowest_cost){
                        lowest_cost = cost;
                        best_margin = safety_margin;
                        best_acceleration_limit = acceleration_limit;
                        best_max_speed = max_speed;
                        best_risk_cost= risk_cost;
                    }
                }
            }
        }
        debug_display.risk_cost = best_risk_cost;
        debug_display.optimal_safety_margin = best_margin;
        debug_display.optimal_max_acc = best_acceleration_limit;
        debug_display.optimal_speed = best_max_speed;
        action["safety_margin"] =  best_margin;
        action["acceleration_limit"] =  best_acceleration_limit;
        action["max_speed_setpoint"] =  best_max_speed;
        {
        net.setEvidence(action);
        auto output = net.evaluateStates(all_prediction_node_names);
        debug_display.motor_saturation_caused_deviation_beyond_safety_margin = output.at("motor_saturation_caused_deviation_beyond_safety_margin").at("true");
        debug_display.hitting_obstacle_causes_loss_of_control = output.at("hitting_obstacle_causes_loss_of_control").at("true");
        
        debug_display.mean_predicted_drag = mean(output.at("drag"));
        debug_display.mean_predicted_compond_motoruse_badness = mean(output.at("compond_motoruse_badness"));
        debug_display.mean_predicted_deviation_when_motors_saturate = mean(output.at("deviation_when_motor_saturate"));

        debug_display.mean_frequency_of_motor_saturation = log_mean(output.at("expected_frequency_of_motor_saturation"));
        debug_display.mean_frequency_of_motor_saturation_causing_loss_of_control = log_mean(output.at("frequency_of_motor_saturation_causing_loss_of_control"));
        debug_display.mean_frequency_of_motor_saturation_exceeding_safety_margin = log_mean(output.at("frequency_of_motor_saturation_exceeding_safety_margin"));
        debug_display.mean_frequency_of_hitting_obstacles = log_mean(output.at("frequency_of_hitting_obstacles"));
        debug_display.mean_frequency_of_disturbance_causing_deviation_beyond_safety_margin = log_mean(output.at("expected_frequency_of_disturbance_causing_deviation_beyond_safety_margin"));
        }

        ROS_INFO("Optimal action - Margin: %i, Acc: %i, Speed: %i. Risk_cost: %f, lowest_cost: %0.1f", best_margin, best_acceleration_limit, best_max_speed, best_risk_cost, lowest_cost);
        //ROS_INFO("Optimal action - Margin: %i, Acc: %i. Cost: %.2f, Output prob: %.2f", best_margin, best_acceleration_limit, lowest_cost);
        //Publiser på en ros topic ønkset hastighet, margin, etc, så man kan se på dem i plotjuggler 

        /*auto param_set = mavros_msgs::ParamSet{};
        param_set.request.param_id = "SA_DISTANCE";
        param_set.request.value.real = best_margin;

        if (!ros::service::call("/mavros/param/set", param_set) ||
        (param_set.response.success == 0u))
        throw "Failed to write PX4 parameter " + param_set.request.param_id;*/

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