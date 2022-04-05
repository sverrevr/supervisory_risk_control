#pragma once
#include <cmath>
#include <algorithm>

int motor_use_clasifier(double val){
    return std::clamp((int)std::floor(val*10),0,9);
}

int js_derivative_clasifier(double val){
    return std::clamp((int)std::floor(val*10),0,9);
}

int loss_of_control_authority_classifier(double val){
    return val>0.5 ? 1: 0;
}

int velocity_deviation_classifier(double val){
    return std::clamp((int)std::floor(val*10),0,9);
}

int collision_speed_classifier(double val){
    return std::clamp((int)std::floor(val*10),0,9);
}

int rp_rate_deviation_classifier(double val){
    return val>0.5 ? 1: 0;
}

int is_close_to_obstacle_classifier(double val){
    return val>0.5 ? 1: 0;
}