#pragma once
#include <vector>
#include <string>


double mean(std::map<std::string,double> input){
    double mean = 0;
    double prob_sum = 0;
    for(int i=0; i<10; ++i){
        mean+=i*input.at("State"+std::to_string(i));
        prob_sum += input.at("State"+std::to_string(i));
    }
    if(prob_sum<0.98 || prob_sum > 1.02){
        ROS_ERROR("Probability sum does not sum to 1 when evaluating mean!");
        assert(false);
    }
    return mean;
}

double log_mean(std::map<std::string,double> input){
    double mean = 0;
    double prob_sum = 0;
    std::vector<double> states = {0.001,0.002,0.005,0.01,0.02,0.05,0.1,0.2,0.5,1};
    std::vector<std::string> state_names = {"p1000min","p0500min","p0200min","p0100min","p0050min","p0020min","p0010min","p0005min","p0002min","p0001min"};
    for(int i=0; i<10; ++i){
        mean+=states[i]*input.at(state_names[i]);
        prob_sum += input.at(state_names[i]);
    }
    if(prob_sum<0.98 || prob_sum > 1.02){
        ROS_ERROR("Probability sum does not sum to 1 when evaluating mean!");
        assert(false);
    }
    return mean;
}