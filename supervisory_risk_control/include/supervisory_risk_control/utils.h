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