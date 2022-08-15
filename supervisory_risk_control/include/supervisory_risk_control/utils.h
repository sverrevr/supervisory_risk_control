#pragma once
#include <vector>
#include <string>

template <typename t>
class map_stringkey : public std::map<std::string, t>{
    public:
    map_stringkey(const std::map<std::string, t>& val) : std::map<std::string,t>(val){}
    map_stringkey() :std::map<std::string,t>(){}

    auto& at(std::string value){
        if(std::map<std::string, t>::find(value) == std::map<std::string, t>::end()){
            ROS_ERROR("Attempting to access non existing key \"%s\"",value.c_str());
            /*ROS_ERROR("Existing keys are:");
            for(auto it = std::map<std::string, t>::begin(); it!=std::map<std::string, t>::end(); ++it){
                ROS_ERROR("%s",it->first().c_str());
            }*/
            throw std::string{"Attempting to access non existing key " + value};
        }
        return std::map<std::string, t>::at(value);
    }

    const auto& at(std::string value) const{
        if(std::map<std::string, t>::find(value) == std::map<std::string, t>::end()){
            ROS_ERROR("Attempting to access non existing key \"%s\"",value.c_str());
            /*ROS_ERROR("Existing keys are:");
            for(auto it = std::map<std::string, t>::begin(); it!=std::map<std::string, t>::end(); ++it){
                ROS_ERROR("%s",it->first().c_str());
            }*/
            throw std::string{"Attempting to access non existing key " + value};
        }
        return std::map<std::string, t>::at(value);
    }
};

double mean(map_stringkey<double> input){
    double mean = 0;
    double prob_sum = 0;
    for(int i=0; i<10; ++i){
        mean+=i*input.at("State"+std::to_string(i));
        prob_sum += input.at("State"+std::to_string(i));
    }
    if(prob_sum<0.98 || prob_sum > 1.02){
        throw std::string("Probability sum does not sum to 1 when evaluating mean!");
    }
    return mean/10;
}
/*double log_mean(std::map<std::string,double> input){
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
}*/

double log_mean(map_stringkey<double> input){
    int number_of_states = input.size();
    double mean = 0;
    double prob_sum = 0;
    for(int i=0; i<number_of_states; ++i){
        std::string state_name = "State"+std::to_string(i);
        double lower_state_frequency = (std::pow(100.0,i/(number_of_states*1.0))-1)/99.0;
        double upper_state_frequency = (std::pow(100.0,(i+1)/(number_of_states*1.0))-1)/99.0;
        double mean_state_frequency = (upper_state_frequency+lower_state_frequency)/2;
        mean+=mean_state_frequency*input.at(state_name);
        prob_sum += input.at("State"+std::to_string(i));
    }
    if(prob_sum<0.98 || prob_sum > 1.02){
        throw std::string("Probability sum does not sum to 1 when evaluating mean!");
    }
    return mean;
}

