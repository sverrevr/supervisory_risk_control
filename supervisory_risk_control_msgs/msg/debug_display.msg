#Optimal actions
float32 risk_cost
int16 optimal_safety_margin
int16 optimal_max_acc
int16 optimal_speed
bool found_feasible_option

#Probability of binary predictions with this choise:
float32 motor_saturation_causing_contact_with_obstacle
float32 hitting_obstacle_due_to_motorsaturation_causes_loss_of_control

#Mean frequency of prediction with this choise:
float32 mean_frequency_of_motor_saturation
float32 mean_frequency_of_contact_with_obstacle_due_to_motosaturation
float32 mean_frequency_of_loss_of_control_due_to_motorsturation
float32 mean_frequency_of_loss_of_control_due_to_motor_wear
float32 mean_frequency_of_loss_of_control

#Mean value of intermediate predictions with this choise:

#measurements
int16 measured_height_over_ground
int16 measured_motoruse
int16 measured_yaw_moment
int16 measured_drone_roll_pitch

#Mean intermediate estimation variables
float32 mean_motoruse_for_tether

#Mean of underlying varaibles
float32 mean_tether_tension_motor_use_scaling_factor
float32 mean_drone_roll_pitch_to_counteract_tether
float32 mean_motor_wear
float32 mean_motoruse_for_whirlewind
