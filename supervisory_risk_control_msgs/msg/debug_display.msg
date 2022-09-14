#Optimal actions
float32 risk_cost
float32 safety_margin
float32 max_acc
float32 speed
bool abort

#Probability of binary predictions with this choise:

#Mean frequency of prediction with this choise:
float32 S1_inv_obs
float32 S2_braking_dist_sm
float32 S3_turbulence_sm
float32 S4_mtr_saturation_sm
float32 S5_obstacle
float32 S6_mtr_failure
float32 S7_disturbance

#measurements
float32 m_height
float32 m_mtr
float32 m_tilt
float32 m_rp_er
float32 m_visual_dust
float32 m_lidar_dust

#Mean intermediate estimation variables
float32 mtr_tether
float32 mtr
float32 mtr_trblnc

#Mean of underlying varaibles
float32 mtr_wear
float32 mtr_rand
float32 dist
float32 env_obs