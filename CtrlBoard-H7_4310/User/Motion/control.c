#include "control.h"


void aaaa(){
	printf("control\n\r");
}

void init_bilateral_info(BilateralInfo_t* info, float Ts, int role,
                         OpenArm_t* arm,
						 FDCAN_HandleTypeDef *hcan,
                         float *Dn, float *Jn, float *gnd, float *gnf,
                         float *Gn, float *Kp, float *Kd, float *Kf,
                         float *disturbance_lowpass_in,
                         float *disturbance_lowpass_out,
                         float *disturbance,
                         float *reactionforce_lowpass_in,
                         float *reactionforce_lowpass_out,
						 float *reacrionforce,
                         float *joint_torque)
{
    info->Ts = Ts;
    info->role = role;
    info->arm = arm;
	info->hcan = hcan;

    info->Dn  = Dn;
    info->Jn  = Jn;
    info->gnd = gnd;
    info->gnf = gnf;
    info->Gn  = Gn;
    info->Kp  = Kp;
    info->Kd  = Kd;
    info->Kf  = Kf;

    info->disturbance_lowpass_in  = disturbance_lowpass_in;
    info->disturbance_lowpass_out = disturbance_lowpass_out;
    info->disturbance             = disturbance;
    info->reactionforce_lowpass_in  = reactionforce_lowpass_in;
    info->reactionforce_lowpass_out = reactionforce_lowpass_out;
	info->reactionforce             = reacrionforce;
    info->joint_torque              = joint_torque;
}

void Compute_Friction_Torque(BilateralInfo_t* bilateinfo, float* vel, float* friction){
    for (int i = 0; i < NUM_MOTORS; i++) {
        friction[i] = bilateinfo->Dn[i] * vel[i];
    }
}

void Get_Response(BilateralInfo_t* bilateinfo, float* pos, float* vel, float* effort){
    for (int i = 0; i < NUM_MOTORS; i++) {
		pos[i] = bilateinfo->arm->motors[i].pos;
		vel[i] = bilateinfo->arm->motors[i].vel;
		effort[i] = bilateinfo->reactionforce[i];
    }
}

void bilateral_control_v1(BilateralInfo_t* leader_info, BilateralInfo_t* follower_info){
	// printf("bilateral v1 \n\r");
	float pos_l[NUM_MOTORS], vel_l[NUM_MOTORS], effort_l[NUM_MOTORS];
	float pos_f[NUM_MOTORS], vel_f[NUM_MOTORS], effort_f[NUM_MOTORS];

	//temp
	float zero[NUM_MOTORS] = {0.0f, 0.0f};

	Get_Response(leader_info, pos_l, vel_l, effort_l);
	Get_Response(follower_info, pos_f, vel_f, effort_f);

	//compute friction 
	float friction_l[NUM_MOTORS];
	float friction_f[NUM_MOTORS];
	Compute_Friction_Torque(leader_info, vel_l, friction_l);
	Compute_Friction_Torque(follower_info, vel_f, friction_f);
	
	for(int i = 0; i < NUM_MOTORS; i++){

		float tau_leader_p = leader_info->Kp[i]*(pos_f[i] - pos_l[i]);
		float tau_leader_v = leader_info->Kd[i]*(vel_f[i] - vel_l[i]);
		float tau_leader_f = -leader_info->Kf[i]*(effort_f[i] + effort_l[i]);

		float tau_follower_p = follower_info->Kp[i]*(pos_l[i] - pos_f[i]);
		float tau_follower_v = follower_info->Kd[i]*(vel_l[i] - vel_f[i]);
		float tau_follower_f = -follower_info->Kf[i]*(effort_l[i] + effort_f[i]);

		leader_info->joint_torque[i] = tau_leader_p + tau_leader_v + tau_leader_f + leader_info->disturbance[i];
		follower_info->joint_torque[i] = tau_follower_p + tau_follower_v + tau_follower_f + follower_info->disturbance[i];

		double a_dl = leader_info->gnd[i] * leader_info->Ts;
		double a_df = follower_info->gnd[i] * follower_info->Ts;
		double a_fl = leader_info->gnf[i] * leader_info->Ts;
		double a_ff = follower_info->gnf[i] * follower_info->Ts;
		// print("a_l : %f \n\r", a_dl);

		//DOB leader
		leader_info->disturbance_lowpass_in[i] = (leader_info->joint_torque[i]) + leader_info->gnd[i] * leader_info->Jn[i] * vel_l[i];
		leader_info->disturbance_lowpass_out[i] +=  a_dl * (leader_info->disturbance_lowpass_in[i] - leader_info->disturbance_lowpass_out[i]);
		leader_info->disturbance[i] = leader_info->disturbance_lowpass_out[i] - leader_info->gnd[i] * leader_info->Jn[i] * vel_l[i];
		
		//DOB follower
		follower_info->disturbance_lowpass_in[i] = (follower_info->joint_torque[i]) + follower_info->gnd[i] * follower_info->Jn[i] * vel_l[i];
		follower_info->disturbance_lowpass_out[i] += a_df * (follower_info->disturbance_lowpass_in[i] - follower_info->disturbance_lowpass_out[i]);
		follower_info->disturbance[i] = follower_info->disturbance_lowpass_out[i] - follower_info->gnd[i] * follower_info->Jn[i] * vel_l[i];
		
		//RFOB leader
		leader_info->reactionforce_lowpass_in[i] = (leader_info->joint_torque[i]) + leader_info->gnf[i] * leader_info->Jn[i] * vel_l[i] - friction_l[i];
		leader_info->reactionforce_lowpass_out[i] += a_fl * (leader_info->reactionforce_lowpass_in[i] - leader_info->reactionforce_lowpass_out[i]);
		leader_info->reactionforce[i] = leader_info->reactionforce_lowpass_out[i] - leader_info->gnf[i] * leader_info->Jn[i] * vel_l[i];
		
		//RFOB follower
		follower_info->reactionforce_lowpass_in[i] = (follower_info->joint_torque[i]) + follower_info->gnf[i] * follower_info->Jn[i] * vel_f[i] - friction_f[i];
		follower_info->reactionforce_lowpass_out[i] += a_ff * (follower_info->reactionforce_lowpass_in[i] - follower_info->reactionforce_lowpass_out[i]);
		follower_info->reactionforce[i] = follower_info->reactionforce_lowpass_out[i] - follower_info->gnf[i] * follower_info->Jn[i] * vel_f[i];
		
		//fix input torque
		leader_info->joint_torque[i] += (-tau_leader_p - tau_leader_v);
		follower_info->joint_torque[i] += (-tau_follower_p - tau_follower_v);

	}

	// printf("dis : %f \n\r", leader_info->disturbance[1]);

	move_mit_all(leader_info->arm, leader_info->hcan, pos_f, vel_f, leader_info->Kp, leader_info->Kd, leader_info->joint_torque);
	move_mit_all(follower_info->arm, follower_info->hcan, pos_l, vel_l, follower_info->Kp, follower_info->Kd, follower_info->joint_torque);

	// move_mit_all(leader_info->arm, leader_info->hcan, pos_f, vel_f, leader_info->Kp, leader_info->Kd, zero);
	// move_mit_all(follower_info->arm, follower_info->hcan, pos_l, vel_l, follower_info->Kp, follower_info->Kd, zero);

}



