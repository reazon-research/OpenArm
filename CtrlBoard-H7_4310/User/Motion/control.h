#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "stdio.h"
#include "dm_drv.h"
#include "can_bsp.h"
#include "arm_math.h"

#define ROLE_LEADER 1
#define ROLE_FOLLWER 2
#define GRIP_SCALE 3.3
// #include "fdcan.h"

// extern OpenArm_t arm;
// extern OpenArm_t arm_f;

// typedef struct {
//     float Ts;
//     int role;
//     OpenArm_t *arm;
//     float Dn[NUM_MOTORS];
//     float Jn[NUM_MOTORS];
//     float gnd[NUM_MOTORS];
//     float gnf[NUM_MOTORS];
//     float Gn[NUM_MOTORS];
//     float Kp[NUM_MOTORS];
//     float Kd[NUM_MOTORS];
//     float Kf[NUM_MOTORS];
//     float disturbance_lowpass_in[NUM_MOTORS];
//     float disturbance_lowpass_out[NUM_MOTORS];
//     float disturbance[NUM_MOTORS];
//     float reactionforce_lowpass_in[NUM_MOTORS];
//     float reactionforce_lowpass_out[NUM_MOTORS];
//     float joint_torque[NUM_MOTORS];
// } BilateralInfo_t;

typedef struct {
    float Ts;
    int role;
    OpenArm_t *arm;
    FDCAN_HandleTypeDef* hcan;

    float *Dn;
    float *Jn;
    float *gnd;
    float *gnf;
    float *Gn;
    float *Kp;
    float *Kd;
    float *Kf;

    float *disturbance_lowpass_in;
    float *disturbance_lowpass_out;
    float *disturbance;
    float *reactionforce_lowpass_in;
    float *reactionforce_lowpass_out;
    float *reactionforce;
    float *joint_torque;
} BilateralInfo_t;



void aaaa();


// void init_bilateral_info(BilateralInfo_t* info, float Ts, int role,
//                          OpenArm_t* arm,
//                          float Dn[], float Jn[], float gnd[], float gnf[],
//                          float Gn[], float Kp[], float Kd[], float Kf[]);

void init_bilateral_info(BilateralInfo_t* info, float Ts, int role,
                         OpenArm_t* arm,
                         hcan_t *hcan,
                         float *Dn, float *Jn, float *gnd, float *gnf,
                         float *Gn, float *Kp, float *Kd, float *Kf,
                         float *disturbance_lowpass_in,
                         float *disturbance_lowpass_out,
                         float *disturbance,
                         float *reactionforce_lowpass_in,
                         float *reactionforce_lowpass_out,
                         float *reacrionforce,
                         float *joint_torque);

void Get_Response(BilateralInfo_t* bilateinfo, float* pos, float* vel, float* effort);
void Set_Reference();

void Compute_Friction_Torque(BilateralInfo_t* bilateinfo, float* vel, float* friction);
void Comupute_Gravity_Torque(BilateralInfo_t* bilateinfo, float* pos, float* gravity);
void Comupute_Corioli_Torque(BilateralInfo_t* bilateinfo, float* pos, float* vel, float* corioli);
void Comupute_Inertia_Diag(BilateralInfo_t* bilateinfo, float* pos, float* friction);

void Compute_velocity(BilateralInfo_t* bilateinfo, float* pos, float* vel);

void bilateral_control_v1(BilateralInfo_t* leader_info, BilateralInfo_t* follower_info);
void bilateral_control_21();
void bilateral_control_v3();


#endif /* __DM_DRV_H__ */

