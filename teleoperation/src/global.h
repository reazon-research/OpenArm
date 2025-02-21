#ifndef REAZON_BILATERAL_ALOHA_GLOBAL_H_
#define REAZON_BILATERAL_ALOHA_GLOBAL_H_

#include <unistd.h>
#include <time.h>

/*
 * Basic config for Aloha Stationary Kits
 */
constexpr double PI = 3.14159265358979323846;

// 9piecies including gripper
#define NJOINTS 8
#define NMOTORS 8

#define ROLE_LEADER 1
#define ROLE_FOLLOWER 2

#define LEADER_DEVICENAME0 "can0"
#define FOLLOWER_DEVICENAME0 "can1"


/*
 * URDF Files for KDL
 */
#define LEADER_URDF_PATH    "/home/thomason/triad/logbook/toki/openarm_bilateral/urdf/openarm_grip.urdf"
#define LEADER_START_LINK   "base_link"
#define LEADER_END_LINK     "grip_attach_1"

#define FOLLOWER_URDF_PATH  "/home/thomason/triad/logbook/toki/openarm_bilateral/urdf/openarm_grip.urdf"
#define FOLLOWER_START_LINK "base_link"
#define FOLLOWER_END_LINK   "grip_attach_1"

/*
 * Macros for Bilateral control
 */
#define FREQUENCY 200.0
#define ALPHA 1.0
#define BETA  0.6
#define CUTOFF_FREQUENCY 100.0

static const double Dn_L[] = {0.6, 0.9, 0.9, 0.9, 0.10, 0.160, 0.160, 0.07};
static const double Gn_L[] = {0.000, 2.100, 1.600, 0.100, 0.200, 0.000, 0.000, 0.150};
static const double Jn_L[] = {0.042, 0.100, 0.090, 0.018, 0.016, 0.010, 0.008, 0.014};
static const double gn_L[] = {3.0,4.3,4.0,6.5,3.0,3.0,3.0};
static const double Kp_L[] = {450.0, 400.0, 450.0, 650.0, 970.0, 500.0, 300.0, 350.0};
static const double Kd_L[] = {15.00, 15.00, 15.00, 60.00, 100.00, 20.00, 20.00, 25.00};
static const double Kf_L[] = {1.3, 1.5, 1.7, 2.8, 0.8, 1.0, 0.8, 1.5};


static const double Dn_F[] = {0.6, 0.9, 0.9, 0.9, 0.10, 0.160, 0.160, 0.07};
static const double Gn_F[] = {0.000, 4.300, 1.900, 0.000, 0.350, 0.000, 0.000, 0.200};
static const double Jn_F[] = {0.075, 0.150, 0.140, 0.024, 0.020, 0.013, 0.008, 0.014};
static const double gn_F[] = {3.0,4.3,4.0,6.5,3.0,3.0,3.0};
static const double Kp_F[] = {450.0, 400.0, 450.0, 650.0, 970.0, 300.0, 300.0, 350.0};
static const double Kd_F[] = {15.00, 15.00, 15.00, 60.00, 100.00, 20.00, 20.00, 25.00};
static const double Kf_F[] = {1.3, 1.5, 1.7, 2.8, 0.8, 1.0, 0.8, 1.5};


// static const double INITIAL_POSITION[NMOTORS] = {
//   0, -0.62, 0, 1.69, 0, -0.8, 0, 0
// };

static const double INITIAL_POSITION[NMOTORS] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

/*
 * Other util functions
 */
static inline double calcTime(void)
{
  struct::timespec getTime;
  clock_gettime(CLOCK_MONOTONIC, &getTime);
  return (getTime.tv_sec + getTime.tv_nsec*1e-9);
}

#endif
