
#include "Energia.h"
//12V Current Sensing
#define AUX_MOUNT_SENSE     PK3
#define MULTIMEDIA_SENSE    PK2
#define LOW_CURRENT_SENSE   PK1
#define GIMBAL_SENSE        PK0

//12V CTL
#define BUCKCTL             PG_1
#define MULTIMEDIA_LOG_CTL  PK_5
#define MULTIMEDIA_ACT_CTL  PM_0
#define AUX_LOG_CTL         PM_1
#define AUX_ACT_CTL         PM_2
#define GIMBAL_LOG_CTL      PH_0
#define GIMBAL_ACT_CTL      PH_1
#define NAV_BOARD_CTL       PK_7
#define CAM_CTL             PP_3
#define EXTRA_CTL           PQ_1
#define DRIVE_CTL           PM_6

//PACK CURRENT
#define P_MOTOR1_SENSE       PD_5
#define P_MOTOR2_SENSE       PD_4
#define P_MOTOR3_SENSE       PD_2
#define P_MOTOR4_SENSE       PD_3
#define P_SPARE_SENSE        PE_4
#define P_ROCKET_SENSE       PD_0
#define P_TWELVE_SENSE       PD_1
#define P_VACUUM_SENSE       PE_1
#define P_DRIVE_SENSE        PE_5
#define P_AUX_SENSE          PE_2

//PACK BUSSES
#define P_MOTOR1_CTL         PQ_0
#define P_MOTOR2_CTL         PP_1
#define P_MOTOR3_CTL         PP_0
#define P_MOTOR4_CTL         PC_7
#define P_SPARE_CTL          PC_4
#define P_ROCKET_CTL         PN_2
#define P_TWELVE_CTL         PH_3
#define P_VACUUM_CTL         PE_0
#define P_DRIVE_CTL          PC_6
#define P_AUX_CTL            PE_3


list currentSense = [ AUX_MOUNT_SENSE, MULTIMEDIA_SENSE, GIMBAL_SENSE, LOW_CURRENT_SENSE];
list busses = [AUX_ACT_CTL, MULTIMEDIA_ACT_CTL, GIMBAL_ACT_CTL, BUCKCTL, AUX_LOG_CTL, MULTIMEDIA_LOG_CTL, GIMBAL_LOG_CTL, NAV_BOARD_CTL, CAM_CTL, EXTRA_CTL, DRIVE_CTL];
list P_CurrentSense = [ P_MOTOR1_SENSE, P_MOTOR2_SENSE, P_MOTOR3_SENSE, P_MOTOR4_SENSE, P_SPARE_SENSE, P_ROCKET_SENSE, P_TWELVE_SENSE, P_VACUUM_SENSE, P_DRIVE_SENSE, P_AUX_SENSE];
list P_busses = [P_MOTOR1_CTL, P_MOTOR2_CTL, P_MOTOR3_CTL, P_MOTOR4_CTL, P_SPARE_CTL, P_ROCKET_CTL, P_TWELVE_CTL, P_VACUUM_CTL, P_DRIVE_CTL, P_AUX_CTL];