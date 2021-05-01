
#include "Energia.h"
#include "RoveComm.h"
//12V Current Sensing
#define AUX_MOUNT_SENSE     PK_3
#define MULTIMEDIA_SENSE    PK_2
#define LOW_CURRENT_SENSE   PK_1
#define GIMBAL_SENSE        PK_0

//12V CTL
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

#define MOTOR_DELAY          5000
#define DRIVE_DELAY          5000

uint8_t currentSense12V[4] = { AUX_MOUNT_SENSE, MULTIMEDIA_SENSE, GIMBAL_SENSE, LOW_CURRENT_SENSE};
uint8_t actuation12V[3] = { GIMBAL_ACT_CTL, MULTIMEDIA_ACT_CTL, AUX_ACT_CTL};
uint8_t logic12V[7] = {GIMBAL_LOG_CTL, MULTIMEDIA_LOG_CTL, AUX_LOG_CTL, DRIVE_CTL, NAV_BOARD_CTL, CAM_CTL, EXTRA_CTL};
uint8_t drivePack = P_DRIVE_CTL;
uint8_t currentSensePack[10] = { P_MOTOR1_SENSE, P_MOTOR2_SENSE, P_MOTOR3_SENSE, P_MOTOR4_SENSE, P_SPARE_SENSE, P_ROCKET_SENSE, P_TWELVE_SENSE, P_VACUUM_SENSE, P_DRIVE_SENSE, P_AUX_SENSE};
uint8_t bussesPackStart[3] = { P_ROCKET_CTL, P_TWELVE_CTL, P_AUX_CTL};
uint8_t bussesPack[4] = { P_TWELVE_CTL, P_ROCKET_CTL, P_AUX_CTL, P_DRIVE_CTL};
uint8_t vacuumCtrl[1] = {P_VACUUM_CTL};
uint8_t bussesMotor[5] = {P_MOTOR1_CTL, P_MOTOR2_CTL, P_MOTOR3_CTL, P_MOTOR4_CTL, P_SPARE_CTL};

RoveCommEthernet RoveComm;
rovecomm_packet packet; 
EthernetServer TCPServer(RC_ROVECOMM_POWERBOARD_PORT);

