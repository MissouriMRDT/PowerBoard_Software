
#include "Energia.h"
#include "RoveComm.h"
//12V Current Sensing
#define MULTIMEDIA_SENSE        PD4
#define NAV_SENSE               PD0
#define GIMBAL_ACT_SENSE        PD1
#define DRIVE_SENSE             PK3
#define SCISENSOR_ACT_SENSE     PD2
#define NETSWITCH_SENSE         PB5
#define CAM1_SENSE              PB4
#define CAM2_SENSE              PK0
#define BBB_SENSE               PD7
#define AUX_LOG_SENSE           PK2

//12V CTL
#define MULTIMEDIA_CTL          PL1
#define NAV_CTL                 PN4
#define GIMBAL_ACT_CTL          PA4
#define GIMBAL_LOG_CTL          PG0
#define DRIVE_CTL               PL2
#define SCISENSOR_ACT_CTL       PL3
#define SCISENSOR_LOG_CTL       PF3
#define NETSWITCH_CTL           PL0
#define CAM1_CTL                PF1
#define CAM2_CTL                PF2
#define BBB_CTL                 PN5
#define AUX_LOG_CTL             PB2
#define SPARE_CTL               PL5

//PACK CURRENT
#define P_MOTOR1_SENSE          PE4
#define P_MOTOR2_SENSE          PE5
#define P_MOTOR3_SENSE          PD3  
#define P_MOTOR4_SENSE          PE0
#define P_MOTOR5_SENSE          PE1
#define P_MOTOR6_SENSE          PE2
#define P_MOTOR7_SENSE          PE3
#define P_POE_SENSE             PD5
#define P_AUX_SENSE             PK1

//PACK BUSSES
#define P_MOTOR1_CTL            PC4       
#define P_MOTOR2_CTL            PC5
#define P_MOTOR3_CTL            PC6
#define P_MOTOR4_CTL            PC7
#define P_MOTOR5_CTL            PP0
#define P_MOTOR6_CTL            PP1
#define P_MOTOR7_CTL            PQ0
#define P_POE_CTL               PP4
#define P_AUX_CTL               PL4

#define MOTOR_DELAY             5000
#define DRIVE_DELAY             5000

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
