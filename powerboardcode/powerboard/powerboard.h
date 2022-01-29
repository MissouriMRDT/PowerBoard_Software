
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

uint8_t currentSense12V[] = {MULTIMEDIA_SENSE, NAV_SENSE, GIMBAL_ACT_SENSE, DRIVE_SENSE, SCISENSOR_ACT_SENSE, NETSWITCH_SENSE, CAM1_SENSE, CAM2_SENSE, BBB_SENSE, AUX_LOG_SENSE};
uint8_t actuation12V[5] = {GIMBAL_ACT_CTL, AUX_LOG_CTL, SPARE_CTL, MULTIMEDIA_CTL, SCISENSOR_ACT_CTL};
uint8_t logic12V[8] = {GIMBAL_LOG_CTL, DRIVE_CTL, NETSWITCH_CTL, NAV_CTL, CAM1_CTL, CAM2_CTL, BBB_CTL, SCISENSOR_LOG_CTL};
//uint8_t drivePack = P_DRIVE_CTL;
uint8_t currentSensePack[9] = {P_MOTOR1_SENSE, P_MOTOR2_SENSE, P_MOTOR3_SENSE, P_MOTOR4_SENSE, P_MOTOR5_SENSE, P_MOTOR6_SENSE, P_MOTOR7_SENSE, P_POE_SENSE, P_AUX_SENSE};
uint8_t bussesPack[4] = {P_POE_CTL, , P_AUX_CTL, };
uint8_t bussesMotor[7] = {P_MOTOR1_CTL, P_MOTOR2_CTL, P_MOTOR3_CTL, P_MOTOR4_CTL, P_MOTOR5_CTL, P_MOTOR6_CTL, P_MOTOR7_CTL};

RoveCommEthernet RoveComm;
rovecomm_packet packet; 
EthernetServer TCPServer(RC_ROVECOMM_POWERBOARD_PORT);
