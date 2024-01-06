#ifndef pinassignments_h
#define pinassignments_h

////////////////////////////
//      Control Pins      //
////////////////////////////

// High Current
#define HC_SPARE_CTL     30
#define AUX_CTL          29
#define M1_CTL           28
#define M2_CTL           12
#define M3_CTL           11
#define M4_CTL           5
#define M5_CTL           6
#define M6_CTL           7
#define MS_CTL           9

// Low Current
#define LC_SPARE_CTL     37
#define ROUTERPI_CTL     34
#define DIFFGPS_CTL      35
#define CORE_CTL         36

// 12 Volts
#define CAM_CTL          10


////////////////////////////
//  Current Sensing Pins  //
////////////////////////////

// High Current
#define M1_CS           21
#define M2_CS           22
#define M3_CS           23
#define M4_CS           24
#define M5_CS           25
#define M6_CS           26
#define MS_CS           27
#define AUX_CS          20
#define HC_SPARE_CS     19

// Low Current
#define ROUTERPI_CS     41
#define DIFFGPS_CS      40
#define CORE_CS         39
#define LC_SPARE_CS     38

// 12 Volts
#define CAM_CS          18

// Networking
#define POE_CS          14
#define NS1_CS          17
#define NS2_CS          16
#define NS3_CS          15


#endif