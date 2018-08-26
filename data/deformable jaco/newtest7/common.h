
#ifndef COMMON_H
#define COMMON_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "./adrobot_kinematics/adrobot_kinematics.h"
/*Eigen header*/
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
using namespace Eigen;
/*Eigen header*/

#define DEBUG
//#define ROBOT_OFFLINE
#define ON 1
#define OFF 0
#define INIT_C 0
#define EXIT_C 255
#define EXP_DATA_LENGTH 10000
#define EXP_DATA_INTERVAL 1
#define NSEC_PER_SEC	(1000000000)
#define Rad2Deg 180.0/M_PI
#define Deg2Rad M_PI/180.0

struct shm_interface{
    int status_print;
    int status_control;
};

typedef struct{
    double t[6];
}THETA;

typedef struct{
    double t[6];
}POS;

typedef struct{
    float t[1];
}FORCE;

typedef struct{
    double Orig[6];
    double Goal[6];
    double Freq;
    int Mode;
}PATH;

typedef struct{
	double t[6];
}HND_PA;
typedef struct{
    double K_vt[6];
    double K_pt[6];
    double K_it[6];
}GAIN;

typedef struct{
    float t[6];
}MARKPOS;

typedef struct{
    float t[1];
}DISTOGRIPPER;

typedef struct{
    double t[3];
}POSITION;
typedef struct{
    float t[1];
}DISTANCE;
//extern MatrixXd MinimumDistance(1,10);

typedef struct{
    int PosOriServoFlag;
    int SamplingFreq;
    double SamplingTime;
    double Time;
    int ServoFlag;
    int NewPathFlag;
    int PathtailFlag;
    THETA CurTheta;
    THETA CurDTheta;
    THETA RefTheta;
    THETA RefDTheta;
    THETA deltaTheta;
    THETA xianweiTheta;
    JOINTLINK jnk;
    POS   CurPos;
    POS	  RefPos;
    FORCE CurForce;
    GAIN Gain;
    PATH Path;
    MARKPOS markpos;
    DISTOGRIPPER distogripper;
    POSITION  errpos;
    FORCE Attractive;
    FORCE Repulsive;
    DISTANCE xy;
    DISTANCE uv;
}SVO;


typedef struct{
    double Time;
    THETA CurTheta;
    THETA CurDTheta;
    THETA RefTheta;
    THETA RefDTheta;
    THETA deltaTheta;
    THETA xianweiTheta;
    POS   CurPos;
    POS   RefPos;
    PATH Path;
    FORCE CurForce;
    GAIN  Gain;
    MARKPOS markpos;
    DISTOGRIPPER distogripper;
    POSITION  errpos;
    FORCE Attractive;
    FORCE Repulsive;
    DISTANCE xy;
    DISTANCE uv;
 }SVO_SAVE;
#endif

