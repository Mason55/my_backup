#ifndef ADROBOT_ETC_H
#define ADROBOT_ETC_H
#include "../common.h"
/* trajectory.c */
extern void initTrjBuff();
extern int PutTrjBuff(PATH* path);
extern int GetTrjBuff(PATH* path);
extern double CalcStepTraje(double orig, double goal, double freq, double time);
extern double Calc1JiTraje(double orig, double goal, double freq, double time);
extern double Calc3JiTraje(double orig, double goal, double freq, double time);
extern double Calc5JiTraje(double orig, double goal, double freq, double time);
extern double CalcSinTraje(double orig, double goal, double freq, double time);
extern double Calc3JiTrajeVelo(double orig, double goal, double freq, double time);
extern double Calc5JiTrajeVelo(double orig, double goal, double freq, double time);
extern double CalcSinTrajeVelo(double orig, double goal, double freq, double time);

#endif
