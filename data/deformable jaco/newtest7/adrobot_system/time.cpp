
#include "adrobot_system.h"

unsigned long long int tsc0;
int timer_counter =0;
static double StartTime = 0.0;

double GetCurrentTime()
{
  struct timespec time;
  double curTime;
  unsigned long long int tsc1;
  clock_gettime(CLOCK_REALTIME, &time);
  tsc1 = (long long int) (time.tv_sec)*NSEC_PER_SEC + (long long int) (time.tv_nsec);;

  //curTime = (tsc1-tsc0)/NSEC_PER_SEC;
   curTime = (long double)(tsc1-tsc0)/NSEC_PER_SEC;
  return curTime;
 }
void ResetTime()
{
    unsigned long long int tsc1;
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    tsc1=(long long int) (time.tv_sec)*NSEC_PER_SEC + (long long int) (time.tv_nsec);
    tsc0=tsc1;
}
void ResetTimerCounter()
{
    timer_counter=0;
}
void TimerCounterAdd()
{
    timer_counter++;
}
int GetTimerCounter()
{
    return timer_counter;
}
double GetOffsetTime(void)
{
    return GetCurrentTime()-StartTime;
}
void SetStartTime(double time)
{
    StartTime = time;
    printf("Set start time, t=%f\n", time);
}
/*
double GetSamplingTime()
{
    return CONTROL_SAMPLING_TIME_USEC/(1000000.0);
}
*/




