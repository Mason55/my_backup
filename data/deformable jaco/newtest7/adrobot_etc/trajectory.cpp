
#include "adrobot_etc.h"
#define TRAJECTORY_LENGTH 20

typedef struct
{
  int data_num;
  PATH Path[TRAJECTORY_LENGTH];
} TRJ_BUFF;

TRJ_BUFF PathBuff;

void initTrjBuff()
{

    PathBuff.data_num=0;

}

int PutTrjBuff(PATH *path)
{
    int  i, data_num;

    if(PathBuff.data_num>=TRAJECTORY_LENGTH) //
    {
	printf("Error: The path buffer is full\n");
        return 1;
    }
    else
    {
        data_num=PathBuff.data_num;

	if((data_num>0)&&(data_num<=TRAJECTORY_LENGTH))
        {
	    for(i=data_num;i>0;i--)
	    {
		PathBuff.Path[i]=PathBuff.Path[i-1];
             }
        }
		PathBuff.Path[0]=*path;
        PathBuff.data_num=data_num+1;

	//if(data_num+1>=TRAJECTORY_LENGTH)
	//  PathBuff.Full_or_Not=1; // Set the buffer to be full
    }
    return 0;
}

int GetTrjBuff(PATH *path)
{
    int i,data_num;
    if(PathBuff.data_num<1)  // if the buffer is not empty
    {
	printf("Error: The path buffer is empty");
        return 1;
     }
    else
    {
        data_num=PathBuff.data_num;

        *path=PathBuff.Path[0];

        for(i=0;i<data_num;i++)
	    {
        	PathBuff.Path[i]=PathBuff.Path[i+1];
        }
        PathBuff.data_num=data_num-1;

//	if(data_num-1<=0)
//	    PathBuff.Full_or_Not=0;
    }
    return 0;
}
// ステップ関数による軌道補間
double CalcStepTraje(double orig, double goal, double freq, double time)
{
    if(time < 1.0/freq/2.0)
        return orig;
    return goal;
}

// 1 次関数による軌道補間
double Calc1JiTraje(double orig, double goal, double freq, double time)
{
    double ref = goal;
    double time_n = freq*time;

    if(time_n <= 1)
        ref = orig + (goal-orig)*time_n;
    return ref;
}

// 3 次関数による軌道補間
double Calc3JiTraje(double orig, double goal, double freq, double time)
{
    double ref = goal;
    double time_n = freq*time;

    if(time_n <= 1)
        ref = orig + (goal-orig)*time_n*time_n*(3.0-2*time_n);
    return ref;
}

// 5 次関数による軌道補間
double Calc5JiTraje(double orig, double goal, double freq, double time)
{
    double ref = goal;
    double time_n = freq*time;

    if(time_n <= 1)
        ref = orig + (goal-orig)
            *time_n*time_n*time_n*(10.0+time_n*(-15.0 + 6.0*time_n));
    return ref;
}

// sin 関数による軌道補間
double CalcSinTraje(double orig, double goal, double freq, double time)
{
    double ref = goal;
    double time_n = freq*time;

    if(time_n <= 1)
        ref = orig + (goal-orig)* time_n
            - (goal-orig) * sin(2.0*M_PI*time_n)/(2.0*M_PI);
    return ref;
}

// 3 次関数による速度軌道補間
double Calc3JiTrajeVelo(double orig, double goal, double freq, double time)
{
    double ref = 0.0;
    double time_n = freq*time;

    if(time_n <= 1)
        ref = (goal-orig)*freq*6.0*time_n*(1-time_n);
    return ref;
}

// 5 次関数による速度軌道補間
double Calc5JiTrajeVelo(double orig, double goal, double freq, double time)
{
    double ref = 0.0;
    double time_n = freq*time;

    if(time_n <= 1)
        ref = freq*(goal-orig)*30.0*time_n *time_n*(time_n-1)*(time_n-1);
    return ref;
}

// sin 関数による速度軌道補間
double CalcSinTrajeVelo(double orig, double goal, double freq, double time)
{
    double ref = 0.0;
    double time_n = freq*time;

    if(time_n <= 1)
        ref = freq * (goal-orig)*(1-cos(2.0*M_PI*time_n));
    return ref;
}




