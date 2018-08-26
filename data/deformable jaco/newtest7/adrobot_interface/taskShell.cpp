#include "../common.h"
#include <condition_variable>
#include "adrobot_interface.h"
#include "../matrix/matrix.h"
#include "../adrobot_io/ur_driver.h"
#include "../adrobot_system/adrobot_system.h"
#include "../adrobot_control/adrobot_control.h"
#include "../adrobot_kinematics/adrobot_kinematics.h"
/*shared memory header*/
#include<stdio.h>
#include<stdlib.h>
#include<sys/ipc.h>
#include<sys/shm.h>
#include<string.h>
#include<errno.h>
#include<vector>
#include<iostream>
/*shared memory header*/
#define MY_PRIORITY	(49)
#define MAX_SAFE_STACK  (8*1024)

void *display_function(void* param);
void *interface_function(void* param);
void DisplayMenu(void);
void DisplayCurrentInformation(PATH path,int flag, GAIN gain);

struct shm_interface shm_servo_inter;
pthread_mutex_t servo_inter_mutex = PTHREAD_MUTEX_INITIALIZER;

int shmid; // 共享内存标识符
void* shm_addr=NULL;
extern SVO pSVO;
extern pthread_mutex_t mymutex;
std::condition_variable rt_ur_msg_cond;
std::condition_variable ur_msg_cond;
std::string host_name="192.168.1.102";

/* for test */
UrDriver* testUr;
void stack_prefault(void)
{
  unsigned char dummy[MAX_SAFE_STACK];
  memset(dummy, 0, MAX_SAFE_STACK);   //初始化dummy[]所在的内存空间
  return;
}

int main( void )
{

  pthread_t interface_thread,display_thread;
  int loop_flag;
  loop_flag=0;
    printf("\nExperiment Start\n");

  shm_servo_inter.status_control = INIT_C;
  struct timespec t;
  struct sched_param param;
  int interval = 8000000; /* 8 ms*/

  /* connect to ur robot */
  std::cout<<"Connecting to "<<host_name<<"\n";
  UrDriver urRobot(rt_ur_msg_cond,ur_msg_cond,host_name,
		   50007, 0.016, 12, 0.08, 0);
 #ifndef ROBOT_OFFLINE
  urRobot.start();
  urRobot.setServojTime(0.008);
  // for test
  testUr=&urRobot;
#endif

  /* Declare ourself as a real time task */
  param.sched_priority = MY_PRIORITY;//スケジューラでのプロセスの優先順位を変更する(99が優先順位最大)
  if(sched_setscheduler(0,SCHED_FIFO, &param) == -1) {
     perror("sched_setscheduler failed\n");
     exit(-1);
   }

   /* Lock memory */
   if(mlockall(MCL_CURRENT|MCL_FUTURE)==-1)
   { perror("mlockall failed");
     exit(-2);
   }

   stack_prefault();
   clock_gettime(CLOCK_MONOTONIC ,&t);
   /* start after one second */
    t.tv_sec++;//一秒后启动控制回路

     /* start interface thread*/
    if(pthread_create(&interface_thread, NULL,interface_function, NULL))
    {
      perror("interface_thread create\n");
      exit(1);
    }
      /* start display thread*/
    if(pthread_create(&display_thread, NULL, display_function, NULL))
    {
      perror("Display_thread create\n");
      exit(1);
    }

      /* reset save buffer */
    SaveDataReset();
//    for(int i=0;i<6;i++){
//        pSVO.CurTheta.t[i]=0.3;
//        pSVO.jnk.c[i]=cos(pSVO.CurTheta.t[i]);
//        pSVO.jnk.s[i]=sin(pSVO.CurTheta.t[i]);
//    }
//    /*创建共享内存*/
//     shmid = shmget((key_t)1234, 24, 0666|IPC_CREAT);//共享内存段名称为1234
//     if (shmid == -1)
//       {
//           fprintf(stderr, "shmat failed\n");
//           exit(EXIT_FAILURE);
//       }
//   /*创建共享内存*/
//   /*将共享内存连接到当前进程的地址空间(映射共享内存)*/
//     shm_addr = (void*)shmat(shmid, 0, 0);

#ifndef ROBOT_OFFLINE
    urRobot.uploadProg();
#endif
        while(1){

        	if(shm_servo_inter.status_control==EXIT_C){
        		printf("Program end\n");
                /*分离共享内存*/
//                if(shmdt(shm_addr)==-1)
//                {
//                    printf("shmdt error!\n");
//                    exit(1);
//                }
                /*分离共享内存*/
        		break;
        	}

        	/* wait until next shot */
        	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
        			//在这里设定执行到特定时刻时将执行权限交给其他的进程．
        	//時刻が来たら実行権限を復活させて優先的に実行される
                /* do the stuff */
                 /* reset */
        	if(loop_flag==0){
        	   	  ResetTime();
        	   	  loop_flag=1;
        	}

           servo_function(&urRobot);
           /* calculate next shot */
           t.tv_nsec += interval;//次にプロセスが復帰する時刻を設定

           while(t.tv_nsec >= NSEC_PER_SEC){//桁の繰り上げ
                t.tv_nsec -= NSEC_PER_SEC;
                t.tv_sec++;
           }
        }


        if(pthread_join(interface_thread,NULL)){
        		perror("pthread_join at interface_thread\n");
        		exit(1);
        }

        if(pthread_join(display_thread,NULL)){
            	perror("pthread_join at displya_thread\n");
        }
        //close IO
        ExpDataWrite();
	    //close UR

#ifndef ROBOT_OFFLINE
	          urRobot.halt();
#endif
//       system("ipcs -m");
       exit(1);
}

void *display_function(void* param)
{
    int status_interface,i;
    int loop_counter;
    SVO display_svo;
    double time;
    double jnt_angle[6];

    loop_counter=0;

    do{
	   time=GetCurrentTime();

       SvoReadFromDis(&display_svo);
       if((display_svo.PosOriServoFlag==ON)&&(display_svo.ServoFlag == ON) && (GetOffsetTime() < (1.0/ (double)display_svo.Path.Freq))){
    		   printf("\n");
    		   printf("TIME:%0.1f\n",time);
    		   printf("Current position of hand:\n");
    		   printf("X[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[Deg]\n");
    	       printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
    	    		display_svo.CurPos.t[0],display_svo.CurPos.t[1],display_svo.CurPos.t[2],
					display_svo.CurPos.t[3]*Rad2Deg,display_svo.CurPos.t[4]*Rad2Deg,display_svo.CurPos.t[5]*Rad2Deg);
    		   printf("Reference position of hand:\n");
    		   printf("X[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[Deg]\n");
    		   printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
    		       	display_svo.RefPos.t[0],display_svo.RefPos.t[1],display_svo.RefPos.t[2],
					display_svo.RefPos.t[3]*Rad2Deg,display_svo.RefPos.t[4]*Rad2Deg,display_svo.RefPos.t[5]*Rad2Deg);
    	    }
       if((display_svo.PosOriServoFlag==OFF)&&(display_svo.ServoFlag == ON) && (GetOffsetTime() < (1.0/ (double)display_svo.Path.Freq))){
    	               for(i=0;i<6;i++)
    	               	jnt_angle[i]=display_svo.CurTheta.t[i]*Rad2Deg;
    	               printf("\n");
    	               printf("T: %.2f Ref[deg]: %.2f %.2f %.2f %.2f %.2f %.2f Jnt[deg]:%.2f %.2f %.2f %.2f %.2f %.2f\r",
    	                      time, display_svo.RefTheta.t[0]*Rad2Deg, display_svo.RefTheta.t[1]*Rad2Deg,
    	                      display_svo.RefTheta.t[2]*Rad2Deg,  display_svo.RefTheta.t[3]*Rad2Deg,
    	                      display_svo.RefTheta.t[4]*Rad2Deg,  display_svo.RefTheta.t[5]*Rad2Deg,
    	   				   jnt_angle[0],jnt_angle[1],jnt_angle[2],jnt_angle[3],jnt_angle[4],jnt_angle[5]);
    	   }
       usleep(25000); //delay for 25 microseconds
    }while(shm_servo_inter.status_control!=EXIT_C);
    printf("\nEnd of Display Function\n");
    return ((void*)0);
}

void *interface_function(void* param)
{

    int interface_counter;
    interface_counter=0;
    int end = 1;
    int command,i;

    SVO interface_svo;

    MATRIX_D A = MatD31(1,2,3), B=MatD31(4,5,6);
    MATRIX_D testp = MatD61(0,0,0,0,0,0);
    MATRIX_D testOri = Zeros(3,3);
    double c;
    std::vector<double> cur_jnt_ang(6);
    std::vector<double> test_com_ang(6);
    printf("Executing interface function\n");

    DisplayMenu();

      do
    {

        // Display the current of the robot:
        // Wait command
        printf("Please hit any key\n");

        command = getchar();
        switch(command) {
        	case 'c':
        	case 'C':
        			testp= ur_kinematics(&(pSVO.jnk), testOri);//求出的结果是末端的位置Matrix61
        			printf("Current position of hand:\n");
        			printf("POS::%.2f[m], %.2f[m], %.2f[m], %.2f[deg], %.2f[deg], %.2f[deg]\n",
        				testp(1,1),
						testp(2,1),
						testp(3,1),
						testp(4,1)*Rad2Deg,
						testp(5,1)*Rad2Deg,
						testp(6,1)*Rad2Deg);
        			break;
        	case 'g': // gain setting
        	case 'G':
            //ChangeGain(&interface_svo.Gain);
        			break;
        	case 'p': // joint reference setting
        	case 'P':
        			printf("-----------------Now you are in JntSvoMode------------------\n");
        			printf("Set the path frequency,path mode and goal joint position");
        			interface_svo.PosOriServoFlag=OFF;
        			ChangePathData(&interface_svo.Path);
        			break;
        	case 's':
        	case 'S':
            		printf("Start\n");
            		if(interface_svo.PosOriServoFlag==ON)
            			SetPosOriSvo(&interface_svo);
            		else
            			SetJntSvo(&interface_svo);
            		break;
        	case 'i':
        	case 'I': // Show the information of robot
        				DisplayCurrentInformation(interface_svo.Path,interface_svo.PosOriServoFlag,interface_svo.Gain);
            		break;
        	case 'd':
        	case 'D': 	// Demo function
            		//JntDemoFunction(&interface_svo);
        			printf("----------------Now you are in PosOriServoMode!------------------");
					PosOriServo(&interface_svo.PosOriServoFlag);
        		    ChangeHandData(&interface_svo.Path);
        			break;
        	case 'e':
        	case 'E':
            		end = 0;
            		shm_servo_inter.status_control=EXIT_C;
            		break;

        	default:
        			DisplayMenu();// DisplayCurrentInformation(interface_svo.Path, interface_svo.Gain);
        			break;
        	}
        	interface_counter++;
    }while(end);
    sleep(5);
    printf("End of Experiment\n");
}

void DisplayMenu(void)
{
    printf("\n**************Menu (please input [CR])*****************\n");
    printf("CurPosOfHnd:*************[c : C]\n");
    printf("Gain:********************[g : G]\n");
    printf("Path:********************[p : P]\n");
    printf("Start:*******************[s : S]\n");
    printf("Info:********************[i : I]\n");
    printf("Destination:*************[d : D]\n");
    printf("End:*********************[e : E :ESC]\n");
}

void DisplayCurrentInformation(PATH path,int flag,GAIN gain)
{
        printf("---------------------- Current Information ------------------------\n");
        printf("Path frequency = %f [Hz]\n", path.Freq);
        switch(path.Mode)
        {
	case 0:
	    printf("Path mode: Sin\n");
 	    break;
        case 1:
            printf("Path mode: 5JI\n");
 	    break;
        case 2:
            printf("Path mode: 3JI\n");
 	    break;
        case 3:
            printf("Path mode: 1JI\n");
 	    break;
        case 4:
            printf("Path mode: Step\n");
 	    break;
        default:
            printf("Error path mode\n");
        }
    /*printf("K_vt = %f, %f, %f, %f, %f, %f, %f\n", gain.K_vt[0], gain.K_vt[1], gain.K_vt[2],
            gain.K_vt[3], gain.K_vt[4],gain.K_vt[5], gain.K_vt[6]);
	printf("K_pt = %f, %f, %f, %f, %f, %f, %f\n", gain.K_pt[0], gain.K_pt[1], gain.K_pt[2],
            gain.K_pt[3], gain.K_pt[4],gain.K_vt[5], gain.K_vt[6]);
        printf("K_it = %f, %f, %f, %f, %f, %f, %f\n", gain.K_it[0], gain.K_it[1], gain.K_it[2],
            gain.K_it[3], gain.K_it[4],gain.K_it[5], gain.K_it[6]);
	*/
    if(flag==OFF)
    {
	printf("Current Joint angle[deg]:%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
		   pSVO.CurTheta.t[0] * Rad2Deg,
		   pSVO.CurTheta.t[1] * Rad2Deg,
		   pSVO.CurTheta.t[2] * Rad2Deg,
           pSVO.CurTheta.t[3] * Rad2Deg,
           pSVO.CurTheta.t[4] * Rad2Deg,
           pSVO.CurTheta.t[5] * Rad2Deg);

	printf("Goal Joint Angle[Deg]:%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
		   path.Goal[0] * Rad2Deg,
		   path.Goal[1] * Rad2Deg,
           path.Goal[2] * Rad2Deg,
           path.Goal[3] * Rad2Deg,
           path.Goal[4] * Rad2Deg,
           path.Goal[5] * Rad2Deg);
    printf("------------------------------------------------------------------\n");
    }
    else{
    	printf("Current Position of hand:\nX[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[deg]\n");
    	printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
    			pSVO.CurPos.t[0],pSVO.CurPos.t[1],pSVO.CurPos.t[2],
				pSVO.CurPos.t[3] * Rad2Deg,pSVO.CurPos.t[4] * Rad2Deg,pSVO.CurPos.t[5] * Rad2Deg);

    	printf("Goal Position of hand:\nX[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[deg]\n");
    	printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
    			path.Goal[0],path.Goal[1],path.Goal[2],
				path.Goal[3] * Rad2Deg,path.Goal[4] * Rad2Deg,path.Goal[5] * Rad2Deg);
    	printf("--------------------------------------------------------------------------\n");
    }
}



