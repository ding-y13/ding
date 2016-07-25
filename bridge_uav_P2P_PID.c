/*********************************************************************
*filename: bridge_uav_P2P_PID.c
*purpose: OptiTrack+FFData, ���Ʒ���5s ֱ��һ�׵�Բ
*date time:2016-07-07 10:45:00
*Author: James Huang  15010130829 12997876@qq.com
./bridge_uav_P2P_PID ffsetting.conf
*********************************************************************/
#include "inc/flight_control.h"
//#include "inc/adspmsgd.h"
//#include "inc/rpcmem.h"

//#include "inc/Queue.h"


#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <errno.h>
#include <float.h>
#include <math.h>

#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>


#include <sys/time.h> //struct itimerval, setitimer()



//����
#define QUEUE_SIZE 15
#define CLOCKID CLOCK_REALTIME  

#define DELTA_S 0.01


//#include "inc/rpcmem.h"
#include "inc/ZeroTech_UAV_SDK.h"
#include <pthread.h>

#define BUFLEN 1000
#define N_RIGIDBODY 4
#define TRAJ_GENERATOR_TIME 3000   	// 3s
#define POS_CAPTURE_TIME	1	   	// 1ms,1000fps
#define PID_CTRL_TIME  	 	30		// 30ms,33fps
#define MAX_RB_NUMBER       5		
//#define TGTID				2		// Ŀ�������
//#define CTRL_STICK_ID       1		// ���ư�������

typedef struct{
	float P;		// P����
	float I;		// I����
	float D;		// D����
}PID_PARA;

#define MAX_DATA 8000

	typedef struct{
		double fps;
		double optiT,optiTL,dt;
		double uc[4];
		double uo[4];
		double v[4];
	}Info_Check;

Info_Check info_check[MAX_DATA];	

typedef struct{
	unsigned int UAV_ID;		// ����ID
	unsigned int UAV_FRONT_ID;  //ǰһ�ܷɻ���ID
	char TRACK_FILENAME[30];     //
	char SAVE_FILENAME[30];
	unsigned int READ_MODE;    //
	//char FFDATA_PATH[300];		// ���нű�·��
	PID_PARA PID_PARAS;			// ����PID���Ʋ���
}FFCONFIG;

FFCONFIG ffconf;
//------------------------------------------

typedef struct _RigidBodyData {
   int  id;
   float x;
   float y;
   float z;
   float a;
   float b;
   float c;
   float d;
}RigidBodyData_t;

unsigned int UAV_FRONT_ID;				// ǰһ�ܷɻ����
unsigned int tgtID;
char *trackFilename;
char *saveFilename;
unsigned int readMode;
PID_PARA PID_para;
int Info_Check_Number = 0;

RigidBodyData_t PositionBuf[MAX_DATA];
int PositionBufNumber = 0;
typedef struct _OptitrackData {
   double time_stamp;
   int n_rigidbody;
   RigidBodyData_t rigidbody[N_RIGIDBODY];
}OptitrackData_t;

RigidBodyData_t ObjRigidBody;//Ŀ��opt����
pthread_mutex_t Objmtx = PTHREAD_MUTEX_INITIALIZER;	//Opt������

static OptitrackData_t OptitrackData;				//opt�鲥���մ��
static OptitrackData_t OptitrackDataLast;			//opt�鲥���մ���ϴ�
pthread_mutex_t Optmtx = PTHREAD_MUTEX_INITIALIZER;	//Opt������
pthread_mutex_t Quemtx = PTHREAD_MUTEX_INITIALIZER; //����Queue������
timer_t trajGen_timerid; 							//�켣��������timer



///////////////////////////////////////////////////

// #define FLT_EPSILON 1.192092896e-07f

typedef struct {float x, y, z, w;} Quat; /* Quaternion */
enum QuatPart {X, Y, Z, W};
typedef Quat EulerAngles;    /* (x,y,z)=ang 1,2,3, w=order code  */

// note:
/* ogl matrix representation (column-vectors):

| 1 0 0 x |  
| 0 1 0 y |
| 0 0 1 z |
| 0 0 0 1 |

= 

| [0] [4] [8]  [12] |  
| [1] [5] [9]  [13] |
| [2] [6] [10] [14] |
| [3] [7] [11] [15] |
*/
typedef float HMatrix[4][4]; /* Right-handed, for column vectors */


/*** Order type constants, constructors, extractors ***/
/* There are 24 possible conventions, designated by:    */
/*	  o EulAxI = axis used initially		    */
/*	  o EulPar = parity of axis permutation		    */
/*	  o EulRep = repetition of initial axis as last	    */
/*	  o EulFrm = frame from which axes are taken	    */
/* Axes I,J,K will be a permutation of X,Y,Z.	    */
/* Axis H will be either I or K, depending on EulRep.   */
/* Frame S takes axes from initial static frame.	    */
/* If ord = (AxI=X, Par=Even, Rep=No, Frm=S), then	    */
/* {a,b,c,ord} means Rz(c)Ry(b)Rx(a), where Rz(c)v	    */
/* rotates v around Z by c radians.			    */
#define EulFrmS	     0
#define EulFrmR	     1
#define EulFrm(ord)  ((unsigned)(ord)&1)
#define EulRepNo     0
#define EulRepYes    1
#define EulRep(ord)  (((unsigned)(ord)>>1)&1)
#define EulParEven   0
#define EulParOdd    1
#define EulPar(ord)  (((unsigned)(ord)>>2)&1)
#define EulSafe	     "\000\001\002\000"
#define EulNext	     "\001\002\000\001"
#define EulAxI(ord)  ((int)(EulSafe[(((unsigned)(ord)>>3)&3)]))
#define EulAxJ(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)==EulParOdd)]))
#define EulAxK(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)!=EulParOdd)]))
#define EulAxH(ord)  ((EulRep(ord)==EulRepNo)?EulAxK(ord):EulAxI(ord))
/* EulGetOrd unpacks all useful information about order simultaneously. */
#define EulGetOrd(ord,i,j,k,h,n,s,f) {unsigned o=ord;f=o&1;o>>=1;s=o&1;o>>=1;\
    n=o&1;o>>=1;i=EulSafe[o&3];j=EulNext[i+n];k=EulNext[i+1-n];h=s?k:i;}
/* EulOrd creates an order value between 0 and 23 from 4-tuple choices. */
#define EulOrd(i,p,r,f)	   (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
/* Static axes */
#define EulOrdXYZs    EulOrd(X,EulParEven,EulRepNo,EulFrmS)
#define EulOrdXYXs    EulOrd(X,EulParEven,EulRepYes,EulFrmS)
#define EulOrdXZYs    EulOrd(X,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdXZXs    EulOrd(X,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdYZXs    EulOrd(Y,EulParEven,EulRepNo,EulFrmS)
#define EulOrdYZYs    EulOrd(Y,EulParEven,EulRepYes,EulFrmS)
#define EulOrdYXZs    EulOrd(Y,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdYXYs    EulOrd(Y,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdZXYs    EulOrd(Z,EulParEven,EulRepNo,EulFrmS)
#define EulOrdZXZs    EulOrd(Z,EulParEven,EulRepYes,EulFrmS)
#define EulOrdZYXs    EulOrd(Z,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdZYZs    EulOrd(Z,EulParOdd,EulRepYes,EulFrmS)
/* Rotating axes */
#define EulOrdZYXr    EulOrd(X,EulParEven,EulRepNo,EulFrmR)
#define EulOrdXYXr    EulOrd(X,EulParEven,EulRepYes,EulFrmR)
#define EulOrdYZXr    EulOrd(X,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdXZXr    EulOrd(X,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdXZYr    EulOrd(Y,EulParEven,EulRepNo,EulFrmR)
#define EulOrdYZYr    EulOrd(Y,EulParEven,EulRepYes,EulFrmR)
#define EulOrdZXYr    EulOrd(Y,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdYXYr    EulOrd(Y,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdYXZr    EulOrd(Z,EulParEven,EulRepNo,EulFrmR)
#define EulOrdZXZr    EulOrd(Z,EulParEven,EulRepYes,EulFrmR)
#define EulOrdXYZr    EulOrd(Z,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdZYZr    EulOrd(Z,EulParOdd,EulRepYes,EulFrmR)






//����

  
  
typedef struct SeqQueue  
{  
    RigidBodyData_t data[QUEUE_SIZE];  
    int front;  
    int rear;  
  
}QUEUE;  
  
/* QUEUE *InitQueue()  
{  
    QUEUE *Queue = (QUEUE *)malloc(sizeof(QUEUE));  
    if(Queue == NULL)  
    {  
        printf("Malloc failed!\n");  
        exit(-1);  
    }  
    Queue->front = 0;  
    Queue->rear = 0;  
  
    return Queue;  
}  
   */
int IsFull(QUEUE *Queue)  
{  
    return ((Queue->rear+1)%QUEUE_SIZE == Queue->front);  
}  
  
int IsEmpty(QUEUE *Queue)  
{  
    return (Queue->front == Queue->rear);  
}  
  
void EnQueue(QUEUE *Queue,RigidBodyData_t uav_front)  
{  
    if(IsFull(Queue))  
    {  
        return;  
    }  
    Queue->data[Queue->rear] = uav_front;  
    Queue->rear = (Queue->rear+1)%QUEUE_SIZE;  
}  
  
RigidBodyData_t DeQueue(QUEUE *Queue)  
{  

    RigidBodyData_t tmp = Queue->data[Queue->front];  
    Queue->front = (Queue->front+1)%QUEUE_SIZE;  
    return tmp;  
} 
void EmptyQueue(QUEUE *Queue)
{
	Queue->rear = 0;
	Queue->front = Queue->rear;
	pthread_mutex_lock( & Objmtx );//����
	ObjRigidBody.x = 1;
	ObjRigidBody.y = 0;
	ObjRigidBody.z = 1;
	printf("initialize the position data!\n");
	pthread_mutex_unlock( & Objmtx );//����
	
} 




QUEUE Queue[1];















float RadiansToDegrees(float fRadians)
{
    return fRadians * (180.0F / 3.14159265F);
}

/* Convert matrix to Euler angles (in radians). */
EulerAngles Eul_FromHMatrix(HMatrix M, int order)
{
    EulerAngles ea;
    int i,j,k,h,n,s,f;
    EulGetOrd(order,i,j,k,h,n,s,f);
    if (s==EulRepYes) {
        double sy = sqrt(M[i][j]*M[i][j] + M[i][k]*M[i][k]);
        if (sy > 16*FLT_EPSILON) {
            ea.x = atan2((double)M[i][j], (double)M[i][k]);
            ea.y = atan2(sy, (double)M[i][i]);
            ea.z = atan2(M[j][i], -M[k][i]);
        } else {
            ea.x = atan2(-M[j][k], M[j][j]);
            ea.y = atan2(sy, (double)M[i][i]);
            ea.z = 0;
        }
    } else {
        double cy = sqrt(M[i][i]*M[i][i] + M[j][i]*M[j][i]);
        if (cy > 16*FLT_EPSILON) {
            ea.x = atan2(M[k][j], M[k][k]);
            ea.y = atan2((double)-M[k][i], cy);
            ea.z = atan2(M[j][i], M[i][i]);
        } else {
            ea.x = atan2(-M[j][k], M[j][j]);
            ea.y = atan2((double)-M[k][i], cy);
            ea.z = 0;
        }
    }
    if (n==EulParOdd) {ea.x = -ea.x; ea.y = - ea.y; ea.z = -ea.z;}
    if (f==EulFrmR) {float t = ea.x; ea.x = ea.z; ea.z = t;}
    ea.w = order;
    return (ea);
}

/* Convert quaternion to Euler angles (in radians). */
EulerAngles Eul_FromQuat(Quat q, int order)
{
    HMatrix M;
    double Nq = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
    double xs = q.x*s,	  ys = q.y*s,	 zs = q.z*s;
    double wx = q.w*xs,	  wy = q.w*ys,	 wz = q.w*zs;
    double xx = q.x*xs,	  xy = q.x*ys,	 xz = q.x*zs;
    double yy = q.y*ys,	  yz = q.y*zs,	 zz = q.z*zs;
    M[X][X] = 1.0 - (yy + zz); M[X][Y] = xy - wz; M[X][Z] = xz + wy;
    M[Y][X] = xy + wz; M[Y][Y] = 1.0 - (xx + zz); M[Y][Z] = yz - wx;
    M[Z][X] = xz - wy; M[Z][Y] = yz + wx; M[Z][Z] = 1.0 - (xx + yy);
    M[W][X]=M[W][Y]=M[W][Z]=M[X][W]=M[Y][W]=M[Z][W]=0.0; M[W][W]=1.0;
    return (Eul_FromHMatrix(M, order));
}

////////////////////////////////////////////////////////////////////////
/*
    [RHS, Y-Up]     [RHS, Z-Up]

                          Y
     Y                 Z /
     |__ X             |/__ X
     /
    Z
*/
#define MATH_PI 3.14159265F


void ConvertRHSRotZUpToYUp(float *qx, float *qy, float *qz, float *qw)
{
    // 90 deg rotation about +X ��ʱ����ת90 [90 0 0]
    float qRx, qRy, qRz, qRw;
    float angle = 90.0f * MATH_PI / 180.0f;
    qRx = sin(angle / 2.0f);
    qRy = 0.0f;
    qRz = 0.0f;
    qRw = cos(angle / 2.0f);

    // rotate quat using quat multiply
    float qxNew, qyNew, qzNew, qwNew;
    qxNew = (*qw)*qRx + (*qx)*qRw + (*qy)*qRz - (*qz)*qRy;
    qyNew = (*qw)*qRy - (*qx)*qRz + (*qy)*qRw + (*qz)*qRx;
    qzNew = (*qw)*qRz + (*qx)*qRy - (*qy)*qRx + (*qz)*qRw;
    qwNew = (*qw)*qRw - (*qx)*qRx - (*qy)*qRy - (*qz)*qRz;

    *qx = qxNew;
    *qy = qyNew;
    *qz = qzNew;
    *qw = qwNew;
}


void ConvertRHSRotYUpToZUp(float *qRx, float *qRy, float *qRz, float *qRw)
{
/*  // -90 deg rotation about +X ��ʱ����ת-90  [-90 0 0]
    float qRx, qRy, qRz, qRw;
    float angle = -90.0f * MATH_PI / 180.0f;
    qRx = sin(angle / 2.0f);
    qRy = 0.0f;
    qRz = 0.0f;
    qRw = cos(angle / 2.0f);
*/
	float qw,qx,qy,qz;	
	
	// [-90,0,0]ΪR��R���������
	qw = 0.707106781;
	qx = 0.707106781;
	qy = 0;
	qz = 0;
    // rotate quat using quat multiply
    float qxNew, qyNew, qzNew, qwNew;
    qxNew = qw*(*qRx) + qx*(*qRw) + qy*(*qRz) - qz*(*qRy);
    qyNew = qw*(*qRy) - qx*(*qRz) + qy*(*qRw) + qz*(*qRx);
    qzNew = qw*(*qRz) + qx*(*qRy) - qy*(*qRx) + qz*(*qRw);
    qwNew = qw*(*qRw) - qx*(*qRx) - qy*(*qRy) - qz*(*qRz);

    *qRx = qxNew;
    *qRy = qyNew;
    *qRz = qzNew;
    *qRw = qwNew;
}

void ConvertRHSRotUAVNEEToUAVENS(float *qx, float *qy, float *qz, float *qw)
{
    // �ɻ�����ϵ�� "������" -> "������"   [180, 0, -90]
    //     
    //     [RHS, Z-Down]     [RHS, Z-Up]   
    //       
    //       
    //          X              Y  						
    //         /            Z /   						
    //        /__ Y         |/__ X						
    //       |               										
    //       |               										
    //       Z               										
    //
	            
    // 90 deg rotation about +X ��ʱ����תΪ��
    float qRx, qRy, qRz, qRw;
/*    float angle = 90.0f * MATH_PI / 180.0f;
    qRx = sin(angle / 2.0f);
    qRy = 0.0f;
    qRz = 0.0f;
    qRw = cos(angle / 2.0f);
*/
	qRw = 0.0;
	qRx = 0.707106781;
	qRy = 0.707106781;
	qRz = 0.0;
	 

    // rotate quat using quat multiply
    float qxNew, qyNew, qzNew, qwNew;
    qxNew = (*qw)*qRx + (*qx)*qRw + (*qy)*qRz - (*qz)*qRy;
    qyNew = (*qw)*qRy - (*qx)*qRz + (*qy)*qRw + (*qz)*qRx;
    qzNew = (*qw)*qRz + (*qx)*qRy - (*qy)*qRx + (*qz)*qRw;
    qwNew = (*qw)*qRw - (*qx)*qRx - (*qy)*qRy - (*qz)*qRz;

    *qx = qxNew;
    *qy = qyNew;
    *qz = qzNew;
    *qw = qwNew;
}

Quat ConvertOptiTrackSystem2ENSSystem(Quat q)
{
	int upAxis = 1;//default: Y-up
	float qx = q.x;
	float qy = q.y;
	float qz = q.z;
	float qw = q.w;	
	
	//printf("before: qw=%f, qx=%f, qy=%f, qz=%f\n", qw,qx,qy,qz);
	
	// If Motive is streaming Y-up, convert to this renderer's Z-up coordinate system
	if(upAxis==1)
	{
		// �任�ɻ�����ϵ�������� -> ������
		ConvertRHSRotUAVNEEToUAVENS(&qx, &qy, &qz, &qw);
		// printf("after1: qw=%f, qx=%f, qy=%f, qz=%f\n", qw,qx,qy,qz);
		
		// �任�������ϵ��Y-Up -> Z-Up
		ConvertRHSRotYUpToZUp(&qx, &qy, &qz, &qw); 
	}
	
	// If Motive is streaming Z-up, convert to this renderer's Y-up coordinate system
/*	if (upAxis==2)
	{
		ConvertRHSRotZUpToYUp(&qx, &qy, &qz, &qw);
	}
*/	
	q.x = qx;
	q.y = qy;
	q.z = qz;
	q.w = qw;
	// printf("after2: qw=%f, qx=%f, qy=%f, qz=%f\n", qw,qx,qy,qz);
	
	return q;
}

EulerAngles Quat2Euler(Quat q)
{	
	EulerAngles ea;

	// Convert Motive quaternion output to euler angles
	// Motive coordinate conventions : X(Pitch), Y(Yaw), Z(Roll), Relative, RHS
	int order = EulOrdXYZr;
	
	ea = Eul_FromQuat(q, order);

	ea.x = RadiansToDegrees(ea.x);
	ea.y = RadiansToDegrees(ea.y);
	ea.z = -RadiansToDegrees(ea.z);
	//printf("Pitch=%f, Roll=%f, Yaw=%f ",ea.x, ea.y, ea.z);
	
	return ea;
}
		
/////////////////////////////////////////////////////////////////

#define SUCCESS 0
static volatile int LoopFlag = 1;
static volatile int FlyFlag = 0;
static volatile int ControlMode=2;  //Ĭ�ϸ���ָ�Ӱ�
void loop_ctrl(int sig)
{
	if(sig==SIGRTMIN)
	{   
	   /* while(takeoff() != SUCCESS)
		{
			takeoff();
		} 
        sleep(15);*/
		
		
		if(enable_action_control_cmd_to_ZeroTech_UAV_SDK()!= 0)
		{
			printf("enable_action_control_cmd_to_ZeroTech_UAV_SDK failed!\n");
		} 
        FlyFlag = 1;
		
		printf("Takeoff Start!!!\n");
	}
	if(sig==SIGRTMIN+1)
	{
		
		
		
		if(disable_action_control_cmd_to_ZeroTech_UAV_SDK()!= 0)
		{
			printf("enable_action_control_cmd_to_ZeroTech_UAV_SDK failed!\n");
		}	
		FlyFlag = 0;
		pthread_mutex_lock( & Quemtx );
		EmptyQueue(Queue);
		pthread_mutex_unlock( & Quemtx );
		printf("Auto Land!!!\n");
	}
	if(sig==SIGRTMIN+2)
	{
		/* int temp1 = 0;
		printf("please input the way you desire:");
		scanf("%d",&temp1);
		while(temp1 != 1 && temp1 != 2)
		{
			printf("your input is wrong, please input again:");
			scanf("%d",&temp1);
		} */
		change_track();
	}
	if(sig==SIGINT)
	{
		LoopFlag = 0;
		printf("Press Ctrl+C!!!\n");
	}
	/* if(sig==SIGRTMIN+2)
	{
		ControlMode = 1;
		printf("PositionBuf!!!\n");
	}
	if(sig==SIGRTMIN+3)
	{
		ControlMode = 2;
		printf("Ctrlstick!!!\n");
	} */
}

void mysend(flight_control_OptiTrackData* pdata)
{   
	// ��Ϊ��UDP�׽��֣�ʹ��SOCK_DGRAM,������SOCK_STREAM
	flight_control_ap_send_optitrack_data(pdata);
	
}

int Available(RigidBodyData_t uav_front,RigidBodyData_t uav_front_last){
	//printf("this is Available!\n");
	if(fabs(uav_front.x-uav_front_last.x)<DELTA_S&&fabs(uav_front.y-uav_front_last.y)<DELTA_S&&fabs(uav_front.z-uav_front_last.z)<DELTA_S)
	{
		printf("dx = %lf   %lf    %lf\n",fabs(uav_front.x-uav_front_last.x),fabs(uav_front.y-uav_front_last.y),fabs(uav_front.z-uav_front_last.z));
		printf("unavailable!!!\n");
		return 0;
	}
	else{
		printf("dx = %lf   %lf    %lf\n",fabs(uav_front.x-uav_front_last.x),fabs(uav_front.y-uav_front_last.y),fabs(uav_front.z-uav_front_last.z));
		printf("Available!!\n");
		return 1;
	}
	
}

double GetFPS()
{
	static int    cnt = 0;			// ������
	static double fps = 0.0;

	static struct timeval t_start={0,0},t_end={0,0};
	long delta_s;
	long delta_us;
	float delta_time;

	// ��ú�ʱ
	gettimeofday (&t_end, NULL);
	delta_s  = t_end.tv_sec - t_start.tv_sec;
	delta_us = t_end.tv_usec - t_start.tv_usec;

	delta_time = delta_s + delta_us * 0.000001f;

	// If time - last time is > than 1, save fps.
	if(delta_time > 1.0f)
	{
		// Calculate FPS.
		fps = (cnt+1)/(delta_time);
		
		// Reset the FPS counter.
		cnt = 0;

		// Record last time.
		t_start = t_end;
	}
	else
	{
		// Add to the frames.
		cnt++;
	}
	//printf(" FPS:%.2f\n",fps);
	return fps;
}


// Timer Call Back
void timer_thread(union sigval v)  
{  
	//printf("this is timer_thread\n");
	struct timeval  tv;
	struct timezone tz;
	static int iCnt = 0;  
	gettimeofday(&tv,&tz);
	//printf("SignalInt %d: The %d Times, tv:%ld.%06ld\n", v.sival_int,iCnt++,tv.tv_sec,tv.tv_usec);
	
	static unsigned int Counter = 0;
	//unsigned int row_cnt = sizeof(PositionBuf)/sizeof(PositionBuf[0]);

	// 33ms��ʱ���ɲ����͹켣
	if(FlyFlag) 
	{  
		if (ControlMode==2)
		{
			pthread_mutex_lock( & Quemtx ); //��������
			pthread_mutex_lock(&Objmtx);	//����
			//������ʱ�ŷ�������
			if(IsFull(Queue))
			{
				ObjRigidBody = DeQueue( Queue );
			}
			printf("timer_thread ObjRigidBody:%f   %f   %f\n",ObjRigidBody.x,ObjRigidBody.y,ObjRigidBody.z);
			pthread_mutex_unlock(&Objmtx);
			pthread_mutex_unlock( & Quemtx );//���н���
		}
		if (ControlMode==1)
		{
			pthread_mutex_lock( & Objmtx );//����
			if(Counter<PositionBufNumber)
			{
				ObjRigidBody = PositionBuf[ Counter];
				Counter ++ ;
			}
			else if(readMode==1)
			{
				ObjRigidBody = PositionBuf[Counter-1];
			}
			else if(readMode==2)
			{
				ObjRigidBody = PositionBuf[0];
				Counter = 0;
			}
			/* if(Counter % row_cnt == 0)
			printf("---------------------------------------------\n");
	 		printf("timer_thread ObjRigidBody:%f   %f   %f\n",ObjRigidBody.x,ObjRigidBody.y,ObjRigidBody.z); */
			pthread_mutex_unlock( & Objmtx );//����
		}
	// Counter ++ ;
	// printf("FPS:%.2f xyz [%5.2f,%5.2f,%5.2f] %d %d\n",GetFPS(),ObjRigidBody.x,ObjRigidBody.y,ObjRigidBody.z, Counter, row_cnt);
	
	}	
	
	// ֹͣ��ʱ��
	if(LoopFlag==0)
	{
		timer_delete(trajGen_timerid);
		printf("timer stoped!\n");
	}
} 


static void* TrajGeneratorPthread(void* arg)
{
	/*unsigned int Counter = 0;
	unsigned int row_cnt = sizeof(PositionBuf)/sizeof(PositionBuf[0]);
	*/
	////////////////////////////////////////////////////
	// TIMER SETTINGS
	// XXX int timer_create(clockid_t clockid, struct sigevent *evp, timer_t *timerid);  
    // clockid--ֵ��CLOCK_REALTIME, CLOCK_MONOTONIC, CLOCK_PROCESS_CPUTIME_ID, CLOCK_THREAD_CPUTIME_ID
    // evp--��Ż���ֵ�ĵ�ַ,�ṹ��Ա˵���˶�ʱ�����ڵ�֪ͨ��ʽ�ʹ���ʽ��  
    // timerid--��ʱ����ʶ��  
     
    struct sigevent evp;  	
	
    memset(&evp, 0, sizeof(struct sigevent));       //�����ʼ��  
  
    evp.sigev_value.sival_int = 111;            	//Ҳ�Ǳ�ʶ��ʱ���ģ����timerid��ʲô���𣿻ص��������Ի��  
    evp.sigev_notify = SIGEV_THREAD;            	//�߳�֪ͨ�ķ�ʽ����פ���߳�  
    evp.sigev_notify_function = timer_thread;       //�̺߳�����ַ  
  
    if (timer_create(CLOCKID, &evp, &trajGen_timerid) == -1)  
    {  
        perror("fail to timer_create");  
        exit(-1);  
    }
		
  
    // XXX int timer_settime(timer_t timerid, int flags, const struct itimerspec *new_value,struct itimerspec *old_value);  
    // timerid--��ʱ����ʶ  
    // flags--0��ʾ���ʱ�䣬1��ʾ����ʱ��  
    // new_value--��ʱ�����³�ʼֵ�ͼ�����������it  
    // old_value--ȡֵͨ��Ϊ0�������ĸ�������ΪNULL,����ΪNULL���򷵻ض�ʱ����ǰһ��ֵ  
      
    //��һ�μ��it.it_value��ô��,�Ժ�ÿ�ζ���it.it_interval��ô��,����˵it.it_value��0��ʱ���װ��it.it_interval��ֵ  
    struct itimerspec it;  
    it.it_interval.tv_sec  = 0;  
    it.it_interval.tv_nsec = 33333333;//33333333;  //33 ms
    it.it_value.tv_sec  = 1;  
    it.it_value.tv_nsec = 0;  
  
    if (timer_settime(trajGen_timerid, 0, &it, NULL) == -1)  
    {  
        perror("fail to timer_settime");  
        exit(-1);  
    }  
	/////////////////////////////////////////////////////

	// ѭ���������ɵĹ켣
	while(LoopFlag==1)
	{
/*		pthread_mutex_lock( & Objmtx );//����
		ObjRigidBody = PositionBuf[ Counter % row_cnt ];
		pthread_mutex_unlock( & Objmtx );//����
		Counter ++ ;
*/
		usleep( 1000 * TRAJ_GENERATOR_TIME);//about 3s
	}
	return (void *)0;
}


 float ErrorAngleNormalize(float a)
 {
    // make a into (-180,180]
 	if(a>180)
		a -= 360;
	if(a<=-180)
		a += 360; 
 	return a;
 }

//λ�ø����߳�
static void* PIDCtrlPthread(void* arg)
{
	double delta_v = 0;
	printf("this is PIDCtrl\n");
	
	static unsigned int Counter;
	
	
	RigidBodyData_t Ctrlstick;	// ���ư���������								
	RigidBodyData_t uav_front;  // ǰһ���ɻ�����
	static RigidBodyData_t CtrlstickLast;							// ���ư��ϴ�����
	static RigidBodyData_t uav_front_last;							// ǰһ���ɻ��ϴ�����
	RigidBodyData_t uav_obj;										// Ŀ����������
	RigidBodyData_t uav_curr;										// Ŀ�걾������
	static RigidBodyData_t uav_last;								// Ŀ���ϴ�����
	sdk_fly_status_t fly_status_data;
	sdk_target_speed_t target_speed;
	double CurrYaw, CurrRoll, CurrPitch;
	double ObjYaw, ObjRoll, ObjPitch;
	double CSv_x, CSv_y, CSv_z;												// ���ư��ٶ�
	double TGTv_x, TGTv_y, TGTv_z;											// Ŀ���ٶ�
	double XErr = 0,YErr = 0,ZErr = 0, YawErr = 0;				 			//����ƫ��
	static double LastXErr = 0,LastYErr = 0,LastZErr = 0,LastYawErr = 0; 	//�ϴ�ƫ��
	double XErrDiff = 0,YErrDiff = 0,ZErrDiff = 0,YawErrDiff = 0; 			//ƫ��΢��
	double XErrSum = 0,YErrSum = 0,ZErrSum = 0,YawErrSum = 0;	 			//ƫ�����
	
	static double Xout = 0, Yout = 0, Zout = 0, YawOut = 0;					//PID���������ֵ
	
	double X_P = PID_para.P, X_I = PID_para.I, X_D = PID_para.D;						//x��PID��������
	double Y_P = PID_para.P, Y_I = PID_para.I, Y_D = PID_para.D;						//y��PID��������
	double Z_P = PID_para.P, Z_I = PID_para.I, Z_D = PID_para.D;						//z��PID��������
	double Yaw_P = 0.0, Yaw_I = 1/DBL_MAX, Yaw_D = 0.0;			//����PID�������ԣ���Ҫ�Ǳ�֤���Ƶ�X,Y,Z����optitrack��X,Y,Z������ͬ
	
	int s = 0;
	int i,idx;
	double optTimeStamp;
	static double optTimeStampLast;
	double dt,invdt;
	OptitrackData_t TmpOptiData;
	flight_control_OptiTrackData optitrack_data;
	//unsigned int tgtID = ffconf.ID;

	/*	
	X_P = Y_P = Z_P = ffconf.PID_PARAS.P;
	X_I = Y_I = Z_I = ffconf.PID_PARAS.I;
	X_D = Y_D = Z_D = ffconf.PID_PARAS.D;
	*/
	
/* 	printf("================================================");
	printf("X_P = %f, Y_P = %f, Z_P = %f\n", X_P, Y_P, Z_P);
	printf("X_I = %f, Y_I = %f, Z_I = %f\n", X_I, Y_I, Z_I);
	printf("X_D = %f, Y_D = %f, Z_D = %f\n", X_D, Y_D, Z_D);
	printf("================================================");
     */
	
	//����PID����
	/* ѭ�����������������鲥��Ϣ */
	while(LoopFlag==1)
	{
		////�����ȡĿ��λ������
		pthread_mutex_lock( & Objmtx );//����
		{
			uav_obj.x = ObjRigidBody.x;
			uav_obj.y = ObjRigidBody.y;
			uav_obj.z = ObjRigidBody.z;			
			
			// [ObjPitch,ObjRoll,ObjYaw] = quat2angle(ObjRigidBody.a,b,c,d, 'xyz');
			ObjYaw = ObjRigidBody.a;
		}
		pthread_mutex_unlock( & Objmtx );//����
		
		//printf("PID control module...\n");

		// �����ȡ��ǰλ�����ݵ�TmpOptiData
		pthread_mutex_lock( & Optmtx );//����
		{
			flight_control_ap_get_optitrack_data(&optitrack_data);
			/* printf("OPTI��%f %d %d %d, %f %f %f\n",optitrack_data.time_stamp, 
											optitrack_data.n_rigidbody, 
											optitrack_data.rigidbody[0].id, 
											optitrack_data.rigidbody[0].cmd, 
											optitrack_data.rigidbody[0].x,
											optitrack_data.rigidbody[0].y,
											optitrack_data.rigidbody[0].z); */
			//memcpy(&TmpOptiData, &OptitrackData, sizeof(OptitrackData_t));
		}
		pthread_mutex_unlock( & Optmtx );//����
		
		//printf("optitrack\n");
		
		// ��TmpOptiData��ȡĿ������
		{
			for(i=0;i<MAX_RB_NUMBER;i++)
			{
				//printf("i :%d id :%d\n",i,optitrack_data.rigidbody[i].id);
				if(tgtID == i)//optitrack_data.rigidbody[i].id)
				{
					idx = i;
					uav_curr.x = optitrack_data.rigidbody[idx].x;
					uav_curr.y = optitrack_data.rigidbody[idx].y;
					uav_curr.z = optitrack_data.rigidbody[idx].z;
					uav_curr.a = optitrack_data.rigidbody[idx].a;
					uav_curr.b = optitrack_data.rigidbody[idx].b;
					uav_curr.c = optitrack_data.rigidbody[idx].c;
					uav_curr.d = optitrack_data.rigidbody[idx].d;
					optTimeStamp = optitrack_data.time_stamp;
					printf("UAV_curr [   %f,   %f,   %f   ]\n", uav_curr.x, uav_curr.y, uav_curr.z);
					//printf("UAVabc [%f, %f, %f]\n", uav_curr.a, uav_curr.b, uav_curr.c);
					break;
				}
			}
			
			for(i=0;i<MAX_RB_NUMBER;i++)
			{
				if(UAV_FRONT_ID == i)
				{
					idx = i;
					uav_front.x = optitrack_data.rigidbody[idx].x;
					uav_front.y = optitrack_data.rigidbody[idx].y;
					uav_front.z = optitrack_data.rigidbody[idx].z;
					uav_front.a = optitrack_data.rigidbody[idx].a;
					uav_front.b = optitrack_data.rigidbody[idx].b;
					uav_front.c = optitrack_data.rigidbody[idx].c;
					uav_front.d = optitrack_data.rigidbody[idx].d;
				
					//MyOptTimeStamp = TmpOptiData.time_stamp;
					
					break;
				}
			}
			
			
			
/* 		if(Counter<PositionBufNumber)
		{
			uav_front.x = PositionBuf[ Counter].x;
			uav_front.y = PositionBuf[Counter].y;
			uav_front.z = PositionBuf[ Counter].z;
			Counter ++ ;
		}
		else
		{
			Counter = 0;
			uav_front.x = PositionBuf[ Counter].x;
			uav_front.y = PositionBuf[Counter].y;
			uav_front.z = PositionBuf[ Counter].z;
		} */
			
			printf("uav_front: %f   %f  %f\n",uav_front.x,uav_front.y,uav_front.z);
			printf("uav_obj:  %f   %f   %f\n",uav_obj.x,uav_obj.y,uav_obj.z);
			
			
			
			
			
			
			
			
			
			// ���ǰһ���ɻ�Ŀ��Ԫ�ؿ��ã�������������
			if(Available(uav_front,uav_front_last)){
				pthread_mutex_lock(& Quemtx);
				EnQueue(Queue,uav_front);
				pthread_mutex_unlock( &Quemtx );
			}
			
			
			
/*			
			// ��ȡ���ư�����
			for(i=0;i<MAX_RB_NUMBER;i++)
			{
				if(CTRL_STICK_ID == TmpOptiData.rigidbody[i].id)
				{
					idx = i;
					Ctrlstick.x = TmpOptiData.rigidbody[idx].x;
					Ctrlstick.y = TmpOptiData.rigidbody[idx].y;
					Ctrlstick.z = TmpOptiData.rigidbody[idx].z;
					Ctrlstick.a = TmpOptiData.rigidbody[idx].a;
					Ctrlstick.b = TmpOptiData.rigidbody[idx].b;
					Ctrlstick.c = TmpOptiData.rigidbody[idx].c;
					Ctrlstick.d = TmpOptiData.rigidbody[idx].d;
				
					//MyOptTimeStamp = TmpOptiData.time_stamp;
					
					break;
				}
			}
*/		
			////////////////////////////////////////////////////
			// ����1: OptiTrack���������̬
			{
				Quat q;
				EulerAngles ea;
				
				q.w = uav_curr.a;
				q.x = uav_curr.b;
				q.y = uav_curr.c;
				q.z = uav_curr.d;
				//printf("Q[%.2f,%.2f,%.2f,%.2f]\n",uav_curr.a,uav_curr.b,uav_curr.c,uav_curr.d);
				q   = ConvertOptiTrackSystem2ENSSystem(q);
				ea  = Quat2Euler(q);
				//printf("ea[%.2f, %.2f, %.2f]\n",ea.x, ea.y, ea.z);
			}			 
			
			////////////////////////////////////////////////////
			// ����2����÷�����̬����
			// get_fly_status_from_ZeroTech_UAV_SDK(&fly_status_data);
			// CurrYaw   = fly_status_data.yaw_angle;
			// CurrPitch = fly_status_data.pitch_angle;
			// CurrRoll  = fly_status_data.roll_angle;	
					
		}
		//printf("optTimeStamp = %lf  optTimeStampLast = %lf\n",optTimeStamp,optTimeStampLast);
		////////////////////////////////////////////////////
		//����ʱ���� dt
		dt = optTimeStamp - optTimeStampLast;				
		
		//printf("FlyFlag =   %d\n",FlyFlag);	
		if(dt < DBL_MIN)
		{
			//printf("PIDThread Time��%f, dt: %f, too small!\n",optTimeStamp,dt);
			// continue;
		}
		else
		{		
			invdt = 1/dt;
			////////////////////////////////////////////////////
			// ���ư��ٶ�(CSv)����
		//	Xout = (uav_obj.x - uav_curr.x)*100*X_P*invdt;
		//	Yout = (uav_obj.y - uav_curr.y)*100*Y_P*invdt;
		//	Zout = (uav_obj.z - uav_curr.z)*100*Z_P*invdt;
			
			
			
			
			XErr = uav_obj.x-uav_curr.x;
			YErr = uav_obj.y-uav_curr.y;
			ZErr = uav_obj.z-uav_curr.z;
			
			
			//�������
			XErrSum = (XErr + LastXErr) * dt;
			YErrSum = (YErr + LastYErr) * dt;
			ZErrSum = (ZErr + LastZErr) * dt;
			
			//�������΢�� D
			
			XErrDiff = (XErr - LastXErr) * invdt;
			YErrDiff = (YErr - LastYErr) * invdt;
			ZErrDiff = (ZErr - LastZErr) * invdt;
			
			
			//����PID������
			Xout = ( X_P * XErr + X_I * XErrSum + X_D * XErrDiff ) * invdt * 100;  // cm/s
			Yout = ( Y_P * YErr + Y_I * YErrSum + Y_D * YErrDiff ) * invdt * 100;
			Zout = ( Z_P * ZErr + Z_I * ZErrSum + Z_D * ZErrDiff ) * invdt * 100;

			//if(Ctrlstick.x < DBL_MIN && Ctrlstick.y < DBL_MIN && Ctrlstick.z < DBL_MIN )
			//	continue;
	/*		
			CSv_x = (Ctrlstick.x - CtrlstickLast.x)*invdt;
			CSv_y = (Ctrlstick.y - CtrlstickLast.y)*invdt;
			CSv_z = (Ctrlstick.z - CtrlstickLast.z)*invdt;
			
			
			////////////////////////////////////////////////////
			// Ŀ���ٶ�(TGTv)����
			TGTv_x = (uav_curr.x - uav_last.x)*invdt;
			TGTv_y = (uav_curr.y - uav_last.y)*invdt;
			TGTv_z = (uav_curr.z - uav_last.z)*invdt;
			
			////////////////////////////////////////////////////
			//����ƫ�� Err
			XErr = CSv_x - TGTv_x;  //vxƫ��
			YErr = CSv_y - TGTv_y;  //vyƫ��
			ZErr = CSv_z - TGTv_z;  //vzƫ��
			
			YawErr = 0;//YawErr = ErrorAngleNormalize(ObjYaw - CurrYaw); // yaw����ƫ��
			
			////////////////////////////////////////////////////
			//���������� I
			XErrSum = (XErr + LastXErr) * dt;
			YErrSum = (YErr + LastYErr) * dt;
			ZErrSum = (ZErr + LastZErr) * dt;		
			
			YawErrSum = (YawErr + LastYawErr) * dt;
			
			////////////////////////////////////////////////////
			//�������΢�� D
			
			XErrDiff = (XErr - LastXErr) * invdt;
			YErrDiff = (YErr - LastYErr) * invdt;
			ZErrDiff = (ZErr - LastZErr) * invdt;
			
			YawErrDiff = (YawErr - LastYawErr) * invdt;
			
			////////////////////////////////////////////////////
			//����PID������
			Xout += ( X_P * XErr + X_I * XErrSum + X_D * XErrDiff ) * 100;  // cm/s
			Yout += ( Y_P * YErr + Y_I * YErrSum + Y_D * YErrDiff ) * 100;
			Zout += ( Z_P * ZErr + Z_I * ZErrSum + Z_D * ZErrDiff ) * 100;
			YawOut += ( Yaw_P * YawErr + Yaw_I * YawErrSum + Yaw_D * YawErrDiff );  // ��/s
	*/
			if(FlyFlag)
			{
				// ��Ͻ��
				
				if(Xout>100) 
				{target_speed.vx=100;}
				else if(Xout<-100)
				{target_speed.vx=-100;}
				else {target_speed.vx = Xout;}
				if(Yout>100) 
				{target_speed.vy=100;}
			    else if(Yout<-100) 
				{target_speed.vy=-100;}
				else {target_speed.vy = Yout;}
			    if(Zout>50) 
				{target_speed.vz=50;}
			    else if(Zout<-50) 
				{target_speed.vz=-50;}
				else {target_speed.vz = Zout;}
				/* target_speed.vx = 30;
				target_speed.vy = 0;
				target_speed.vz = 0; */
				target_speed.vo = 0;//YawOut;
				target_speed.coordinate = COORDINATE_PROJECT_CONTROL; 		//�ɻ�����ϵ
				
				delta_v = sqrt((uav_curr.x-uav_front.x)*(uav_curr.x-uav_front.x)+(uav_curr.y-uav_front.y)*(uav_curr.y-uav_front.y)+(uav_curr.z-uav_front.z)*(uav_curr.z-uav_front.z));
				if(sqrt((uav_curr.x-uav_front.x)*(uav_curr.x-uav_front.x)+(uav_curr.y-uav_front.y)*(uav_curr.y-uav_front.y))<0.5)
				{
					target_speed.vx = 0;
					target_speed.vy = 0;
					target_speed.vz = 0;
				}
				printf("length between them = %lf\n",delta_v);
				set_target_speed_to_ZeroTech_UAV_SDK( &target_speed );		//ִ���ٶȿ���
				
/* 				printf("FPS:%.2f, Time:%f,%f,%f, Opti[%+5.2f, %+5.2f, %+5.2f; %+06.1f], Tgt[%+5.2f, %+5.2f, %+5.2f; %+06.1f], SpeedSend[%+7.2f,%+7.2f,%+7.2f; %+5.1f]\n", 
											GetFPS(), 
											optTimeStamp,optTimeStampLast,dt,
											uav_curr.x,uav_curr.y,uav_curr.z,CurrYaw,  
											uav_obj.x,uav_obj.y,uav_obj.z,ObjYaw, 
											Xout,Yout,Zout,YawOut);	 */
				printf("target_speed: %d  %d  %d\n",target_speed.vx,target_speed.vy,target_speed.vz);
                if(Info_Check_Number<MAX_DATA)
				{
					info_check[Info_Check_Number].fps = GetFPS();
					info_check[Info_Check_Number].optiT = optTimeStamp;
					info_check[Info_Check_Number].optiTL = optTimeStampLast;
					info_check[Info_Check_Number].dt = dt;
					info_check[Info_Check_Number].uc[0] = uav_curr.x;
					info_check[Info_Check_Number].uc[1] = uav_curr.y;
					info_check[Info_Check_Number].uc[2] = uav_curr.z;
					info_check[Info_Check_Number].uc[3] = CurrYaw;
					info_check[Info_Check_Number].uo[0] = uav_obj.x;
					info_check[Info_Check_Number].uo[1] = uav_obj.y;				
					info_check[Info_Check_Number].uo[2] = uav_obj.z;
					info_check[Info_Check_Number].uo[3] = ObjYaw;
					info_check[Info_Check_Number].v[0] = Xout;
					info_check[Info_Check_Number].v[1] = Yout;
					info_check[Info_Check_Number].v[2] = Zout;
					info_check[Info_Check_Number].v[3] = YawOut;
					Info_Check_Number++;
				}
				else
				{
					printf("����Խ��\n");
				}				
			}

			
			// ��ӡ���
			/* printf("FPS:%.2f, Time:%f,%f,%f, Opti[%+5.2f, %+5.2f, %+5.2f; %+06.1f], Tgt[%+5.2f, %+5.2f, %+5.2f; %+06.1f], SpeedSend[%+7.2f,%+7.2f,%+7.2f; %+5.1f]\n", 
											GetFPS(), 
											optTimeStamp,optTimeStampLast,dt,
											uav_curr.x,uav_curr.y,uav_curr.z,CurrYaw,  
											uav_obj.x,uav_obj.y,uav_obj.z,ObjYaw, 
											Xout,Yout,Zout,YawOut);
								 */		
		
			LastXErr = XErr, LastYErr = YErr, LastZErr = ZErr;			//����ƫ��ֵ���棬�Թ��´μ���΢��ʹ��
			LastYawErr = YawErr;		

		}

		
		optTimeStampLast = optTimeStamp;
		uav_front_last = uav_front;
		CtrlstickLast = Ctrlstick;
		uav_last = uav_curr;
		
		
		usleep( 1000 * PID_CTRL_TIME );//�����ڿ��ƣ��������ڣ�20ms
	}
	return (void *)0;
}

void PrintConf(FFCONFIG* conf)
{
	printf("===========================\n");
	printf("UAV_ID=%d\n",conf->UAV_ID);
	printf("UAV_FRONT_ID=%d\n",conf->UAV_FRONT_ID);
	printf("TRACK_FILENAME=%s\n",conf->TRACK_FILENAME);
    printf("SAVE_FILENAME=%s\n",conf->SAVE_FILENAME);
	printf("READ_MODE=%d\n",conf->READ_MODE);
	//printf("FFDATA_PATH=%s\n",conf->FFDATA_PATH);
	printf("P=%f\n",conf->PID_PARAS.P);
	printf("I=%f\n",conf->PID_PARAS.I);
	printf("D=%f\n",conf->PID_PARAS.D);
	printf("===========================\n");
}

int LoadConfFile(char* argv,FFCONFIG* conf)
{
	FILE *fp;
	if((fp = fopen(argv,"r")) == NULL)
	{
		return -1;
	}
	
	fscanf(fp,"UAV_ID=%d\n",&conf->UAV_ID);
	fscanf(fp,"UAV_FRONT_ID=%d\n",&conf->UAV_FRONT_ID);
	fscanf(fp,"TRACK_FILENAME=%s\n",conf->TRACK_FILENAME);
	fscanf(fp,"SAVE_FILENAME=%s\n",conf->SAVE_FILENAME);
	fscanf(fp,"READ_MODE=%d\n",&conf->READ_MODE);
	//fscanf(fp,"FFDATA_PATH=%s\n",conf->FFDATA_PATH);
	fscanf(fp,"P=%f\n",&conf->PID_PARAS.P);
	fscanf(fp,"I=%f\n",&conf->PID_PARAS.I);
	fscanf(fp,"D=%f\n",&conf->PID_PARAS.D);
	
	fclose(fp);	
	
	PrintConf(conf);
	
	return 0;
}

/* #if 0
void PrintConf(FFCONFIG* conf)
{
	printf("===========================\n");
	printf("UAV_ID=%d\n",conf->ID);
	printf("LOCAL_IP=%s\n",conf->LOCAL_IP);
	printf("MCAST_IP=%s\n",conf->MCAST_IP);
	printf("MCAST_PORT=%s\n",conf->MCAST_PORT);
	printf("FFDATA_PATH=%s\n",conf->FFDATA_PATH);
	printf("P=%f\n",conf->PID_PARAS.P);
	printf("I=%f\n",conf->PID_PARAS.I);
	printf("D=%f\n",conf->PID_PARAS.D);
	printf("===========================\n");
}

int LoadConfFile(char* argv,FFCONFIG* conf)
{
	FILE *fp;
	if((fp = fopen(argv,"r")) == NULL)
	{
		return -1;
	}
	
	fscanf(fp,"ID=%d\n",&conf->ID);
	fscanf(fp,"LOCAL_IP=%s\n",conf->LOCAL_IP);
	fscanf(fp,"MCAST_IP=%s\n",conf->MCAST_IP);
	fscanf(fp,"MCAST_PORT=%s\n",conf->MCAST_PORT);
	fscanf(fp,"FFDATA_PATH=%s\n",conf->FFDATA_PATH);
	fscanf(fp,"P=%f\n",&conf->PID_PARAS.P);
	fscanf(fp,"I=%f\n",&conf->PID_PARAS.I);
	fscanf(fp,"D=%f\n",&conf->PID_PARAS.D);
	
	fclose(fp);	
	
	PrintConf(conf);
	
	return 0;
}
#endif */
#define SUCCESS 0

int unlock(void)
{	
	printf("unlock_cmd_to_ZeroTech_UAV_SDK...\n");
	if(unlock_cmd_to_ZeroTech_UAV_SDK() != SUCCESS)
	{
		printf("....unlock failed!\n");
		return -1;
	}
	printf("....unlock successfully!\n");
	return 0;
}

int lock(void)
{	
	printf("lock_cmd_to_ZeroTech_UAV_SDK...\n");
	if(lock_cmd_to_ZeroTech_UAV_SDK() != SUCCESS)
	{
		printf("....lock failed!\n");
		return -1;
	}
	printf("....lock successfully!\n");
	return 0;
}

int takeoff(void)
{	
	printf("takeoff_cmd_to_ZeroTech_UAV_SDK...\n");
	sdk_takeoff_cmd_t takeoff_cmd;
	memset(&takeoff_cmd, 0, sizeof(sdk_takeoff_cmd_t));
	
	takeoff_cmd.target_height = 100;//1.5m
	
	printf("takeoff_cmd.target_height = %.2fm\n",takeoff_cmd.target_height/100.0);
	if(takeoff_cmd_to_ZeroTech_UAV_SDK(&takeoff_cmd) != SUCCESS)
	{
		printf("....takeoff failed!\n");
		return -1;
	}
	printf("....takeoff successfully!\n");
	return 0;
}
int change_track()
{
	if(ControlMode == 2)
	{
		ControlMode = 1;
		read_track();
		printf("read from file!\n");
	}
	else if(ControlMode == 1)
	{
		ControlMode = 2;
		printf("follow the stick!\n");
	}
	return 1;
}

int autoland(void)
{	
	sdk_autoland_cmd_t autoland_cmd;
	memset(&autoland_cmd, 0, sizeof(sdk_autoland_cmd_t));
	
	printf("autoland_cmd_to_ZeroTech_UAV_SDK...\n");
	if(autoland_cmd_to_ZeroTech_UAV_SDK(&autoland_cmd) != SUCCESS)
	{
		printf("....autoland failed!\n");
		return -1;
	}
	printf("....autoland successfully!\n");
	return 0;
}

int read_track()
{
	FILE *fp;
	float tid,tx,ty,tz,ta,tb,tc,td;
	fp = fopen(trackFilename,"r");
	if(PositionBuf==NULL)
	{
		printf("can not make peace\n");
		return -1;
	}
	if(fp==NULL)
	{
		printf("can not open\n");
		return -1;
	}
	fscanf(fp,"%f%f%f%f%f%f%f%f",&tid,&tx,&ty,&tz,&ta,&tb,&tc,&td);
	while(!feof(fp))
	{
		PositionBuf[PositionBufNumber].id = (int)tid;PositionBuf[PositionBufNumber].x = tx;
		PositionBuf[PositionBufNumber].y = ty;PositionBuf[PositionBufNumber].z = tz;
		PositionBuf[PositionBufNumber].a = ta;PositionBuf[PositionBufNumber].b = tb;
		PositionBuf[PositionBufNumber].c = tc;PositionBuf[PositionBufNumber].d = td;
		//printf("%f\t%f\t%f\n",PositionBuf[PositionBufNumber].x,PositionBuf[PositionBufNumber].y,PositionBuf[PositionBufNumber].z);
		PositionBufNumber++;
	    fscanf(fp,"%f%f%f%f%f%f%f%f",&tid,&tx,&ty,&tz,&ta,&tb,&tc,&td);
	}
	printf("total point number = %d\n",PositionBufNumber);
	fclose(fp);
	return 1;
}

int save_info()
{
	FILE *fp;
	int counter = 0;
	fp = fopen(saveFilename,"w");
	if(info_check==NULL)
	{
		printf("can not make peace\n");
		return -1;
	}
	if(fp==NULL)
	{
		printf("can not open\n");
		return -1;
	}
	for(counter = 0;counter<Info_Check_Number;counter++)
	{
		fprintf(fp," %lf %lf %lf %lf",info_check[counter].fps,info_check[counter].optiT,info_check[counter].optiTL,info_check[counter].dt); 
		fprintf(fp," %lf %lf %lf %lf",info_check[counter].uc[0],info_check[counter].uc[1],info_check[counter].uc[2],info_check[counter].uc[3]); 
		fprintf(fp," %lf %lf %lf %lf",info_check[counter].uo[0],info_check[counter].uo[1] ,info_check[counter].uo[2],info_check[counter].uo[3]); 
		fprintf(fp," %lf %lf %lf %lf\n",info_check[counter].v[0],info_check[counter].v[1],info_check[counter].v[2],info_check[counter].v[3]); 
	}
	
    fclose(fp);
	return 1;
}

int main(int argc, char **argv)
{
	Queue->rear = 0;
	Queue->front = 0;
	int numq = 0;
	//QUEUE Queue[1];
	RigidBodyData_t tempq;
	int err;	
	/*if(read_track()==-1)
	{
		printf("read track failed!");
		return -1;
	}
	 */
	if(argc != 2)
	{
		printf("Usage: ./bridge_uav_P2P_PID ffsetting.conf\n");
		perror("Usage Error"); 
	}	

	char *filename="ffsetting.conf";
	// ���������ļ�
	if(LoadConfFile(filename,&ffconf)!=0)
	{
		perror("ffsetting.conf File Open Error"); 
		exit(-2);
	}	
	
	tgtID = ffconf.UAV_ID;
	UAV_FRONT_ID= ffconf.UAV_FRONT_ID;
	trackFilename = ffconf.TRACK_FILENAME;
	saveFilename = ffconf.SAVE_FILENAME;
	readMode = ffconf.READ_MODE;
	PID_para = ffconf.PID_PARAS;
	pthread_t posCapturerThread;   	 // ��̬�ɼ��߳�
	pthread_t trjGeneratorThread;	 // �켣�����߳�
	pthread_t flightCtrlThread;	 	 // ���п����߳�
	
	sdk_target_speed_t target_speed;
	
	

	
	
	// �źŲ�׽�����趨
	printf("\n- ZeroTech_UAV_SDK Test, press CTRL+C to quit the query loop. \n");
	signal(SIGINT, loop_ctrl);
	signal(SIGRTMIN, loop_ctrl);
    signal(SIGRTMIN+1, loop_ctrl);
	signal(SIGRTMIN+2, loop_ctrl);
  //  signal(SIGRTMIN+2, loop_ctrl);
    //signal(SIGRTMIN+3, loop_ctrl);

	if(init_ZeroTech_UAV_SDK() != 0)
	{
		printf("init_ZeroTech_UAV_SDK() failed!\n");
		return -1;
	}


#ifdef TEST_PID
     //������
	while(unlock() != SUCCESS)
	{
		unlock();
	}
	sleep(5);
		
	 //һ����ɣ�
	while(takeoff() != SUCCESS)
	{
		takeoff();
	}
	sleep(7); 
#endif
	
	
	// ���SDK����Ȩ
	/* if(enable_action_control_cmd_to_ZeroTech_UAV_SDK()!= 0)
	{
		printf("enable_action_control_cmd_to_ZeroTech_UAV_SDK failed!\n");
		return -1;
	} 
     */

	
	 
	 // ��ʼ���켣�����߳�
	 //printf("create the multicast pthread\n");
	 //pthread_create( &posCapturerThread, NULL, OptiTrackPthread, NULL);
	 printf("create the position control pthread!\n");
	 pthread_create( &trjGeneratorThread, NULL, TrajGeneratorPthread, NULL);
	 printf("create the PID control pthread!\n");
	 pthread_create( &flightCtrlThread, NULL, PIDCtrlPthread, NULL);
	 printf("P2P control start!\n");
	 	// ��ʼ��PID�����߳�
 	pthread_mutex_lock( & Objmtx );//����
	ObjRigidBody.x =1;
	ObjRigidBody.y = 0;
	ObjRigidBody.z = 1;
	printf("initialize the position data!\n");
	pthread_mutex_unlock( & Objmtx );//���� 
	
	 
#ifdef TEST_PID

			
	// ��ʼ��Ŀ���ٶ�
	 target_speed.vx = 0;
	target_speed.vy = 0;
	target_speed.vz = 0;
	target_speed.coordinate = COORDINATE_PROJECT_CONTROL; //��ʼִ������ʱ������ϵ 
	//printf("\ninitialize the speed!\n");
		
	//flight_control_sdk_takeoff_cmd();
	set_target_speed_to_ZeroTech_UAV_SDK( &target_speed );//�ٶȳ�ʼ��
		
	// �ȴ�PID�߳̽���30s���п���
	
	 while(LoopFlag == 1)
	{
		sleep(60);
		LoopFlag = 0;
	} 
 
#endif
	/* 
	// �ȴ��߳��˳�
	err = pthread_join(posCapturerThread, NULL);
    if (err != 0)
    {
      printf("can not join with thread %s:%s\n", "posCapturerThread",strerror(err));
      exit(-1);
    } */
	
 /*	while(!IsEmpty(Queue)){
		tempq = DeQueue(Queue);
		numq++;
	}
	printf("numq = %d\n",numq);*/
	
	
	
	
	
	 err = pthread_join(trjGeneratorThread, NULL);
    if (err != 0)
    {
      printf("can not join with thread %s:%s\n", "trjGeneratorThread",strerror(err));
      exit(-1);
    }

	err = pthread_join(flightCtrlThread, NULL);
    if (err != 0)
    {
      printf("can not join with thread %s:%s\n", "flightCtrlThread",strerror(err));
      exit(-1);
    }    
		
	// �ͷ�SDK����Ȩ
	/* if(disable_action_control_cmd_to_ZeroTech_UAV_SDK()!= 0)
	{
		printf("enable_action_control_cmd_to_ZeroTech_UAV_SDK failed!\n");
		return -1;
	}	 */
	
#ifdef TEST_PID
 	//һ�����䣺
	while(autoland() != SUCCESS)
	{
		autoland();
	}
	sleep(1);
		
	//������
	while(lock() != SUCCESS)
	{
		lock();
	}
	sleep(2);
	
	if(save_info()==-1)
	{
		printf("save failed!");
		return -1;
	}  
#endif
	
// ������Դ
	exit_ZeroTech_UAV_SDK(); 
	return 0;
}


//=====================================================================================================
#if 0

//para: 1������صĵ㣻2��Բ�ε�������������ԭ�㣻3��x=yΪԲ�İ뾶,zΪ���ġ���߶ȡ�
void  CircleElectronicCage( RigidBodyData_t * CurrRigidBodyData_t, RigidBodyData_t * p_ZeroRigidBodyData_t, RigidBodyData_t * p_DertaRigidBodyData_t )
{
	float Derta_R = sqrt( ( ( * CurrRigidBodyData_t ).x - ( * p_ZeroRigidBodyData_t ).x ) * ( ( * CurrRigidBodyData_t ).x - ( * p_ZeroRigidBodyData_t ).x )  + 
				          ( ( * CurrRigidBodyData_t ).y - ( * p_ZeroRigidBodyData_t ).y ) * ( ( * CurrRigidBodyData_t ).y - ( * p_ZeroRigidBodyData_t ).y )  );
	float Derta_r = ( * p_DertaRigidBodyData_t ).x;//���������뾶
	if( Derta_R > Derta_r )
	{
		( * CurrRigidBodyData_t ).x = ( * CurrRigidBodyData_t ).x * Derta_r / ( Derta_R - Derta_r );//�޶�Ŀ��x
		( * CurrRigidBodyData_t ).y = ( * CurrRigidBodyData_t ).y * Derta_r / ( Derta_R - Derta_r );//�޶�Ŀ��y
	}
	if(  ( * CurrRigidBodyData_t ).z > ( * p_ZeroRigidBodyData_t ).z + ( * p_DertaRigidBodyData_t ).z  )
	{
		( * CurrRigidBodyData_t ).z = ( * p_ZeroRigidBodyData_t ).z + ( * p_DertaRigidBodyData_t ).z ;//�޶�Ŀ��z�Ϸ�
	}
	if(  ( * CurrRigidBodyData_t ).z < ( * p_ZeroRigidBodyData_t ).z - ( * p_DertaRigidBodyData_t ).z  )
	{
		( * CurrRigidBodyData_t ).z = ( * p_ZeroRigidBodyData_t ).z - ( * p_DertaRigidBodyData_t ).z ;//�޶�Ŀ��z�·�
	}
}

//para: 1������صĵ㣻2�����ε�������������ԭ�㣻3��x,y,zΪ��Ӧ�ߵİ�߳�
void  RectangleElectronicCage( RigidBodyData_t * CurrRigidBodyData_t, RigidBodyData_t * p_ZeroRigidBodyData_t, RigidBodyData_t * p_DertaRigidBodyData_t )
{
	if(  ( * CurrRigidBodyData_t ).x > ( * p_ZeroRigidBodyData_t ).x + ( * p_DertaRigidBodyData_t ).x  )
	{
		( * CurrRigidBodyData_t ).x = ( * p_ZeroRigidBodyData_t ).x + ( * p_DertaRigidBodyData_t ).x ;//�޶�Ŀ��x�Ϸ�
	}else if(  ( * CurrRigidBodyData_t ).x < ( * p_ZeroRigidBodyData_t ).x - ( * p_DertaRigidBodyData_t ).x  )
	{
		( * CurrRigidBodyData_t ).x = ( * p_ZeroRigidBodyData_t ).x - ( * p_DertaRigidBodyData_t ).x ;//�޶�Ŀ��x�·�
	}else{}
	
	if(  ( * CurrRigidBodyData_t ).y > ( * p_ZeroRigidBodyData_t ).y + ( * p_DertaRigidBodyData_t ).y  )
	{
		( * CurrRigidBodyData_t ).y = ( * p_ZeroRigidBodyData_t ).y + ( * p_DertaRigidBodyData_t ).y ;//�޶�Ŀ��y�Ϸ�
	}else if(  ( * CurrRigidBodyData_t ).y < ( * p_ZeroRigidBodyData_t ).y - ( * p_DertaRigidBodyData_t ).y  )
	{
		( * CurrRigidBodyData_t ).y = ( * p_ZeroRigidBodyData_t ).y - ( * p_DertaRigidBodyData_t ).y ;//�޶�Ŀ��y�·�
	}else{}
	
	if(  ( * CurrRigidBodyData_t ).z > ( * p_ZeroRigidBodyData_t ).z + ( * p_DertaRigidBodyData_t ).z  )
	{
		( * CurrRigidBodyData_t ).z = ( * p_ZeroRigidBodyData_t ).z + ( * p_DertaRigidBodyData_t ).z ;//�޶�Ŀ��z�Ϸ�
	}else if(  ( * CurrRigidBodyData_t ).z < ( * p_ZeroRigidBodyData_t ).z - ( * p_DertaRigidBodyData_t ).z  )
	{
		( * CurrRigidBodyData_t ).z = ( * p_ZeroRigidBodyData_t ).z - ( * p_DertaRigidBodyData_t ).z ;//�޶�Ŀ��z�·�
	}else{}
}
//
//1Ϊ˳ʱ�룬-1Ϊ��ʱ��
RigidBodyData_t CircleCalc( RigidBodyData_t * p_Zero, float R_data, int flag, RigidBodyData_t CurrRigidBodyData_t, float time ,float DertaDistance )
{
	static float pi = 3.141592653;
	static float DertaTime = 0;
	static long N_data = 0;
	static float DertaAngle = 0;
	N_data = pi * 2 * R_data / DertaDistance;
	//DertaTime = time * / N_data;
	DertaAngle = pi * 2 / N_data;
	if( 1 == flag )
	{
		
	}
}
#endif















