#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <mqueue.h>
#include <stdbool.h>
#include <sys/time.h>
#include <sys/stat.h>        /* For mode constants */
#include <signal.h>
#include <errno.h>

#include "Pilot.h"
#include "Robot.h"
#include "prose.h"

/*
 * paramètres de configuration du pilote
 */
#define NAME_MQ_BOX  "/bal_Pilot" //ne pas oublier le / devant le nom !
#define MQ_MSG_COUNT 10 // min et max variable pour Linux (suivant version noyau, cf doc)
#define MQ_MSG_SIZE 256 // 128 est le minimum absolu pour Linux, 1024 une valeur souvent utilisée !

//STRUCTURES
#define STATE_GENERATION S(S_FORGET) S(S_IDLE) S(S_RUNNING) S(S_IF_IDLE) S(S_IF_RUN_VEL) S(S_IF_RUN_BUMP) S(S_DEATH)
#define S(x) x,
typedef enum {STATE_GENERATION STATE_NB} State;
#undef S
#define S(x) #x,
const char * const State_Name[] = { STATE_GENERATION };
#undef STATE_GENERATION
#undef S

#define ACTION_GENERATION S(A_NOP) S(A_SEND_MVT) S(A_TEST_BUMPED) S(A_STOP)
#define S(x) x,
typedef enum {ACTION_GENERATION ACTION_NB}  Action;
#undef S
#define S(x) #x,
const char * const Action_Name[] = { ACTION_GENERATION };
#undef ACTION_GENERATION
#undef S

#define EVENT_GENERATION S(E_VELOCITY) S(E_CHECK) S(E_VEL_NULL) S(E_VEL_N_NULL) S(E_BUMPED) S(E_N_BUMPED) S(E_STOP)
#define S(x) x,
typedef enum {EVENT_GENERATION EVENT_NB}  Event;
#undef S
#define S(x) #x,
const char * const EventName[] = { EVENT_GENERATION };
#undef EVENT_GENERATION
#undef S

typedef struct
{
	State destinationState;
	Action action;
} Transition;

struct Pilot_t
{
    Robot robot;
    VelocityVector currentVel;
    PilotState myPilotState;
	State state;
};

typedef struct
{
	Event event;
} MqMsg;

typedef union
{
	char buffer[MQ_MSG_SIZE];
	MqMsg data;
} MqMsgAdapter;

typedef void (*ActionPtr)();

// PROTOTYPES
static void Pilot_mqSend (MqMsg aMsg);
static void* Pilot_run();
static void Pilot_sendMvt(VelocityVector vel);

// VARIABLES
static Pilot myPilot;
static Transition mySm[STATE_NB][EVENT_NB] =
{
	[S_IDLE][E_VELOCITY]={S_IF_IDLE,A_NOP},
	[S_IDLE][E_STOP]={S_DEATH,A_STOP},
	[S_RUNNING][E_VELOCITY]={S_IF_RUN_VEL,A_NOP},
	[S_RUNNING][E_CHECK]={S_IF_RUN_BUMP,A_TEST_BUMPED},
	[S_RUNNING][E_STOP]={S_DEATH,A_STOP},
	[S_IF_IDLE][E_VEL_NULL]={S_IDLE,A_SEND_MVT},
	[S_IF_IDLE][E_VEL_N_NULL]={S_RUNNING,A_SEND_MVT},
	[S_IF_RUN_VEL][E_VEL_NULL]={S_IDLE,A_SEND_MVT},
	[S_IF_RUN_VEL][E_VEL_N_NULL]={S_RUNNING,A_SEND_MVT},
	[S_IF_RUN_BUMP][E_BUMPED]={S_IDLE,A_SEND_MVT},
	[S_IF_RUN_BUMP][E_N_BUMPED]={S_RUNNING,A_SEND_MVT}
};
static pthread_t MyThread;
static const ActionPtr ActionsTab[3] = {&Pilot_stop, &Pilot_setRobotVelocity, &Pilot_toggleEmergencyStop, &Pilot_check};

// FONCTIONS
extern void Pilot_start()
{
	myPilot.currentVel.dir = FORWARD;
	myPilot.currentVel.power = 0;
	myPilot.state = S_IDLE;
}

extern void Pilot_stop()
{
	MqMsg msg = {.event = E_STOP};
	myPilot.currentVel.dir = FORWARD;
	myPilot.currentVel.power = 0;

	Pilot_mqSend (msg);
}

extern MqMsg Pilot_mqReceive ()
{
	int check;
	mqd_t mq;
	MqMsgAdapter msg;
	mq = mq_open (NAME_MQ_BOX, O_RDONLY);

	check = mq_receive (mq, msg.buffer, MQ_MSG_SIZE, 0);

	check = mq_close (mq);

	return msg.data;
}

static void Pilot_mqSend (MqMsg aMsg)
{
	int check;
	MqMsgAdapter msg;
	mqd_t mq;
	msg.data = aMsg;
	/* envoi de l'ordre à travers la BAL */
	mq = mq_open (NAME_MQ_BOX, O_WRONLY);

	check = mq_send (mq, msg.buffer, sizeof (msg.buffer), 0);

	check = mq_close (mq);

}

extern Pilot_setRobotVelocity(VelocityVector vel)
{
	MqMsg msg = {.event = E_VELOCITY};
	myPilot.currentVel.dir = vel.dir;
	myPilot.currentVel.power = vel.power;
	Pilot_sendMvt(myPilot.currentVel);
	Pilot_mqSend (msg);
}

extern void Pilot_toggleEmergencyStop()
{
	myPilot.currentVel.dir = FORWARD;
	myPilot.currentVel.power = 0;
	MqMsg msg = {.event = E_STOP};
	Pilot_mqSend (msg);
}

extern void Pilot_check()
{
	if(hasBumped()){
		MqMsg msg = {.event = E_BUMPED};
		Pilot_mqSend (msg);
	}else{
		MqMsg msg = {.event = E_N_BUMPED};
		Pilot_mqSend (msg);
	}
}

static void* Pilot_run()
{
	MqMsg msg;
	Action act;

	while (myPilot.state != S_DEATH)
	{
		msg = Lampe_mqReceive ();
		if (mySm[myPilot.state][msg.event].destinationState != S_FORGET)
		{
			act = mySm[myPilot.state][msg.event].action;
			ActionsTab[act]();
			myPilot.state = mySm[myPilot.state][msg.event].destinationState;
		}
	}
	Robot_free();
	Pilot_free ();
	return (0);
}

static void Pilot_sendMvt(VelocityVector vel)
{
	switch (vel.dir){
			case LEFT:
				Robot_setWheelsVelocity(myPilot.robot,-vel.power,vel.power);
				break;

			case RIGHT:
				Robot_setWheelsVelocity(myPilot.robot,vel.power,-vel.power);
				break;

			case FORWARD:
				Robot_setWheelsVelocity(myPilot.robot,vel.power,vel.power);
				break;

			case BACKWARD:
                if(Robot_getRobotSpeed(myPilot.robot) !=0){
                    Robot_setWheelsVelocity(myPilot.robot,0,0);
                }else{
                    Robot_setWheelsVelocity(myPilot.robot,-vel.power,-vel.power);
                }
				break;

			default:
				Robot_setWheelsVelocity(myPilot.robot,0,0);
		}
}

extern void Pilot_new()
{
    struct mq_attr mqa;
	int check;
	mqd_t mq;

	/* creation de la BAL */
	mqa.mq_maxmsg = MQ_MSG_COUNT;
	mqa.mq_msgsize = MQ_MSG_SIZE;
	mqa.mq_flags = 0;
	// Destruction de la BAL au cas ou (si BAL mal détruite lors d'une précédente utilisation)
	check = mq_unlink (NAME_MQ_BOX);
	mq = mq_open (NAME_MQ_BOX, O_CREAT, 0777, &mqa);

	check = mq_close (mq);
	myPilot.state = S_IDLE;
	Robot_new();
	check = pthread_create (&MyThread, NULL, Pilot_run, NULL);

}

extern void Pilot_free()
{
	int check;
	/* destruction de la BAL */
	check = mq_unlink (NAME_MQ_BOX);
}
