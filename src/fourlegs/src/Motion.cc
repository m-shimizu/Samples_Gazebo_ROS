/*---------------------------------------------------------------*
 | Motion Control Program                     2015-12-18         |
 |                                             By M.SHIMIZU      |
 |                                  (shimizu@sist.chukyo-u.ac.jp)|
 |  COMPILER: GCC                                                |
 |  TARGET  : Gazebo Plugin                                      |
 *---------------------------------------------------------------*/

#include <stdio.h>
#include <malloc.h>
#include "Motion.hh"
#include "my_defs.h"

extern void	set_angle(int _motor, float _angle);
// servo number(NOT PWM CH), angle(degree)
					// ser_angle must be prepared in the another source file.

/************************************/
/*	Function prototype declaration  */
/************************************/
void	Set_Start_Parameters(ATI_PACK* mdblp, int l, int m);
int		MakeFitMotionData(ATI_PACK* fit, ATI_PACK* _new);

void	Init_ATI(ATI* mdbp, int motors, int motions, int* motor_number_array
								, ATI_CELL* mdb_cell_array)
{
	mdbp->Motors = motors;
	mdbp->Motions = motions;
	mdbp->Motor_Number = motor_number_array;
	mdbp->Motion = mdb_cell_array;
	mdbp->Index = NULL; // There is a possibility that Index is NULL.
}

void	Set_Index_to_ATI(ATI* mdbp, int* index_array)
{
	mdbp->Index = index_array; // There is a possibility that Index is NULL.
}

#define	MTN_MM(R,N)	(mdbp->Motions*(R)+(N))

void	Disp_ATI(ATI* mdbp)
{
	int	mtr, mtn;
//	printf("Motors = %d  ,  Motions = %d\n", mdbp->Motors, mdbp->Motions);
//	printf("Motor Number = { ");
	for(mtr = 0; mtr < mdbp->Motors; mtr++)
		printf( "%d ", mdbp->Motor_Number[mtr]);
	printf("}\n");
	for(mtr = 0; mtr < mdbp->Motors; mtr++)
	{
		printf("[ %2d ] A,T = {", mtr);
		for(mtn = 0; mtn < mdbp->Motions; mtn++)
		{
			printf(" { %d , %d } "
				, (int)(mdbp->Motion[MTN_MM(mtr,mtn)].Angle)
				, (int)(mdbp->Motion[MTN_MM(mtr,mtn)].Time));
		}
		printf("}\n");
	}
	printf("\n");
}

#define ATI_MTRS	(mdblp->Mdb[l]->Motors)
#define SIZEOF_ATIW	(sizeof(ATI_WORK)*(mdblp->ATIS))
#define SIZEOF_DA	(sizeof(mdblp->Mdbw[0].Delta_Angle[0])*ATI_MTRS)
#define SIZEOF_C	(sizeof(mdblp->Mdbw[0].Current[0])*ATI_MTRS)
#define SIZEOF_CS	(sizeof(mdblp->Mdbw[0].Current_Step[0])*ATI_MTRS)
#define	GET_MEM_E(D,S,M)	if(NULL == ( D = (typeof(D))malloc( S )))\
			{ printf("Insufficient Memory!! : %s\n", M ); for(;;); }

void	free_ATIW(ATI_PACK* mdblp)
{
	int	l;
	if(mdblp->Mdbw != NULL && mdblp->ATIS > 0)
	{
		for(l = 0; l < mdblp->ATIS; l++)
		{
			free(mdblp->Mdbw[l].Delta_Angle);
			free(mdblp->Mdbw[l].Current);
			free(mdblp->Mdbw[l].Current_Step);
		}
		free(mdblp->Mdbw);
		mdblp->Mdbw = NULL;
	}
}

void	malloc_ATIW(ATI_PACK* mdblp)
{
	int	l;
	if(mdblp->Mdbw == NULL && mdblp->ATIS > 0)
	{
		GET_MEM_E( mdblp->Mdbw, SIZEOF_ATIW, "ATIW x ATIS")
		for(l = 0; l < mdblp->ATIS; l++)
		{
			GET_MEM_E(mdblp->Mdbw[l].Delta_Angle, SIZEOF_DA, "Delta_Angle")
			GET_MEM_E(mdblp->Mdbw[l].Current, SIZEOF_C, "Current")
			GET_MEM_E(mdblp->Mdbw[l].Current_Step, SIZEOF_CS, "Current_step")
		}
	}
}

void	Clear_ATI_PACK(ATI_PACK* mdblp)
{
	mdblp->ATIS = 0;
	mdblp->Mdbw = NULL;
}

void	Init_ATI_PACK(ATI_PACK* mdblp, int ATIS, ATI** Mdb, float* MS
													, int* SS, START_ID* SI)
{
	int	l, m;
	free_ATIW(mdblp);
	mdblp->ATIS = ATIS;
	mdblp->Mdb  = Mdb;
	mdblp->MS   = MS;
	mdblp->SS   = SS;
	mdblp->SI   = SI;
	mdblp->SF   = 1.0;
	mdblp->FDBS = 0;
	mdblp->Fbf  = NULL;
	mdblp->Change_flag = 0;
	mdblp->Reflex_flag = 0;
	malloc_ATIW(mdblp);
	for(l = 0; l < mdblp->ATIS; l++)
	{	// <<<< Rewriting follows, CHECK ResetMotion(),too. >>>>
		for(m = 0; m < mdblp->Mdb[l]->Motors; m++)
		{
			mdblp->Mdbw[l].Current_Step[m] = mdblp->SS[l];
			mdblp->Mdbw[l].Current[m].Time = 0;
			Set_Start_Parameters(mdblp, l, m); // This must be called after setting of Current_Step
		}
		mdblp->Mdbw[l].Waiting_flag = (mdblp->SI[l].Index_num != 0)?1:0;
		mdblp->Mdbw[l].Active_flag = 0;
	}
}

void	Set_Speed_of_ATI_PACK(ATI_PACK* mdblp, float SF)
{
	mdblp->SF   = SF;
}

void	Set_Feedback_to_ATI_PACK(ATI_PACK* mdblp
								, int FDBS, void (**Fbf)(struct ATI_PACK *))
{
	mdblp->FDBS = FDBS;
	mdblp->Fbf  = Fbf;
}

/************************************/
/*     Motion Data Base Engine      */
/************************************/
int*	TmpAngles;
float*	TmpAngle;
float*	TmpAngle_NA;
float*	Current_Motor_Angle;

static int	MAX_MOTOR_CHANNELS;

void	Clear_TmpAngle(void)
{
	int	m;
	for(m = 0; m < MAX_MOTOR_CHANNELS; m++)
	{
		TmpAngles[m]   = 0;
		TmpAngle[m]    = 0.0;
		TmpAngle_NA[m] = 0.0;
	}
}

void	Add_Angle(int rmn, float angle)
{
	TmpAngle[rmn] += angle;
	TmpAngles[rmn]++;
}

void	Add_Angle_NA(int rmn, float angle)
{
	TmpAngle_NA[rmn] += angle;
}

inline void	Set_Angle(int m, float _angle)
{
	Current_Motor_Angle[m] = _angle;
	set_angle(m, _angle);
}

void	Set_Angles(void)
{
	int	m;
	for(m = 0; m < MAX_MOTOR_CHANNELS; m++)
		if(TmpAngles[m] != 0)
			Set_Angle(m, TmpAngle[m]/TmpAngles[m] + TmpAngle_NA[m]);
}

void	Calc_Angles(float Angle[], char Active_Angle[])
{
	int	m;
	for(m = 0; m < MAX_MOTOR_CHANNELS; m++)
	{
		if(TmpAngles[m] != 0)
		{
			Active_Angle[m] = 1;
			Angle[m] = TmpAngle[m]/TmpAngles[m];
//printf("calc_angles : [%d] %d  \n", m, (int)(Angle[m]*10));
		}
		else
			Active_Angle[m] = 0;
	}
}

int	Get_Angles(int rmn)
{
	if(rmn < 0 || rmn >= MAX_MOTOR_CHANNELS)
		return -1;
	return TmpAngles[rmn];
}

float	Get_Angle(int rmn)
{
	if(rmn < 0 || rmn >= MAX_MOTOR_CHANNELS)
		return 0.0;
	return TmpAngle[rmn];
}

float	Get_Angle_NA(int rmn)
{
	if(rmn < 0 || rmn >= MAX_MOTOR_CHANNELS)
		return 0.0;
	return TmpAngle_NA[rmn];
}

#define	MTN_MM2(R,N)	(mdblp->Mdb[l]->Motions*(R)+(N))

#define CSI	mdblp->Mdbw[mdblp->SI[l].Motion_num].Current_Step[0] // Index is on the Motor 0's timing.
#define	CS	mdblp->Mdbw[l].Current_Step[m]
#define	NS	((mdblp->Mdbw[l].Current_Step[m] + 1) % mdblp->Mdb[l]->Motions)
#define	SUB_ANGLE	(mdblp->Mdb[l]->Motion[MTN_MM2(m,NS)].Angle - mdblp->Mdb[l]->Motion[MTN_MM2(m,CS)].Angle)
#define	lSTEP_LOOPS	(mdblp->Mdb[l]->Motion[MTN_MM2(m,CS)].Time)
#define	SPEED_FACTOR	(mdblp->MS[l] * mdblp->SF) // Speed Factor:Large value means Faster.

void	Set_Start_Parameters(ATI_PACK* mdblp, int l, int m)
{
	mdblp->Mdbw[l].Delta_Angle[m] = (lSTEP_LOOPS != 0)?
									(SUB_ANGLE * SPEED_FACTOR / lSTEP_LOOPS):0;
	mdblp->Mdbw[l].Current[m].Angle = mdblp->Mdb[l]->Motion[MTN_MM2(m,CS)].Angle;
}

void	MotionEngine(ATI_PACK* mdblp, int l)
{
	int	m;
	if(mdblp->Mdbw[l].Active_flag == 0)
		return;
	if(mdblp->Mdbw[l].Waiting_flag == 1)
	{
		if(mdblp->Mdb[mdblp->SI[l].Motion_num]->Index != NULL 
		  && mdblp->SI[l].Index_num == mdblp->Mdb[mdblp->SI[l].Motion_num]->Index[CSI])
			mdblp->Mdbw[l].Waiting_flag = 0;
		else
			return;
	}
	for(m = 0; m < mdblp->Mdb[l]->Motors; m++)
	{
		if(mdblp->Mdb[l]->Motion[MTN_MM2(m,CS)].Time != 0)
		{		// Time==0 -> Writing to the wrong memory....WHY?
			if(mdblp->Mdbw[l].Current[m].Time == 0) // Initialize ?
			{
				Set_Start_Parameters(mdblp, l, m);
			}
			if(mdblp->Mdbw[l].Current[m].Time >= (int)(mdblp->Mdb[l]->Motion[MTN_MM2(m,CS)].Time / SPEED_FACTOR))
			{
				CS = NS;
				Set_Start_Parameters(mdblp, l, m);
				mdblp->Mdbw[l].Current[m].Time = 0;
			}
			else
			{
				mdblp->Mdbw[l].Current[m].Angle += mdblp->Mdbw[l].Delta_Angle[m];
				mdblp->Mdbw[l].Current[m].Time  += 1;
			}
			if(mdblp->Mdb[l]->Motor_Number[m] != -1) // -1 means No Operation
				Add_Angle(mdblp->Mdb[l]->Motor_Number[m], mdblp->Mdbw[l].Current[m].Angle);
		}
		else
		{
			if(mdblp->Mdb[l]->Motor_Number[m] != -1) // -1 means No Operation
				Add_Angle(mdblp->Mdb[l]->Motor_Number[m], mdblp->Mdbw[l].Current[m].Angle);
		}
	}
}

void	FeedBackAction(ATI_PACK* mdblp)
{
	int	fb;
	for(fb = 0; fb < mdblp->FDBS; fb++)
	{
		(* mdblp->Fbf[fb])(mdblp);
	}
}

extern ATI_PACK  FML;

/************************************/
/*        Motion List Player        */
/************************************/
void	MotionPlayer(ATI_PACK* mdblp)
{
	int	l;
	static ATI_PACK*	last_mdblp = (ATI_PACK*)NULL;
	static char			FM_flag = 0;
	if(mdblp != last_mdblp || mdblp->Change_flag == 1)
	{
		mdblp->Change_flag = 0;
		FM_flag = MakeFitMotionData(&FML, mdblp);
		last_mdblp = mdblp;
	}
	if(FM_flag == 1)
		mdblp = &FML;
	/*vvvvvvvvvvvvvvvvvv The Motion Player vvvvvvvvvvvvvvvvv*/
	Clear_TmpAngle();
	for(l = 0; l < mdblp->ATIS; l++)
		MotionEngine(mdblp, l);
	FeedBackAction(mdblp);
	Set_Angles();
	/*^^^^^^^^^^^^^^^^^^ The Motion Player ^^^^^^^^^^^^^^^^^*/
	if(FML.Mdbw[0].Current_Step[0] == 1)
		FM_flag = 0;
}

/************************************/
/*       Fit Motion Data Base       */
/************************************/
#define	FITSTEPS 200

ATI	FITMOTION;
int*	FITMTN;
ATI_CELL*	FITATI; // Motion Data by Each Motor
ATI*		FML_ATI[] = {&FITMOTION};
float		FML_MS[] = {1};
int			FML_SS[] = {0};
START_ID	FML_SI[] = {{0,0}};
ATI_PACK	FML;

void	Set_FitSteps(int _fs)
{
	int	i;
	for(i = 0; i < MAX_MOTOR_CHANNELS; i++)
		FITATI[i * 2].Time = _fs;
}

void	Calc_Motor_Angles(float Angle[], char Active_Angle[], ATI_PACK* mdblp)
{
	int	l, m;
	Clear_TmpAngle();
	for(l = 0; l < mdblp->ATIS; l++)
	{
		if(mdblp->Mdbw[l].Active_flag == 0)
			continue;
		for(m = 0; m < mdblp->Mdb[l]->Motors; m++)
			Add_Angle(mdblp->Mdb[l]->Motor_Number[m], mdblp->Mdbw[l].Current[m].Angle);
	}
	Calc_Angles(Angle, Active_Angle);
}

#define	NEEDLESS_FIT_ANGLE	1
#define	_abs(X)	(((X)<0)?-(X):(X))

int	Need_Fit(float New_Angle[], float Current_Angle[], char Active_Angle[])
{
	int	m;
	for(m = 0; m < MAX_MOTOR_CHANNELS; m++)
	{
		if(Active_Angle[m] == 0)
			continue;
		if(NEEDLESS_FIT_ANGLE < _abs(New_Angle[m] - Current_Angle[m]))
			return 1;
	}
	return 0;
}

#define	MTN_MM3(R,N)	(fit->Mdb[0]->Motions*(R)+(N))

void	Set_FitMotionData(ATI_PACK* fit, float New_Angle[], float Current_Angle[], char Active_Angle[])
{
	int	m;
	for(m = 0; m < MAX_MOTOR_CHANNELS; m++)
	{
		fit->Mdb[0]->Motion[MTN_MM3(m,0)].Angle = Current_Angle[m];
		if(Active_Angle[m] == 0)
			fit->Mdb[0]->Motion[MTN_MM3(m,1)].Angle = Current_Angle[m];
		else
			fit->Mdb[0]->Motion[MTN_MM3(m,1)].Angle = New_Angle[m];
	}
	for(m = 0; m < fit->Mdb[0]->Motors; m++)
	{
		fit->Mdbw[0].Current_Step[m] = 0;
		fit->Mdbw[0].Current[m].Time = 0;
		Set_Start_Parameters(fit, 0, m); // This must be called after setting of Current_Step
	}
	fit->Mdbw[0].Active_flag = 1;
	fit->Mdbw[0].Waiting_flag = 0;
}

int	MakeFitMotionData(ATI_PACK* fit, ATI_PACK* n_mdblp)
{
	char	Active_Angle[MAX_MOTOR_CHANNELS];
	float	New_Angle[MAX_MOTOR_CHANNELS];
	Calc_Motor_Angles(New_Angle, Active_Angle, n_mdblp);
	if(Need_Fit(New_Angle, Current_Motor_Angle, Active_Angle))
	{
		Set_FitMotionData(fit, New_Angle, Current_Motor_Angle, Active_Angle);
		if(n_mdblp->Reflex_flag == 1)
			return 0;
		return 1;
	}
	return 0;
}

#define	SIZEOFDxMMC(D)	(sizeof(* D)*MAX_MOTOR_CHANNELS)
#define	GET_MEM_E_MMC(D,M)	GET_MEM_E(D, SIZEOFDxMMC(D), M)

void	Get_Motion_Arrays(void)
{
	int	m;
	GET_MEM_E_MMC(TmpAngles, "TmpAngles")
	GET_MEM_E_MMC(TmpAngle, "TmpAngle")
	GET_MEM_E_MMC(TmpAngle_NA, "TmpAngle_NA")
	GET_MEM_E_MMC(Current_Motor_Angle, "Current_Motor_Angle")
	GET_MEM_E_MMC(FITMTN, "FITMTN")
	GET_MEM_E(FITATI, SIZEOFDxMMC(FITATI)*2, "FITATI")
	for(m = 0; m < MAX_MOTOR_CHANNELS; m++)
	{
		Current_Motor_Angle[m] = 0;
	}
}

void	Init_FITMTN_FITATI(void)
{
	int	m;
	for(m = 0; m < MAX_MOTOR_CHANNELS; m++)
	{
		FITMTN[m    ]       = m;
		FITATI[m*2  ].Angle = 0;
		FITATI[m*2  ].Time  = FITSTEPS;
		FITATI[m*2+1].Angle = 0;
		FITATI[m*2+1].Time  = 100;
	}
/*
int	FITMTN[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
ATI_CELL	FITATI[]={{0, FITSTEPS},{  5, 100} // Motion Data by Each Motor
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}
					,{0, FITSTEPS},{  5, 100}};
*/
}

void	Init_Motion(int max_motor_channels)
{
	MAX_MOTOR_CHANNELS = max_motor_channels;
	Get_Motion_Arrays();
	Init_FITMTN_FITATI();
	Init_ATI(&FITMOTION, MAX_MOTOR_CHANNELS, 2, FITMTN, FITATI);
	Init_ATI_PACK(&FML, 1, FML_ATI, FML_MS, FML_SS, FML_SI);
}

/************************************/
/*      Other Good Functions        */
/************************************/
void	StartMotion(ATI_PACK* mdblp, int l)
{
	mdblp->Mdbw[l].Active_flag = 1;
	mdblp->Change_flag = 1;
}

void	StopMotion(ATI_PACK* mdblp, int l)
{
	mdblp->Mdbw[l].Active_flag = 0;
}

void	ResetMotion(ATI_PACK* mdblp, int l)
{
	int	m;
	for(m = 0; m < mdblp->Mdb[l]->Motors; m++)
	{
		mdblp->Mdbw[l].Current_Step[m] = mdblp->SS[l];
		mdblp->Mdbw[l].Current[m].Time = 0;
		Set_Start_Parameters(mdblp, l, m); // This must be called after setting of Current_Step
	}
	mdblp->Mdbw[l].Waiting_flag = (mdblp->SI[l].Index_num != 0)?1:0;
	mdblp->Change_flag = 1;
}

void	StartAllMotions(ATI_PACK* mdblp)
{
	int	l;
	for(l = 0; l < mdblp->ATIS; l++)
		StartMotion(mdblp, l);
}

void	StopAllMotions(ATI_PACK* mdblp)
{
	int	l;
	for(l = 0; l < mdblp->ATIS; l++)
		StopMotion(mdblp, l);
}

void	ResetAllMotions(ATI_PACK* mdblp)
{
	int	l;
	for(l = 0; l < mdblp->ATIS; l++)
		ResetMotion(mdblp, l);
}

