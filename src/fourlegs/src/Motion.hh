#ifndef MOTION_H
#define MOTION_H

typedef	struct Angle_Timing_Information_Cell
{
	float	Angle;
	int		Time; // X 100
} ATI_CELL;

typedef	struct Angle_Timing_Information
{
	int			Motors, Motions;
	int*		Motor_Number; // -1 is No Operation.
	ATI_CELL*	Motion;
	int*		Index; // Index is on the Motor 0's timing.
} ATI; // Motion Data Base

typedef	struct Angle_Timing_Information_Working
{
	float*		Delta_Angle;
	ATI_CELL*	Current;
	int*		Current_Step;
	char		Waiting_flag; // Waiting_flag == 1 -> Waiting for Start Index.
	char		Active_flag; // Active_flag == 1 -> Now the motion is Active.
} ATI_WORK;

typedef struct Start_ID
{
	int	Motion_num;
	int	Index_num;
} START_ID;

typedef	struct ATI_PACK /* ATI_Package */
{
	int			ATIS; // The number of Motion Data Base
	ATI**		Mdb; // Array of pointer
	float*		MS; // Each motion speed
	float		SF; // Speed factor of this ATI_PACK
	int*		SS; // Start Step of each motion
	START_ID*	SI; // Start index of certain motion
	int			FDBS; // The number of Feedback Functions
	void		(**Fbf)(struct ATI_PACK *);
	ATI_WORK*	Mdbw;
	char		Change_flag;
	char		Reflex_flag;
} ATI_PACK;

extern void	Init_Motion(int embeded_motors);
extern void	Init_ATI(ATI* mdbp, int motors, int motions, int* motor_number_array
						, ATI_CELL* motion_array);
extern void	Set_Index_to_ATI(ATI* mdbp, int* index_array);
extern void	Disp_ATI(ATI* mdbp);
extern void	Clear_ATI_PACK(ATI_PACK* mdblp); // ATIS = 0, Mdbw = NULL;
extern void	Init_ATI_PACK(ATI_PACK* mdblp, int ATIS, ATI** Mdb, float* MS
													, int* SS, START_ID* SI);
extern void	Set_Speed_of_ATI_PACK(ATI_PACK* mdblp, float SF);
extern void	Set_Feedback_to_ATI_PACK(ATI_PACK* mdblp
								, int FDBS, void (**Fbf)(struct ATI_PACK *));
extern void	MotionPlayer(ATI_PACK* mdblp);
extern void	Set_FitSteps(int _fs); // You can set new fit steps by this. DEF=200.
extern void	StartMotion(ATI_PACK* mdblp, int l);
extern void	StopMotion(ATI_PACK* mdblp, int l);
extern void	ResetMotion(ATI_PACK* mdblp, int l);
extern void	StartAllMotions(ATI_PACK* mdblp);
extern void	StopAllMotions(ATI_PACK* mdblp);
extern void	ResetAllMotions(ATI_PACK* mdblp);

			// You should use this to set angle.
extern void	Add_Angle(int rmn, float angle);
			// You should use this to set angle. This don't make average.
extern void	Add_Angle_NA(int rmn, float angle);
extern int	Get_Angles(int rmn);
extern float	Get_Angle(int rmn);
extern float	Get_Angle_NA(int rmn);
extern int*	TmpAngles;
extern float*	TmpAngle;
extern float*	TmpAngle_NA;

#ifdef SAMPLE__
/************************************/
/*              Sample              */
/************************************/

/************************************/
/*       Feed Back Functions        */
/************************************/
void	FeedBackType1(struct ATI_PACK* mdblp) // Continuous Type
{
	/*
	// Write the feed back program which have calling Add_Angle_NA().
	*/
}

void	FeedBackType2(struct ATI_PACK* mdblp) // Motion Active Type
{
	if(0/* Check Switch */)
	{
		StartMotion(mdblp, 2);
	}
}

void	(*FB[])(ATI_PACK*) = {FeedBackType1, FeedBackType2};

/************************************/
/*      Sample Motion Data Base     */
/************************************/

int	FL_N[] = {0,4,8}; // Front-Left Leg's Motor Number
int	RR_N[] = {2,6,10}; // Rear-Left Leg's Motor Number
int	FR_N[] = {1,5,9}; // Front-Right Leg's Motor Number
int	RL_N[] = {3,7,11}; // Rear-Left Leg's Motor Number
ATI	STDFL;
ATI	STDRR;
ATI	STDFR;
ATI	STDRL;
ATI_CELL	STAND_M[] =	{{-30, 0}	// Motion Data by Each Motor
						,{  0, 0}
						,{-30, 0} };
ATI*		STAND_ATI[]= {&STDFR,&STDFL,&STDRR,&STDRL};
float		STAND_MS[] = {1, 1, 1, 1};
int			STAND_SS[] = {0, 0, 0, 0};
START_ID	STAND_SI[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
ATI_PACK	STAND_L;

void	Set_Stand_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array
	Init_ATI(&STDFR, 3, 1, FR_N, STAND_M);
	Init_ATI(&STDFL, 3, 1, FL_N, STAND_M);
	Init_ATI(&STDRR, 3, 1, RR_N, STAND_M);
	Init_ATI(&STDRL, 3, 1, RL_N, STAND_M);
//ATI_PACK, ATIs, {&ATI1, &ATI2..}, {Speed1, Speed2..}, {Start Step1, Start Step2..}, {Wait1, Wait2..}
	Clear_ATI_PACK(&STAND_L);
	Init_ATI_PACK(&STAND_L, 4, STAND_ATI, STAND_MS, STAND_SS, STAND_SI);
	Set_Speed_of_ATI_PACK(&STAND_L, 1.0);
	Set_Feedback_to_ATI_PACK(&STAND_L, 2, FB);
}

ATI	WLKFL;
ATI	WLKRR;
ATI	WLKFR;
ATI	WLKRL;
ATI_CELL  MDLGOF_M[] =	{{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25}// Motion Data by Each Motor
						,{ 30,  25},{ 30, 50},{  0, 50},{-30,  25},{-30, 25},{  0, 25}
						,{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25} };
ATI_CELL  MDLGOB_M[] =	{{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25}
						,{-30,  25},{-30, 50},{  0, 50},{ 30,  25},{ 30, 25},{  0, 25}
						,{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25} };
int	WLKFL_I[] = {0,1,2,3,4,5};// Motion Index
// ATIs, {&ATI1, &ATI2..}, {Speed1, Speed2..}, MasterSpeed, {Start Step1, Start Step2..}, {Wait1, Wait2..}
ATI*		WALK_ATI[]= {&WLKFL,&WLKRR,&WLKFR,&WLKRL};
float		WALK_MS[] = {1, 1, 1, 1};
int			WALK_SS[] = {5, 5, 2, 2};
START_ID	WALK_SI[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
ATI_PACK  WALK_L;

void	Set_Forward_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array
	Init_ATI(&WLKFL, 3, 6, FL_N, MDLGOF_M);
	Set_Index_to_ATI(&WLKFL, WLKFL_I);
	Init_ATI(&WLKRR, 3, 6, RR_N, MDLGOB_M);
	Init_ATI(&WLKFR, 3, 6, FR_N, MDLGOF_M);
	Init_ATI(&WLKRL, 3, 6, RL_N, MDLGOB_M);
}

void	Set_Backward_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array
	Init_ATI(&WLKFL, 3, 6, FL_N, MDLGOB_M);
	Set_Index_to_ATI(&WLKFL, WLKFL_I);
	Init_ATI(&WLKRR, 3, 6, RR_N, MDLGOF_M);
	Init_ATI(&WLKFR, 3, 6, FR_N, MDLGOB_M);
	Init_ATI(&WLKRL, 3, 6, RL_N, MDLGOF_M);
}

ATI_CELL  MDLTRNF_M[] =	{{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25}// Motion Data by Each Motor
						,{ 30,  25},{ 30, 50},{  0, 50},{-30,  25},{-30, 25},{  0, 25}
						,{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25} };
ATI_CELL  MDLTRNB_M[] =	{{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25}
						,{-30,  25},{-30, 50},{  0, 50},{ 30,  25},{ 30, 25},{  0, 25}
						,{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25} };

void	Set_TurnRight_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array
	Init_ATI(&WLKFL, 3, 6, FL_N, MDLTRNF_M);
	Set_Index_to_ATI(&WLKFL, WLKFL_I);
	Init_ATI(&WLKRR, 3, 6, RR_N, MDLTRNF_M);
	Init_ATI(&WLKFR, 3, 6, FR_N, MDLTRNB_M);
	Init_ATI(&WLKRL, 3, 6, RL_N, MDLTRNB_M);
}

void	Set_TurnLeft_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array, Index array
	Init_ATI(&WLKFL, 3, 6, FL_N, MDLTRNB_M);
	Set_Index_to_ATI(&WLKFL, WLKFL_I);
	Init_ATI(&WLKRR, 3, 6, RR_N, MDLTRNB_M);
	Init_ATI(&WLKFR, 3, 6, FR_N, MDLTRNF_M);
	Init_ATI(&WLKRL, 3, 6, RL_N, MDLTRNF_M);
}

void	Init_ML(void)
{
	Set_Stand_Motion();
	Set_Forward_Motion();
//ATI_PACK, ATIs, {&ATI1, &ATI2..}, {Speed1, Speed2..}, {Start Step1, Start Step2..}, {Wait1, Wait2..}
	Clear_ATI_PACK(&WALK_L);
	Init_ATI_PACK(&WALK_L, 4, WALK_ATI, WALK_MS, WALK_SS, WALK_SI);
	Set_Speed_of_ATI_PACK(&WALK_L, 0.5);
}

/***********************************************************************************************
 main
***********************************************************************************************/

/**********************
 Servo Number   sign
     and          of
   location    direction

		8   9      -   +
		4   5      +   -
		0   1      -   +
		| A |      | A |
		|   |      |   |
		3   2      +   -
		7   6      -   +
		11  10     +   -
		
**********************/

PWMServoInfo	ServoInfo[16] = { 
	/*SRV 0*/{MTU_CH0 , -1,   0, 60.0}, /*SRV 1*/{MTU_CH1 , +1,   0, 60.0}, 
	/*SRV 2*/{MTU_CH2 , -1,   5, 60.0}, /*SRV 3*/{MTU_CH3 , +1,   4, 60.0}, 
	/*SRV 4*/{MTU_CH4 , +1,  75, 60.0}, /*SRV 5*/{MTU_CH5 , -1, -71, 60.0}, 
	/*SRV 6*/{MTU_CH6 , +1,  77, 60.0}, /*SRV 7*/{MTU_CH7 , -1, -75, 60.0}, 
	/*SRV 8*/{MTU_CH8 , -1,  71, 60.0}, /*SRV 9*/{MTU_CH9 , +1, -67, 60.0}, 
	/*SRV10*/{MTU_CH10, -1,  71, 60.0}, /*SRV11*/{MTU_CH11, +1, -68, 60.0}, 
	/*SRV12*/{MTU_CH12, +1,   0, 60.0}, /*SRV13*/{MTU_CH13, +1,   0, 60.0}, 
	/*SRV14*/{MTU_CH14, +1,   0, 60.0}, /*SRV15*/{MTU_CH15, +1,   0, 60.0} };

char    txbuf[100],rxbuf[100];

void    INIT_SCI1_IRQ(void)
{
        SCI1_INIT(br57600, 1, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf));
        SetSRReg(0);
}

int main(void)
{
	int	l;
	ATI_PACK*	mdblp = &STAND_L;
	INIT_SCI1_IRQ();
	SCI1_OUT_STRING("Please Push any key!!\n");
	while(SCI1_IN_DATA_CHECK()==0&&SCI0_IN_DATA_CHECK()==0); // Waiting for Cable is connected
	Init_PWM16ch(PWMServoInfo);
	Power_Off_All_Servo();
	Init_Motion(16);
	Init_ML();
	Set_FitSteps(25);
	for(l = 0; l < 100000; l++);
	for(;;)
	{
		if(SCI1_IN_DATA_CHECK())
		{
			switch(SCI1_IN_DATA())
			{
				case ' ':	StopAllMotions(mdblp);
							break;
				case 'd':	mdblp = &STAND_L;
							StartAllMotions(mdblp);
							break;
				case 'e':	Set_Forward_Motion();
							mdblp = &WALK_L;
							ResetAllMotions(mdblp);
							StartAllMotions(mdblp);
							break;
				case 'c':	Set_Backward_Motion();
							mdblp = &WALK_L;
							ResetAllMotions(mdblp);
							StartAllMotions(mdblp);
							break;
				case 'f':	Set_TurnRight_Motion();
							mdblp = &WALK_L;
							ResetAllMotions(mdblp);
							StartAllMotions(mdblp);
							break;
				case 's':	Set_TurnLeft_Motion();
							mdblp = &WALK_L;
							ResetAllMotions(mdblp);
							StartAllMotions(mdblp);
							break;
			}
		}
		MotionPlayer(mdblp);
	}
	for(;;);
	return 0;
}

#endif


#endif
