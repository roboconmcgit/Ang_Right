#define OTA_ROBO
//define TADA_ROBO
//#define TOMY_ROBY

//Parameter of Robo
int TAIL_ANGLE_STAND_UP = 98; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_RUN      =  3; /* バランス走行時の角度[度] */
int TAIL_ANGLE_DANSA    = 85; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_LUG      = 65
; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_GARAGE   = 94; /* 完全停止時の角度[度]     */

float WheelDiameter = 79.95;  //背面から見て左タイヤの直径[mm] 0817 tada
float WHEEL_R       = 39.975; //Wheel radius
int   RoboTread     = 160; //トレッド長さ[mm]

/* fail rate 20%
float START_ROBO_FORWARD_VAL = 100;
float START_FORWARD_STEP     = 0.1;
*/

/* fail rate under 10%
float START_ROBO_FORWARD_VAL = 50;
float START_FORWARD_STEP     = 0.1;
*/
/* too slow
float START_ROBO_FORWARD_VAL = 25;
float START_FORWARD_STEP     = 0.1;
*/
float START_ROBO_FORWARD_VAL = 40;
float START_FORWARD_STEP     = 0.1;



//Parameter of time length unit
float dT_100ms = 0.1;
float dT_4ms   = 0.004;

float PAI         =  3.1472;
float FIVE_PAI    = 15.708;

float RAD_1_DEG   = 0.0175; //deg@1rad
float RAD_5_DEG   = 0.0873; //
float RAD_15_DEG  = 0.2618; //
float RAD_30_DEG  = 0.5236; //

float MINUS_RAD_5_DEG  = -0.0873; //
float MINUS_RAD_15_DEG = -0.2618; //
float MINUS_RAD_30_DEG = -0.5236; //

float RAD_90_DEG  = 1.5708; //
float RAD_120_DEG = 2.0944; //
float RAD_150_DEG = 2.6180; //
float RAD_315_DEG = 5.4978; //
float RAD_345_DEG = 6.0214; //
float RAD_360_DEG = 6.2832; //
float RAD_450_DEG = 7.8540;

//Parameter of Course
float FINAL_STRAIGHT_LENGTH = 1100.0;
float DEAD_ZONE_LENGTH      =  200.0; //0915 tada


float FST_DANSA_POS         =  260;
float SCD_DANSA_POS         =  260;

//LUG
float APPROACH_TO_LUG_LENGTH = 900;
//float STOP_POS_FROM_LUG      = 5;
float STOP_POS_FROM_LUG      = 10;

//float APPROACH_TO_1st_LUG    = 150;
//float APPROACH_TO_1st_LUG    = 170;
float APPROACH_TO_1st_LUG    = 160; //fix

//float APPROACH_TO_2nd_LUG    = 150;
//float APPROACH_TO_2nd_LUG    = 170;
//float APPROACH_TO_2nd_LUG    = 160;
float APPROACH_TO_2nd_LUG    = 165;

//float APPROACH_TO_3rd_LUG    = 150;
//float APPROACH_TO_3rd_LUG    = 170;
float APPROACH_TO_3rd_LUG    = 160;


//float LUG_1st_STOP           = 200;
//float LUG_1st_STOP           = 150;
//float LUG_1st_STOP           = 170;
float LUG_1st_STOP           = 180;

//float LUG_2nd_STOP           = 150;
//float LUG_2nd_STOP           = 170;
float LUG_2nd_STOP           = 180;


//float LUG_3rd_STOP           = 150;
float LUG_3rd_STOP           = 200;

#ifdef OTA_ROBO
float LUG_YAW_GAIN           = 2.0;
#endif

#ifdef TOMY_ROBO
float LUG_YAW_GAIN           = 30.0;
#endif

int   LUG_COL_VAL_OFFSET     = 60;
int   LUG_COL_VAL_GAIN       = 2;

//float LUG_GRAY_TO_GARAGE     = 410;
//float LUG_GRAY_TO_GARAGE     = 405;
float LUG_GRAY_TO_GARAGE     = 400;

//Parameter of Garage
float STEP_TO_GARAGE_LENGTH = 1100;
float LUG_TO_GARAGE_LENGTH  =  650;
float GARAGE_X_POS          = 1100;
float GARAGE_LENGTH         =  150;


//Parameter of Area

float LineTrace1Area[4]={0.0, 1000.0, 0.0, 2000.0};

#ifdef TADA_ROBO
float MapTraceArea1[4]={0.0, 1000.0, 2000.0, 2382.16};
float MapTraceArea2[4]={0.0, 1118.86, 2382.16, 3800.0};
float MapTraceArea3[4]={1118.86, 2278.25, 2313.53, 3800.0};
float MapTraceArea4[4]={1000.0, 1725.13, 1445.85, 2313.53};
float MapTraceArea5[4]={1000.0, 1725.13, 0.0, 1445.85};
float MapTraceArea6[4]={1725.13, 2142.17, 0.0, 2313.53};
float MapTraceArea7[4]={2142.17, 2521.64, 0.0, 2313.53};
float MapTraceArea8[4]={2521.64, 4200.0, 0.0, 2500};
#endif

#ifdef OTA_ROBO
//float MapTraceArea1[4]={    0.0,  1000.0,  2000.0,  2382.16 };
float MapTraceArea1[4]={    0.0,  1000.0,  2000.0,  2384.76 };

float MapTraceArea2[4]={    0.0,  1118.86, 2382.16, 3800.0  };
float MapTraceArea3[4]={ 1118.86, 2278.25, 2313.53, 3800.0  };
float MapTraceArea4[4]={ 1000.0,  1725.13, 1445.85, 2313.53 };
float MapTraceArea5[4]={ 1000.0,  1625.13,    0.0,  1445.85 };
float MapTraceArea6[4]={ 1625.13, 2142.17,    0.0,  2313.53 };
float MapTraceArea7[4]={ 2142.17, 2521.64,    0.0,  2313.53 };
float MapTraceArea8[4]={ 2521.64, 4200.0,     0.0,  2500    };
#endif

float StartArea[4]       = {-200.0,  200.0, -200.0,  500.0};
float First_Straight[4]  = {-200.0,  200.0,  500.0, 2000.0};
float First_Corner[4]    = {-200.0, 2000.0, 2000.0, 3500.0};
float Second_Corner[4]   = {-200.0, 2000.0, -200.0, 2000.0};
float Second_Straight[4] = {2000.0, 3000.0, 1000.0, 2000.0};
float GoalArea[4]        = {3000.0, 4000.0, 1000.0, 2000.0};
float Goal_to_Step[4]    = {4000.0, 6000.0, 1000.0, 2000.0};
float StepArea[4]        = {4000.0, 6000.0, 2000.0, 5000.0};

float LookUpGateArea[4]={0.0, 0.0, 0.0, 0.0};
float GarageArea[4]={0.0, 0.0, 0.0, 0.0};
float StopArea[4]={0.0, 0.0, 0.0, 0.0};

#ifdef TADA_ROBO
float Gate12Area[4]={0.0, 1000.0, 2000.0, 2382.16};
float Gate23Area[4]={0.0, 1118.86, 2382.16, 3800.0};
float Gate34Area[4]={1118.86, 2278.25, 2313.53, 3800.0};
float Gate45Area[4]={1000.0, 1725.13, 1445.85, 2313.53};
float Gate56Area[4]={1000.0, 1725.13, 0.0, 1445.85};
float Gate67Area[4]={1725.13, 2142.17, 0.0, 2313.53};
float Gate78Area[4]={2142.17, 2521.64, 0.0, 2313.53};
float Gate89Area[4]={2521.64, 4200.0, 0.0, 2500};
#endif

#ifdef OTA_ROBO
//float Gate12Area[4]={0.0, 1000.0, 2000.0, 2382.16};
float Gate12Area[4]={    0.0,  1000.0,  2000.0,  2384.76 };
float Gate23Area[4]={    0.0,  1118.86, 2382.16, 3800.0  };
float Gate34Area[4]={ 1118.86, 2278.25, 2313.53, 3800.0  };
float Gate45Area[4]={ 1000.0,  1725.13, 1445.85, 2313.53 };
float Gate56Area[4]={ 1000.0,  1625.13,    0.0,  1445.85 };
float Gate67Area[4]={ 1625.13, 2142.17,    0.0,  2313.53 };
float Gate78Area[4]={ 2142.17, 2521.64,    0.0,  2313.53 };
float Gate89Area[4]={ 2521.64, 4200.0,     0.0,  2500    };
#endif


//*/
