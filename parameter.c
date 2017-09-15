//Parameter of Robo
int TAIL_ANGLE_STAND_UP = 98; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_RUN      =  3; /* バランス走行時の角度[度] */
int TAIL_ANGLE_DANSA    = 85; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_LUG      = 75; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_GARAGE   = 94; /* 完全停止時の角度[度]     */

float WheelDiameter = 79.95;  //背面から見て左タイヤの直径[mm] 0817 tada
float WHEEL_R       = 39.975; //Wheel radius
int   RoboTread     = 160; //トレッド長さ[mm]

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
float RAD_315_DEG = 5.4978; //
float RAD_345_DEG = 6.0214; //
float RAD_360_DEG = 6.2832; //
float RAD_450_DEG = 7.8540;

//Parameter of Course
float FINAL_STRAIGHT_LENGTH = 1100.0;
float DEAD_ZONE_LENGTH      =  400.0; //0910 tada
float FST_DANSA_POS         =  260;
float SCD_DANSA_POS         =  260;

float STEP_TO_GARAGE_LENGTH = 1100;

//Parameter of Area

float LineTrace1Area[4]={0.0, 1000.0, 0.0, 2000.0};
float MapTraceArea1[4]={0.0, 1000.0, 2000.0, 2635.55};
float MapTraceArea2[4]={0.0, 2278.25, 2635.55, 3800.0};
float MapTraceArea3[4]={1000.0, 2278.25, 2313.53, 2635.55};
float MapTraceArea4[4]={1000.0, 1725.13, 1445.85, 2313.53};
float MapTraceArea5[4]={1000.0, 1725.13, 0.0, 1445.85};
float MapTraceArea6[4]={1725.13, 2142.17, 0.0, 2313.53};
float MapTraceArea7[4]={2142.17, 2521.64, 0.0, 2313.53};
float MapTraceArea8[4]={2521.64, 4200.0, 0.0, 2500};

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

float Gate12Area[4]={0.0, 1000.0, 2000.0, 2635.55};
float Gate23Area[4]={0.0, 2278.25, 2635.55, 3800.0};
float Gate34Area[4]={1000.0, 2278.25, 2313.53, 2635.55};
float Gate45Area[4]={1000.0, 1725.13, 1445.85, 2313.53};
float Gate56Area[4]={1000.0, 1725.13, 0.0, 1445.85};
float Gate67Area[4]={1725.13, 2142.17, 0.0, 2313.53};
float Gate78Area[4]={2142.17, 2521.64, 0.0, 2313.53};
float Gate89Area[4]={2521.64, 4200.0, 0.0, 2500};
//*/
