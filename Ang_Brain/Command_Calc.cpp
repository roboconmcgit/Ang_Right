#include "Command_Calc.h"
#include "ev3api.h"
#include "Clock.h"
#include "math.h"

using ev3api::Clock;

#define liting_radius 10; // liting spot radius [mm]

#define OTA_ROBO
//#define TADA_ROBO
//#define TOMY_ROBO

//#define STEP_DEBUG
//#define GARAGE_DEBUG
//#define BALANCE_DEBUG
//#define LUG_DEBUG
//#define SONAR_DEBUG


Clock*       gClock;

CommandCalc::CommandCalc(){

}

void CommandCalc::init( ){
  gClock       = new Clock();
  Track_Mode = Start_to_1st_Corner;
  Step_Mode  = Step_Start;
  LUG_Mode   = LUG_Start;

  mYaw_angle_offset = 0.0;
  tail_stand_mode   = false;
  tail_lug_mode     = false;
  ref_forward       = 0.0;

#ifdef STEP_DEBUG
  //Track_Mode = Return_to_Line;
  Track_Mode = Go_Step;
#endif

#ifdef BALANCE_DEBUG
  //Track_Mode = Return_to_Line;
  Track_Mode =  Track_Debug_00;
#endif

#ifdef LUG_DEBUG
  //  Track_Mode = Go_LUG;
  //  mYaw_angle_offset = PAI;
  Track_Mode = Return_to_Line;

#endif

#ifdef SONAR_DEBUG
  Track_Mode = Track_Debug_10;
#endif

}

void CommandCalc::SetCurrentData(int     linevalue,
				 float   xvalue,
				 float   yvalue,
				 float   odo,
				 float   speed,
				 float   yawrate,
				 float   abs_angle,
				 int     robo_tail_angle,
				 bool    robo_stop,
				 bool    robo_forward,
				 bool    robo_back,
				 bool    robo_turn_left,
				 bool    robo_turn_right,
				 bool    dansa,
				 int16_t sonar_dis,
				 bool    robo_balance_mode,
				 bool    robo_lug_mode,
				 int     max_forward,
				 float   max_yawrate,
				 float   min_yawrate

				 ) {

    mLinevalue         = linevalue;
    mXvalue            = xvalue;
    mYvalue            = yvalue;
    mOdo               = odo;
    mSpeed             = speed;
    mYawrate           = yawrate;
    mYawangle          = abs_angle + mYaw_angle_offset;
    mTail_angle        = robo_tail_angle;
    mRobo_stop         = robo_stop;
    mRobo_forward      = robo_forward;
    mRobo_back         = robo_back;
    mRobo_turn_left    = robo_turn_left;
    mRobo_turn_right   = robo_turn_right;
    mDansa             = dansa;
    mSonar_dis         = sonar_dis;
    mRobo_balance_mode = robo_balance_mode;
    mRobo_lug_mode     = robo_lug_mode;

    mMax_Forward = max_forward;
    mMax_Yawrate = max_yawrate;
    mMin_Yawrate = min_yawrate;

}

void CommandCalc::Track_run( ) {

  static float   ref_odo;
  static int32_t clock_start;
  int            dammy_line_value;

  switch(Track_Mode){

  case Start_to_1st_Corner:
    forward = mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;

    if(mYawangle < -2){
      Track_Mode = Snd_Corner;
    }
    break;

  case Snd_Corner:
    forward =  mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;

    if(mYawangle > 2){
      Track_Mode = Final_Corner;
    }
    break;

  case Final_Corner:
    forward =  mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;

    if((mYawangle < 1) &&(mRobo_forward == 1)){
      Track_Mode = Final_Straight;
      ref_odo = mOdo +  FINAL_STRAIGHT_LENGTH;
    }
    break;

  case Final_Straight:
    forward =  mMax_Forward;
    mMax_Yawrate = 1.0;
    mMin_Yawrate = -1.0;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;

    if(mOdo > ref_odo){
      Track_Mode = Dead_Zone;
      ref_odo = mOdo + DEAD_ZONE_LENGTH;
    }
    break;

  case Get_Ref_Odo:
    forward =  50;
    dammy_line_value = 50 - 300*(mYawangle-0.1);
    if(dammy_line_value > 100){
      dammy_line_value = 100;
    }else if(dammy_line_value < 0){
      dammy_line_value = 0;
    }

    LineTracerYawrate(dammy_line_value);
    ref_odo = mOdo + DEAD_ZONE_LENGTH;
    Track_Mode = Dead_Zone;

    break;

  case Dead_Zone:
    forward =  50;
    dammy_line_value = 50 - 300*(mYawangle-0.1);
    if(dammy_line_value > 100){
      dammy_line_value = 100;
    }else if(dammy_line_value < 0){
      dammy_line_value = 0;
    }

    LineTracerYawrate(dammy_line_value);

    if(mOdo > ref_odo){
      Track_Mode = Return_to_Line;
    }
    break;

  case Return_to_Line:

    forward =  20;
    if(mYawangle < 0.16 && mLinevalue <20){
      dammy_line_value = 50 - 300*(mYawangle-0.1);
      if(dammy_line_value > 100){
	dammy_line_value = 100;
      }else if(dammy_line_value < 0){
	dammy_line_value = 0;
      }
      LineTracerYawrate(dammy_line_value);
    }
    else{
      LineTracerYawrate(mLinevalue);
    }
    anglecommand = TAIL_ANGLE_RUN;
    tail_stand_mode = false;

    if((mYawangle > 2.5) &&(mRobo_forward == 1)){
      forward =  30;
      LineTracerYawrate((2*mLinevalue));
      Track_Mode = Go_LUG;
      ref_odo    = mOdo + APPROACH_TO_LUG_LENGTH;
    }
    break;

  case Go_LUG:
    forward =  30;
    LineTracerYawrate((2*mLinevalue));
    LookUpGateRunner(mLinevalue, mOdo, mYawangle,mLinevalue);
    break;

    //non used
  case Go_Step:

    break;

  case Approach_to_Garage:
    tail_stand_mode = true;
    ref_odo = mOdo + LUG_TO_GARAGE_LENGTH;
    y_t = -30.0*(PAI - mYawangle);
    yawratecmd = y_t;

    gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    forward = 0.1*(gStep->CalcPIDContrInput(ref_odo, mOdo));
    Track_Mode = Garage_In;
    break;

    //non used
  case Return_to_Line_Garage:
    forward =  50;
    LineTracerYawrate((4*mLinevalue));
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;

    break;

  case Garage_In:
    tail_stand_mode = true;
    y_t = -30.0*(PAI - mYawangle);
    yawratecmd = y_t;
    forward = 0.3*(gStep->CalcPIDContrInput(ref_odo, mOdo));

    break;

  case Track_Debug_00:
    ref_odo = 0;

    anglecommand = TAIL_ANGLE_RUN;
    tail_stand_mode = false;
    clock_start = gClock->now();
    Track_Mode = Track_Debug_01;
    break;

  case Track_Debug_01:

    if(gClock->now() - clock_start > 5000){
      tail_stand_mode = true;
    }
    if(mRobo_balance_mode == false){
      clock_start = gClock->now();
      Track_Mode = Track_Debug_02;
    }
    break;

  case Track_Debug_02:
      tail_stand_mode = true;

    if(gClock->now() - clock_start > 3000){
      tail_lug_mode = true;
      tail_stand_mode = true;
      clock_start = gClock->now();
      Track_Mode = Track_Debug_03;
    }

    break;

  case Track_Debug_03:

    if(gClock->now() - clock_start > 5000){
      tail_lug_mode   = false;
      tail_stand_mode = true;
      clock_start     = gClock->now();
      Track_Mode      = Track_Debug_04;
    }else{
      tail_stand_mode = true;
      tail_lug_mode   = true;
    }

    break;

  case Track_Debug_04:

    if(gClock->now() - clock_start > 5000){
      tail_stand_mode = false;
      tail_lug_mode   = false;
    }else{
      tail_stand_mode = true;
      tail_lug_mode   = false;
    }
    break;


    /*  case Track_Debug_02:
    anglecommand = TAIL_ANGLE_RUN;
    if(gClock->now() - clock_start > 3000){
      tail_stand_mode = false;
    }
    if(mRobo_balance_mode == true){
      Track_Mode = Track_Debug_00;
    }
    break;
    */


  case Track_Debug_10:

    forward         = 10;
    yawratecmd      = 0;
    anglecommand    = TAIL_ANGLE_RUN;
    tail_stand_mode = false;
    tail_lug_mode   = false;
    break;

  default:
    forward = 0;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;
    break;
  }
}

void CommandCalc::StrategyCalcRun(int strategy_num, int virtualgate_num, float xvalue, float yvalue, float yawangle) {

  Strategy=static_cast<enumStrategy>(strategy_num);

  switch(Strategy){
  case StartDash:
    StartDashRunner();
    break;

  case LineTrace1:
    if(mOdo > 20){
      if(mSpeed > 250){
	ref_forward = ref_forward;
      }else{
	ref_forward = ref_forward + START_FORWARD_STEP;
      }
    }else{
      ref_forward = 0.0;
    }

    forward = (int)(ref_forward + START_ROBO_FORWARD_VAL + 0.5);

    if(forward > 100){
      forward = 100;
      }

    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;

    break;


  case MapTrace1:
    forward = 100; //0827 tada
    MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
    anglecommand = TAIL_ANGLE_RUN; //0827 tada
    tail_stand_mode = false; //0827 tada
    break;

  case MapTrace2:
    forward = 100; //0827 tada
    MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
    anglecommand = TAIL_ANGLE_RUN; //0827 tada
    tail_stand_mode = false; //0827 tada
    break;

  case MapTrace3:
    forward = 100; //0827 tada
    MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
    anglecommand = TAIL_ANGLE_RUN; //0827 tada
    tail_stand_mode = false; //0827 tada
    break;

  case MapTrace4:
    forward = 100; //0827 tada
    MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
    anglecommand = TAIL_ANGLE_RUN; //0827 tada
    tail_stand_mode = false; //0827 tada
    break;

  case MapTrace5:
    forward = 100; //0827 tada
    MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
    anglecommand = TAIL_ANGLE_RUN; //0827 tada
    tail_stand_mode = false; //0827 tada
    break;

  case MapTrace6:
    forward = 100; //0827 tada
    if(mLinevalue > 90) line_detect_flag = 1;
    if(line_detect_flag == 1 && mLinevalue < 50) map2line_flag = 1;
    if(map2line_flag == 1){
    	if(mLinevalue < 0) mLinevalue = 0;
    	if(mLinevalue > 100) mLinevalue = 100;
      LineTracerYawrate(mLinevalue);
    	if(mYawangle < -0.4){
    		line_detect_flag = 0;
    		map2line_flag = 0;
    	}
    }
    else{
    MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
    }
    anglecommand = TAIL_ANGLE_RUN; //0827 tada
    tail_stand_mode = false; //0827 tada
    break;

  case MapTrace7:
    forward = 100; //0827 tada
    if(mLinevalue > 90) line_detect_flag = 1;
    if(line_detect_flag == 1 && mLinevalue < 50) map2line_flag = 1;
    if(map2line_flag == 1){
    	if(mLinevalue < 0) mLinevalue = 0;
    	if(mLinevalue > 100) mLinevalue = 100;
      LineTracerYawrate(mLinevalue);
    	if(mYawangle < -0.4){
    		line_detect_flag = 0;
    		map2line_flag = 0;
    	}
    }
    else{
    MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
    }
    anglecommand = TAIL_ANGLE_RUN; //0827 tada
    tail_stand_mode = false; //0827 tada
    break;

  case MapTrace8:
    forward = 100; //0827 tada
    if(mLinevalue > 90) line_detect_flag = 1;
    if(line_detect_flag == 1 && mLinevalue < 50) map2line_flag = 1;
    if(map2line_flag == 1){
    	if(mLinevalue < 0) mLinevalue = 0;
    	if(mLinevalue > 100) mLinevalue = 100;
      LineTracerYawrate(mLinevalue);
    	if(mYawangle < -0.4){
    		line_detect_flag = 0;
    		map2line_flag = 0;
    	}
    }
    else{
      MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
    }
    anglecommand = TAIL_ANGLE_RUN; //0827 tada
    tail_stand_mode = false; //0827 tada
    Track_Mode = Get_Ref_Odo;//0910 tada
    break;

  case Goal:

    break;

  case Goal2Step:

    break;

  case Step:
    gForward->init_pid(0.1,0.005,0.05,dT_4ms);
    StepRunner(mLinevalue, mOdo, mYawangle, mDansa);
    break;

  case LookUpGate:

    break;

  case Garage:

    break;

  case Stop:

    break;

  default:

    break;
  }

}

void CommandCalc::StartDashRunner(){

}
//17.07.31 k-tomii add for position estimation
//ライントレースプログラム
void CommandCalc::LineTracer(int line_value,float traceforward) {

	const int LineTraceCommand=80;			//目標ライン値
	const float KP=0.5,KI=0.005,KD=0.05;			//PIDゲインの設定

	static int error_old=0,error_P_old=0;		//過去の偏差
	static float u=0;							//制御入力

	float u_delta=0;							//制御入力の差分
	int error=0,error_P=0,error_I=0,error_D=0;	//偏差
	float u_P_delta=0,u_I_delta=0,u_D_delta=0;	//制御入力の差分

	error=LineTraceCommand-(line_value);			//制御偏差を計算
	error_P=error-error_old;					//P制御用の偏差を計算
	error_I=error;								//I制御用の偏差を計算
	error_D=error_P-error_P_old;				//D制御用の偏差を計算

	u_P_delta=KP*error_P;						//P制御用の入力差分を計算
	u_I_delta=KI*error_I;						//I制御用の入力差分を計算
	u_D_delta=KD*error_D;						//D制御用の入力差分を計算

	u_delta=u_P_delta+u_I_delta+u_D_delta;		//PID制御入力の差分を計算
	u=u+u_delta;								//制御入力を計算

	//入力制限
	if(u>100){
		u=100;
	}else if(u<-100){
		u=-100;
	}

	yawratecmd=-u;										//目標turn値を更新
	forward=traceforward;						//目標forward値を更新

	error_old=error;							//過去の偏差を保存
	error_P_old=error_P;						//過去の偏差を保存

}

//2017/08/06多田さんライントレーサー
void CommandCalc::LineTracerYawrate(int line_value) {

    y_t = -1.0*(((float)line_value-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
    if(y_t > 10.0) y_t = 10.0;
    if(y_t < -10.0) y_t = -10.0;
	y_t = y_t + 7.0*(y_t/8.0)*(y_t/8.0)*(y_t/8.0);
//    yawratecmd = y_t/4.0;
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));

    if(yawratecmd > mMax_Yawrate){
      yawratecmd =  mMax_Yawrate;

    }else if (yawratecmd < mMin_Yawrate){
      yawratecmd = mMin_Yawrate;
    }else{
      yawratecmd = yawratecmd;
    }

    y_t_prev = y_t;

}

void CommandCalc::MapTracer(int virtualgate_num, float mXvalue, float mYvalue, float mYawangle) {

/*
	float Virtual_S1[4]={327.19,415.74,327.19, 2635.55};
	float Virtual_C1[3]={1018.7,2635.55,691.51};
	float Virtual_S2[4]={1630.66,2313.53,1174.09,1445.85};
	float Virtual_C2[3]={1458.8,1296.03,321.72};
//	float Virtual_S3[4]={1725.13,1115.55,2142.17,1730.95};
//	float Virtual_C3[3]={2657.34,1372.47,628.86};
//	float Virtual_S4[4]={2521.64,1986.52,4190.51,2342.51};
	float Virtual_S3[4]={1725.13,1115.55,2142.17,1730.95+100};
	float Virtual_C3[3]={2657.34,1372.47+100,628.86};
	float Virtual_S4[4]={2521.64,1986.52+100,4190.51,2342.51+200};
*/
#ifdef TADA_ROBO
	float Virtual_S1[4]={327.19,415.74,327.19, 2384.76};
	float Virtual_C1_a[3]={1118.86,2384.76,791.68};
	float Virtual_C1_b[3]={1118.86,2593.54,583.34};
	float Virtual_S2[4]={1630.66,2313.53,1174.09,1445.85};
	float Virtual_C2[3]={1458.8,1296.03,321.72};
	float Virtual_S3[4]={1725.13,1115.55,2142.17,1730.95+100};
	float Virtual_C3[3]={2657.34,1372.47+100,628.86};
	float Virtual_S4[4]={2521.64,1986.52+100,4190.51,2342.51+200};
#endif

#ifdef OTA_ROBO
	float Virtual_S1[4]  = {  327.19,  415.74 ,     327.19, 2384.76     };
	float Virtual_C1_a[3]= { 1118.86, 2384.76,      791.68              };
	float Virtual_C1_b[3]= { 1118.86, 2593.54,      583.34              };
	float Virtual_S2[4]  = { 1630.66, 2313.53,     1174.09, 1445.85     };
	float Virtual_C2[3]  = { 1458.8,  1296.03,      321.72              };
	float Virtual_S3[4]  = { 1725.13, 1115.55,     2142.17, 1730.95+100 };
	float Virtual_C3[3]  = { 2657.34, 1372.47+100,  628.86              };
	float Virtual_S4[4]  = { 2521.64, 1986.52+100, 4190.51, 2342.51+200 };
#endif


	float extend_gain = 1.0;
	float Virtual_point_dist = 50.0;

	float x0,x1,x2,y0,y1,y2,a,a2,b,b2,r2,c,d,tt,f1,x10,y10,x12,y12;

	VirtualGate=static_cast<enumVirtualGate>(virtualgate_num);

	switch(VirtualGate){
	case Gate12:
//	    y_t = -1.0*(((float)50-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
		x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
		y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
		x1 = Virtual_S1[0]*extend_gain;
		y1 = Virtual_S1[1]*extend_gain;
		x2 = Virtual_S1[2]*extend_gain;
		y2 = Virtual_S1[3]*extend_gain;
		x12 = x2-x1;
		y12 = y2-y1;
		x10 = x0-x1;
		y10 = y0-y1;
		a = (x12*y10)-(y12*x10);
		y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));

	    if(y_t > 20.0) y_t = 20.0;
	    if(y_t < -20.0) y_t = -20.0;

//		y_t = -1.0*y_t;

    	yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
//    	yawratecmd = 0.0;
	    y_t_prev = y_t;

	break;

	case Gate23:

		x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
		y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
		x1 = Virtual_C1_a[0]*extend_gain;
		y1 = Virtual_C1_a[1]*extend_gain;
		a = x1 - x0;
		b = y1 - y0;
		a2 = a * a;
		b2 = b * b;
		r2 = a2 + b2;
		y_t = sqrt(r2) - Virtual_C1_a[2]*extend_gain;
	    if(y_t > 20.0) y_t = 20.0;
	    if(y_t < -20.0) y_t = -20.0;
    	yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
	    y_t_prev = y_t;

	break;

	case Gate34:

		x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
		y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
		x1 = Virtual_C1_b[0]*extend_gain;
		y1 = Virtual_C1_b[1]*extend_gain;
		a = x1 - x0;
		b = y1 - y0;
		a2 = a * a;
		b2 = b * b;
		r2 = a2 + b2;
		y_t = sqrt(r2) - Virtual_C1_b[2]*extend_gain;
	    if(y_t > 20.0) y_t = 20.0;
	    if(y_t < -20.0) y_t = -20.0;

//		y_t = -1.0*y_t;

    	yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
//		yawratecmd = 0.0;
	    y_t_prev = y_t;
		break;

	case Gate45:
		x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
		y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
		x1 = Virtual_S2[0]*extend_gain;
		y1 = Virtual_S2[1]*extend_gain;
		x2 = Virtual_S2[2]*extend_gain;
		y2 = Virtual_S2[3]*extend_gain;
		x12 = x2-x1;
		y12 = y2-y1;
		x10 = x0-x1;
		y10 = y0-y1;
		a = (x12*y10)-(y12*x10);
		y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));

	    if(y_t > 20.0) y_t = 20.0;
	    if(y_t < -20.0) y_t = -20.0;

//		y_t = -1.0*y_t;

    	yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
//    	yawratecmd = 0.0;
	    y_t_prev = y_t;

	break;

	case Gate56:

		x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
		y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
		x1 = Virtual_C2[0]*extend_gain;
		y1 = Virtual_C2[1]*extend_gain;
		a = x1 - x0;
		b = y1 - y0;
		a2 = a * a;
		b2 = b * b;
		r2 = a2 + b2;
		y_t = sqrt(r2) - Virtual_C2[2]*extend_gain;
	    if(y_t > 20.0) y_t = 20.0;
	    if(y_t < -20.0) y_t = -20.0;

		y_t = -1.0*y_t;

    	yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
//		yawratecmd = 0.0;
	    y_t_prev = y_t;	break;


	break;

	case Gate67:
		x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
		y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
		x1 = Virtual_S3[0]*extend_gain;
		y1 = Virtual_S3[1]*extend_gain;
		x2 = Virtual_S3[2]*extend_gain;
		y2 = Virtual_S3[3]*extend_gain;
		x12 = x2-x1;
		y12 = y2-y1;
		x10 = x0-x1;
		y10 = y0-y1;
		a = (x12*y10)-(y12*x10);
		y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));

	    if(y_t > 20.0) y_t = 20.0;
	    if(y_t < -20.0) y_t = -20.0;

//		y_t = -1.0*y_t;

    	yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
//    	yawratecmd = 0.0;
	    y_t_prev = y_t;

	break;

	case Gate78:

		x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
		y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
		x1 = Virtual_C3[0]*extend_gain;
		y1 = Virtual_C3[1]*extend_gain;
		a = x1 - x0;
		b = y1 - y0;
		a2 = a * a;
		b2 = b * b;
		r2 = a2 + b2;
		y_t = sqrt(r2) - Virtual_C3[2]*extend_gain;
	    if(y_t > 20.0) y_t = 20.0;
	    if(y_t < -20.0) y_t = -20.0;

//		y_t = -1.0*y_t;

    	yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
//		yawratecmd = 0.0;
	    y_t_prev = y_t;	break;

	break;

	case Gate89:
		x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
		y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
		x1 = Virtual_S4[0]*extend_gain;
		y1 = Virtual_S4[1]*extend_gain;
		x2 = Virtual_S4[2]*extend_gain;
		y2 = Virtual_S4[3]*extend_gain;
		x12 = x2-x1;
		y12 = y2-y1;
		x10 = x0-x1;
		y10 = y0-y1;
		a = (x12*y10)-(y12*x10);
		y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));

	    if(y_t > 20.0) y_t = 20.0;
	    if(y_t < -20.0) y_t = -20.0;

//		y_t = -1.0*y_t;

    	yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
//    	yawratecmd = 0.0;
	    y_t_prev = y_t;

	break;

	default:
		yawratecmd = 0.0;
//		forward = 0;

	break;
	}

}

//There are latest code bellow
//https://github.com/roboconmcgit/Ang_vGate_Left
void CommandCalc::StepRunner(int line_value, float odo, float angle, bool dansa){
  /*前提条件：ロボットがライン上にあること*/

  switch(Step_Mode){

  case Step_Start:
    forward =  0;
    LineTracerYawrate((2*line_value));
    break;


  default:
    yawratecmd = 0;
    forward = 0;
    break;
  }
}

void CommandCalc::LookUpGateRunner(int line_value_lug, float odo, float angle,int line_value){

  static int32_t clock_start;
  static float ref_forward;
  static float ref_yaw;
  static float y_t;
  static float ref_odo;
  static int   stable_cnt;
  static int   pre_sonar_dis;
  static int   min_sonar_dis;
  int          dammy_line_value;

  switch(LUG_Mode){

  case LUG_Start:
    forward = 50;
    LineTracerYawrate((2*line_value));
    LUG_Mode      = Approach_to_LUG;
    ref_forward   = 0.0;
    ref_yaw       = 0.0;
    ref_odo       = odo +  APPROACH_TO_LUG_LENGTH;
    pre_sonar_dis = mSonar_dis;
    gForward->init_pid(0.05,0.01,0.001,dT_4ms);
    break;

  case Approach_to_LUG:

    ref_forward = (ref_odo - odo)/10.0+0.5;

    if(ref_forward > 70){
      ref_forward = 70;
    }else if(ref_forward < 10){
      ref_forward = 10;
    }else{
      ref_forward = ref_forward;
    }
    forward = (int)ref_forward;
    LineTracerYawrate((2*line_value));

    if(mSonar_dis <= STOP_POS_FROM_LUG){
      forward     = 0;
      yawratecmd  = 0;
      stable_cnt  = 0;
      LUG_Mode    = Tail_On_1st;
    }
    break;

  case Tail_On_1st:
    tail_stand_mode = true;
    tail_lug_mode  = false;

    forward    = 0;
    yawratecmd = 0;
    if(mRobo_balance_mode == false){
      forward    = 0;
      yawratecmd = 0;
      ref_odo    = odo + APPROACH_TO_1st_LUG;
      LUG_Mode   = POS_ADJ_1st;
    }
    break;

  case POS_ADJ_1st:

    if(odo < ref_odo){

#ifdef OTA_ROBO
      forward         = 15;
#endif

#ifdef TOMY_ROBO
      forward         = 30;
#endif

      y_t             = -LUG_YAW_GAIN*(PAI - angle);
      yawratecmd      = y_t;
      tail_stand_mode = true;
      tail_lug_mode  = false;
    }else{
      forward         = 0;
      yawratecmd      = 0;
      tail_stand_mode = true;
      tail_lug_mode   = false;
      LUG_Mode        = LUG_Mode_1st;
    }
    break;


  case LUG_Mode_1st:
    forward      = 0;
    ref_forward  = 0.0;
    yawratecmd   = 0;
    tail_lug_mode  = true;

    if(mRobo_lug_mode == true){
      ref_odo       = odo + LUG_1st_STOP;
      min_sonar_dis = 10;
      LUG_Mode      = LUG_1st;

    }
    break;

  case LUG_1st:

    ref_forward = ref_forward+0.1; //modify later
    forward     = (int)(ref_forward + 0.5);

#ifdef OTA_ROBO
    if(forward >= 10){
      forward = 10;
    }
#endif

#ifdef TOMY_ROBO
    if(forward >= 30){
      forward = 30;
    }
#endif


    y_t = -LUG_YAW_GAIN*(PAI - angle);
    yawratecmd = y_t;

    /*
    if(mSonar_dis < min_sonar_dis){
      min_sonar_dis = mSonar_dis;
    }
    if((min_sonar_dis < 7)&&(mSonar_dis > 100)){
      ref_odo = odo + 100;
      min_sonar_dis = 10;
    }
    */

    if(odo > ref_odo){
      LUG_Mode    = Pre_1st_Turn;
    }
    break;

  case Pre_1st_Turn:
    forward       = 0;
    yawratecmd    = 0;
    tail_lug_mode = false;

    if(mRobo_lug_mode == false){
      LUG_Mode    = Turn_1st;
    }

    break;

  case Turn_1st:
    if(angle < 0){
      forward     = 0;
      yawratecmd  = 0;
      ref_odo     = odo + APPROACH_TO_2nd_LUG;
      LUG_Mode        =  Approach_to_2nd_LUG;
    }else{
      forward = 0;
      y_t = y_t + 0.005;
      if(y_t >= 1){
	y_t = 1;
      }
      yawratecmd = y_t;
    }
    break;

  case Approach_to_2nd_LUG:
    if(odo < ref_odo){

#ifdef OTA_ROBO
      forward         = 15;
#endif

#ifdef TOMY_ROBO
      forward         = 30;
#endif

      y_t             = -LUG_YAW_GAIN*(0 - angle);
      yawratecmd      = y_t;
      tail_stand_mode = true;
      tail_lug_mode   = false;
    }else{
      forward         = 0;
      yawratecmd      = 0;
      tail_stand_mode = true;
      tail_lug_mode   = false;
      LUG_Mode        = LUG_Mode_2nd;
    }
    break;

  case LUG_Mode_2nd:
    forward       = 0;
    yawratecmd    = 0;
    tail_lug_mode = true;

    if(mRobo_lug_mode == true){
      ref_odo     = odo + LUG_2nd_STOP;
      ref_forward  = 0.0;
      LUG_Mode    = LUG_2nd;
    }
    break;

  case LUG_2nd:

    ref_forward = ref_forward+0.1; //modify later
    forward     = (int)(ref_forward + 0.5);

#ifdef OTA_ROBO
    if(forward >= 10){
      forward = 10;
    }
#endif

#ifdef TOMY_ROBO
    if(forward >= 30){
      forward = 30;
    }
#endif


    y_t = -LUG_YAW_GAIN*(0 - angle);
    yawratecmd = y_t;

    if(odo > ref_odo){
      LUG_Mode    = Pre_2nd_Turn;
    }
    break;

  case Pre_2nd_Turn:
    forward       = 0;
    yawratecmd    = 0;
    tail_lug_mode = false;

    if(mRobo_lug_mode == false){
      LUG_Mode    = Turn_2nd;
    }

    break;



  case Turn_2nd:
    if(angle > PAI){
          forward     = 0;
          yawratecmd  = 0;
	  LUG_Mode    = Approach_to_3rd_LUG;
	  ref_odo     = odo + APPROACH_TO_3rd_LUG;
      }else{
	forward = 0;
	y_t = y_t - 0.005;
	if(y_t <= -1){
	  y_t = -1;
	}
	yawratecmd = y_t;
      }
      break;

  case Approach_to_3rd_LUG:
    if(odo < ref_odo){

#ifdef OTA_ROBO
      forward         = 15;
#endif


#ifdef TOMY_ROBO
      forward         = 30;
#endif

      y_t             = -LUG_YAW_GAIN*(PAI - angle);
      yawratecmd      = y_t;
      tail_stand_mode = true;
      tail_lug_mode   = false;
    }else{
      forward         = 0;
      yawratecmd      = 0;
      tail_stand_mode = true;
      tail_lug_mode   = false;
      LUG_Mode        = LUG_Mode_3rd;
    }
    break;

  case LUG_Mode_3rd:
    forward      = 0;
    yawratecmd   = 0;
    tail_lug_mode  = true;

    if(mRobo_lug_mode == true){
      ref_odo      = odo + LUG_3rd_STOP;
      ref_forward  = 0.0;
      LUG_Mode     = LUG_3rd;
    }
    break;

  case LUG_3rd:

    ref_forward = ref_forward+0.1; //modify later
    forward     = (int)(ref_forward + 0.5);


#ifdef OTA_ROBO
    if(forward >= 10){
      forward = 10;
    }
#endif

#ifdef TOMY_ROBO
    if(forward >= 30){
      forward = 30;
    }
#endif

    y_t = -LUG_YAW_GAIN*(PAI - angle);
    yawratecmd = y_t;

    if(odo > ref_odo){
      LUG_Mode    = Tail_Stand_Up;
    }
    break;

  case Tail_Stand_Up:
    forward       = 0;
    yawratecmd    = 0;
    tail_lug_mode = false;

    if(mRobo_lug_mode == false){
      //      Track_Mode = Approach_to_Garage;
      LUG_Mode    = FIND_LEFT_EDGE;
      clock_start = gClock->now();
    }
    break;

  case FIND_LEFT_EDGE:

    if(gClock->now() - clock_start > 2000){
      forward       = 15;
    }else{
      forward       = 0;
    }

    dammy_line_value =  LUG_COL_VAL_GAIN*(line_value -  LUG_COL_VAL_OFFSET);
    if(dammy_line_value > 100){
      dammy_line_value = 100;
    }else if(dammy_line_value < 0){
      dammy_line_value = 0;
    }
    LineTracerYawrate(dammy_line_value);

    //det gray zone
    if(angle <  RAD_120_DEG){
      forward       = 0;
    } 
    if(angle <  RAD_90_DEG){
      forward     = 0;
      LUG_Mode    = GRAY_GARAGE;
      ref_odo     = odo + LUG_GRAY_TO_GARAGE;
      clock_start = gClock->now();
    } 

    break;

  case GRAY_GARAGE:

    ref_forward = (ref_odo - odo)/10.0+0.5;

    if(ref_forward > 70){
      ref_forward = 70;
    }else if(ref_forward < 0){
      ref_forward = 0;
    }else{
      ref_forward = ref_forward;
    }
    forward = (int)ref_forward;

    y_t = -2.0*(PAI - angle);
    yawratecmd = y_t;
    
    if(ref_odo - odo < 10){
    forward    = 0;
    yawratecmd = 0;
    }

    break;



  default:
    forward      = 0;
    yawratecmd    = 0;
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;
    break;

    }
}





void CommandCalc::GarageRunner(){

}

void CommandCalc::StopRobo(){

}
