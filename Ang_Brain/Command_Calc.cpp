#include "Command_Calc.h"
#include "ev3api.h"
#include "Clock.h"
#include "math.h"

using ev3api::Clock;

#define liting_radius 10; // liting spot radius [mm]
//#define STEP_DEBUG
//#define GARAGE_DEBUG
//#define BALANCE_DEBUG


Clock*       gClock;

CommandCalc::CommandCalc(){

}

void CommandCalc::init( ){
  gClock       = new Clock();
  Track_Mode = Start_to_1st_Corner;
  Step_Mode  = Step_Start;

#ifdef STEP_DEBUG
  //Track_Mode = Return_to_Line;
  Track_Mode = Go_Step;
#endif

#ifdef BALANCE_DEBUG
  //Track_Mode = Return_to_Line;
  Track_Mode =  Track_Debug_00;
#endif
}

void CommandCalc::SetCurrentData(int   linevalue,
				 float xvalue,
				 float yvalue,
				 float odo,                     
				 float speed,
				 float yawrate,
				 float abs_angle,
				 int   robo_tail_angle,
				 bool  robo_stop,
				 bool  robo_forward,
				 bool  robo_back,
				 bool  robo_turn_left,
				 bool  robo_turn_right,
				 bool  dansa,
				 bool  robo_balance_mode,
				 int   max_forward,
				 float max_yawrate,
				 float min_yawrate

				 ) {

    mLinevalue         = linevalue;
    mXvalue            = xvalue;
    mYvalue            = yvalue;
    mOdo               = odo;
    mSpeed             = speed;
    mYawrate           = yawrate;
    mYawangle          = abs_angle;
    mTail_angle        = robo_tail_angle;
    mRobo_stop         = robo_stop;
    mRobo_forward      = robo_forward;
    mRobo_back         = robo_back;
    mRobo_turn_left    = robo_turn_left;
    mRobo_turn_right   = robo_turn_right;
    mDansa             = dansa;
    mRobo_balance_mode = robo_balance_mode;

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
    tail_mode_lflag = false;

    if(mYawangle < -2){
      Track_Mode = Snd_Corner;
    }
    break;

  case Snd_Corner:
    forward =  mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if(mYawangle > 2){
      Track_Mode = Final_Corner;
    }
    break;
    
  case Final_Corner:
    forward =  mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

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
    tail_mode_lflag = false;
    
    if(mOdo > ref_odo){
      Track_Mode = Dead_Zone;
      ref_odo = mOdo + DEAD_ZONE_LENGTH;
    }
    break;

  case Get_Ref_Odo:
    forward =  50;
    dammy_line_value = 50 - 300*mYawangle;
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
    dammy_line_value = 50 - 300*mYawangle;
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
//    forward =  50; // 0910 tada
    forward =  20; // 0910 tada
  	if(mYawangle < 0.16 && mLinevalue <20){
	    dammy_line_value = 80 - 300*mYawangle;
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
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if((mYawangle > 1) &&(mRobo_forward == 1)){
      Track_Mode = Go_Step;
    }

    break;

  case Go_Step:
    StepRunner(mLinevalue, mOdo, mYawangle, mDansa);
    ref_odo = mOdo + STEP_TO_GARAGE_LENGTH;
    break;

  case Approach_to_Garage:

    ref_odo = mOdo + STEP_TO_GARAGE_LENGTH;
    y_t = -0.5*((FIVE_PAI) - mYawangle);
    
    yawratecmd = y_t;

    gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    forward = 0.1*(gStep->CalcPIDContrInput(ref_odo, mOdo));
    
    anglecommand = TAIL_ANGLE_RUN;
    Track_Mode = Garage_In;

    break;
    
  case Return_to_Line_Garage:
    forward =  50;
    LineTracerYawrate((4*mLinevalue));
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;
    /*
    if((mOdo > 300)&&(mDet_gray)){
      Track_Mode = Garage_In;
      ref_odo = mOdo + GRAY_TO_GARAGE_LENGTH;
      }*/
    break;

  case Garage_In:



    y_t = -0.5*((FIVE_PAI+RAD_1_DEG) - mYawangle);
    yawratecmd = y_t;
    forward = 0.3*(gStep->CalcPIDContrInput(ref_odo, mOdo));
    anglecommand = TAIL_ANGLE_RUN;


    break;

  case Track_Debug_00:
    ref_odo = 0;

    anglecommand = TAIL_ANGLE_RUN;
    tail_mode_lflag = false;
    clock_start = gClock->now();
    Track_Mode = Track_Debug_01;
    break;

  case Track_Debug_01:

    if(gClock->now() - clock_start > 5000){
      tail_mode_lflag = true;
    }
    if(mRobo_balance_mode == false){
      clock_start = gClock->now();
      Track_Mode = Track_Debug_02;
    }
    break;

  case Track_Debug_02:
    anglecommand = TAIL_ANGLE_RUN;
    if(gClock->now() - clock_start > 3000){
      tail_mode_lflag = false;
    }
    if(mRobo_balance_mode == true){
      Track_Mode = Track_Debug_00;
    }
    break;

  default:
    forward = 0;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = true;
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
	  forward = 100;
	  LineTracerYawrate(mLinevalue);
	  anglecommand = TAIL_ANGLE_RUN; //0817 tada
	  tail_mode_lflag = false;
		
	break;


	case MapTrace1:
		forward = 100; //0827 tada
		MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
		anglecommand = TAIL_ANGLE_RUN; //0827 tada
		tail_mode_lflag = false; //0827 tada
	break;

	case MapTrace2:
		forward = 100; //0827 tada
		MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
		anglecommand = TAIL_ANGLE_RUN; //0827 tada
		tail_mode_lflag = false; //0827 tada
	break;

	case MapTrace3:
		forward = 100; //0827 tada
		MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
		anglecommand = TAIL_ANGLE_RUN; //0827 tada
		tail_mode_lflag = false; //0827 tada
	break;

	case MapTrace4:
		forward = 100; //0827 tada
		MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
		anglecommand = TAIL_ANGLE_RUN; //0827 tada
		tail_mode_lflag = false; //0827 tada
	break;

	case MapTrace5:
		forward = 100; //0827 tada
		MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
		anglecommand = TAIL_ANGLE_RUN; //0827 tada
		tail_mode_lflag = false; //0827 tada
	break;

	case MapTrace6:
		forward = 100; //0827 tada
		MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
		anglecommand = TAIL_ANGLE_RUN; //0827 tada
		tail_mode_lflag = false; //0827 tada
	break;

	case MapTrace7:
		forward = 100; //0827 tada
		MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
		anglecommand = TAIL_ANGLE_RUN; //0827 tada
		tail_mode_lflag = false; //0827 tada
	break;

	case MapTrace8:
		forward = 100; //0827 tada
		if(mLinevalue > 20){
			LineTracerYawrate(mLinevalue);
		}
		else{
			MapTracer(virtualgate_num, mXvalue, mYvalue, mYawangle); //0827 tada
		}
		anglecommand = TAIL_ANGLE_RUN; //0827 tada
		tail_mode_lflag = false; //0827 tada
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
		LookUpGateRunner();
	break;

	case Garage:
		GarageRunner();
	break;

	case Stop:
		StopRobo();
	break;

	default:
	  forward = 10;
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
/*ライン（攻）
	float Virtual_S1[4]={735.96,415.74,735.96, 2558.54};
	float Virtual_C1[3]={1083.86,2558.54,347.9};
	float Virtual_S2[4]={1425.89,2494.88,1211.73,1344.24};
	float Virtual_C2[3]={1458.8,1296.03,251.72};
	float Virtual_S3[4]={1592.76,1082.92,2487.29,1645.2};
	float Virtual_C3[3]={3024.44,790.66,1009.34};
	float Virtual_S4[4]={3024.44,1800,4613.06,1800};
*/
	/*ライン（安）１
	float Virtual_S1[4]={735.96,415.74,735.96, 2558.54};
	float Virtual_C1[3]={1083.86,2558.54,347.9};
	float Virtual_S2[4]={1426.36,2494.88,1210.99,1290.24};
	float Virtual_C2[3]={1458.8,1246.03,251.72};
	float Virtual_S3[4]={1601.26,1038.5,2487.14,1645.11};
	float Virtual_C3[3]={3302.03,349.04,1530.96};
	float Virtual_S4[4]={3302.03,1880.0,4600.0,2050.0};
	*/
///*
	float Virtual_S1[4]={327.19,415.74,327.19, 2635.55};
	float Virtual_C1[3]={1018.7,2635.55,691.51};
	float Virtual_S2[4]={1630.66,2313.53,1174.09,1445.85};
	float Virtual_C2[3]={1458.8,1296.03,321.72};
	float Virtual_S3[4]={1725.13,1115.55,2142.17,1730.95};
	float Virtual_C3[3]={2657.34,1372.47,628.86};
	float Virtual_S4[4]={2521.64,1986.52,4190.51,2342.51};
//*/
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
		x1 = Virtual_C1[0]*extend_gain;
		y1 = Virtual_C1[1]*extend_gain;
		a = x1 - x0;
		b = y1 - y0;
		a2 = a * a;
		b2 = b * b;
		r2 = a2 + b2;
		y_t = sqrt(r2) - Virtual_C1[2]*extend_gain;
	    if(y_t > 20.0) y_t = 20.0;
	    if(y_t < -20.0) y_t = -20.0;
		
//		y_t = -1.0*y_t;
		
    	yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
//		yawratecmd = 0.0;
	    y_t_prev = y_t;

	break;

	case Gate34:
	
		x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
		y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
		x1 = Virtual_C1[0]*extend_gain;
		y1 = Virtual_C1[1]*extend_gain;
		a = x1 - x0;
		b = y1 - y0;
		a2 = a * a;
		b2 = b * b;
		r2 = a2 + b2;
		y_t = sqrt(r2) - Virtual_C1[2]*extend_gain;
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

void CommandCalc::StepRunner(int line_value, float odo, float angle, bool dansa){
  /*前提条件：ロボットがライン上にあること*/

  float y_t;
  static float angle_change_right_edge_trace;
  static float target_odo;
  static float target_angle;
  static float target_tail_angle;
  static int32_t clock_start;
  
  switch(Step_Mode){

  case Step_Start:
    forward =  50;
    LineTracerYawrate((2*line_value));
    target_odo = odo + 500;
 

    clock_start = gClock->now();
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    
    Step_Mode = Approach_to_Step;   

#ifdef STEP_DEBUG
    Step_Mode   =  First_Dansa;   
    target_odo  = odo + 250; //for debug
    clock_start = gClock->now();
#endif

    break;

  case Approach_to_Step:

    if(odo > target_odo){
      forward =  20;
    }else{
      forward =  50;
    }

    LineTracerYawrate((2*line_value));
    if((angle >  RAD_90_DEG)&&(yawratecmd < 0) ){
      yawratecmd = 0.0;
    }
    if(dansa){
      Step_Mode = First_Dansa;
      target_odo = odo + FST_DANSA_POS;
    }


    break;

  case First_Dansa:
    if(odo > target_odo){
      forward = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode = First_Dansa_On;
      gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    }else{
      forward    = 20;
      yawratecmd = 0;
    }
    break;

  case First_Dansa_On:

    forward = gStep->CalcPIDContrInput(target_odo, odo);
    forward = forward * 0.1;
    yawratecmd = 0;
    anglecommand = target_tail_angle;


    if((gClock->now() - clock_start) > 5000){
      Step_Mode = First_Dansa_Tail_On;
      clock_start = gClock->now();
    }

  break;

  case First_Dansa_Tail_On:

    tail_mode_lflag = true;
    forward    = 0;
    yawratecmd = 0;
    if(mRobo_balance_mode == false){
      forward    = 0;
      yawratecmd = 0;
      Step_Mode = First_Turn;
      clock_start = gClock->now();
      target_angle = angle + RAD_360_DEG + RAD_15_DEG;
    }

#ifdef STEP_DEBUG
    
#endif

    break;

  case First_Turn:
    tail_mode_lflag = true;
    if(angle >= target_angle){
      //      Step_Mode = First_Dansa_Stand_Up;
      Step_Mode = First_Pre_Stand_Up;
      clock_start = gClock->now();
      forward = 0;
      yawratecmd = 0;
    }else{
      forward = 0;
      y_t = -2.0;
      yawratecmd = y_t;
    }						//目標yawrate値を更新
    break;

  case First_Pre_Stand_Up:
    
    forward = 10;
    y_t = -0.5*(2.5*PAI - angle);
    yawratecmd = y_t;
    tail_mode_lflag = true;

    if((gClock->now() - clock_start) > 1500){
      forward    = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
      clock_start = gClock->now();
      Step_Mode = First_Dansa_Stand_Up;
    }
    
    break;

  case First_Dansa_Stand_Up:

    if((gClock->now() - clock_start) > 3000){

      forward    = 0;
      yawratecmd = 0;

      anglecommand = TAIL_ANGLE_RUN;
      tail_mode_lflag = false;
      if(mRobo_balance_mode == true){
	forward    = 0;
	yawratecmd = 0;
	anglecommand = TAIL_ANGLE_RUN;
	Step_Mode = Approach_to_2nd_Step;
	clock_start = gClock->now();
      }
    }else{
      tail_mode_lflag = true;
      forward    = 0;
      yawratecmd = 0;
    }
#ifdef STEP_DEBUG

#endif
    break;

  case Approach_to_2nd_Step:
    

    if((gClock->now() - clock_start) < 5000){
      forward    = 0;
      yawratecmd = 0;
      anglecommand = TAIL_ANGLE_RUN;
    }else{
      forward = 10;

      y_t = -0.5*(2.5*PAI - angle);
#ifdef STEP_DEBUG
      //      y_t = -0.5*(2.0*PAI - angle);
#endif
      yawratecmd = y_t;


      if(dansa){
	forward    = 10;
	yawratecmd = 0;
	Step_Mode = Second_Dansa;
	target_odo = odo + SCD_DANSA_POS;
      }
    }

    break;
    
  case Second_Dansa:
   if(odo > target_odo){
      forward    = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode = Second_Dansa_On;
    }else{
      forward    = 15;
      yawratecmd = 0;
    }
    break;


  case Second_Dansa_On:
    gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    forward = gStep->CalcPIDContrInput(target_odo, odo);
    forward = forward * 0.1;
    yawratecmd = 0;
    anglecommand = target_tail_angle;

    /*    if(target_tail_angle < TAIL_ANGLE_DANSA){
      target_tail_angle = target_tail_angle + 0.1;
      target_tail_angle = target_tail_angle + 0.2;

      }*/

    if((gClock->now() - clock_start) > 5000){
      Step_Mode = Second_Dansa_Tail_On;
      clock_start = gClock->now();
      forward    = 0;
      yawratecmd = 0;
    }

  break;

  case Second_Dansa_Tail_On:
    forward    = 0;
    yawratecmd = 0;
    tail_mode_lflag = true;
    if(mRobo_balance_mode == false){
      forward    = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
      Step_Mode = Second_Turn;
      clock_start = gClock->now();
      target_angle = angle + RAD_450_DEG + RAD_5_DEG;
    }
    break;

  case Second_Turn:
    tail_mode_lflag = true;
    if((gClock->now() - clock_start) > 1000){
      if(angle >= target_angle){
	//Step_Mode = Second_Dansa_Stand_Up;
	Step_Mode = Second_Pre_Stand_Up;
	target_odo = odo + 50;
	clock_start = gClock->now();
	forward    = 0;
	yawratecmd = 0;
	
      }else{
	forward = 0;
	y_t     = -2.0;
	yawratecmd = y_t;						//目標yawrate値を更新
      }						//目標yawrate値を更新
    }
    else{
      forward = 0;
      yawratecmd = 0;
    }
    break;

  case Second_Pre_Stand_Up:
    
    forward = 10;
    y_t = -0.5*(5*PAI - angle);
    yawratecmd = y_t;
    tail_mode_lflag = true;

    if((gClock->now() - clock_start) > 1500){
      forward    = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
      clock_start = gClock->now();
      Step_Mode = Second_Dansa_Stand_Up;
    }
    
    break;


  case Second_Dansa_Stand_Up:

    if((gClock->now() - clock_start) > 15000){
      forward    = 0;
      yawratecmd = 0;
      anglecommand = TAIL_ANGLE_RUN;
      Step_Mode = Approach_to_Exit;
      clock_start = gClock->now();
      tail_mode_lflag = false;
    }

    if((gClock->now() - clock_start) > 3000){
      forward    = 0;
      yawratecmd = 0;

      anglecommand = TAIL_ANGLE_RUN;
      tail_mode_lflag = false;

      if(mRobo_balance_mode == true){
	forward    = 0;
	yawratecmd = 0;
	anglecommand = TAIL_ANGLE_RUN;
	Step_Mode = Approach_to_Exit;
	clock_start = gClock->now();

#ifdef STEP_DEBUG
	//    Step_Mode =  First_Dansa;   
	//    target_odo = odo + 250; //for debug
	//    clock_start = gClock->now();
#endif

      }
    }else{
      forward = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
    }


    break;

  case Approach_to_Exit:

    forward    = 20;

    y_t = -0.5*(5*PAI - angle);
#ifdef STEP_DEBUG
    //    y_t = -0.5*(4.5*PAI - angle);
#endif

    yawratecmd = y_t;


    if(dansa){
      forward    = 20;
      yawratecmd = 0;
      Track_Mode = Approach_to_Garage;
    }
    if(mOdo > target_odo){
      forward    = 20;
      yawratecmd = 0;
      Track_Mode = Approach_to_Garage;
    }


    break;

  case Change_Left_Edge_Trace:

    break;

  case End_of_Step:

    break;



  default:
    yawratecmd = 0;
    forward = 0;
    break;
  }
}

void CommandCalc::LookUpGateRunner(){

}

void CommandCalc::GarageRunner(){

}

void CommandCalc::StopRobo(){

}
