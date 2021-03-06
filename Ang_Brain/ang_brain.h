/******************************************************************************
 *  ang_brain.h (for LEGO Mindstorms EV3)
 *  Created on: 2017/07/25
 *  Implementation of the Class ang_brain
 *  Author: Keiichi Tomii
 *****************************************************************************/

#ifndef EV3_APP_ANAGOBRAIN_H_
#define EV3_APP_ANAGOBRAIN_H_
//#define DEQUE_EN

#include <deque>

#include "Command_Calc.h"
#include "Strategy_Det.h"

using namespace std;

class Ang_Brain {
public:
	explicit Ang_Brain();//コンストラクタ
	void init();
	void run();            //走行戦略を実行
	void setEyeCommand(int     linevalue,
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
			   int16_t sonar_dis);

	void setRoboCommand(bool robo_balance_mode, bool robo_lug_mode);

	void SetSysMode(int mode);

    //あなご手足に渡す変数

    int   forward;         //前進目標値
    float yawratecmd;      //目標ヨーレート
    float anglecommand;    //尻尾角度
    //    bool  tail_mode_lflag; //倒立走行フラグ
    bool  tail_stand_mode;
    bool  tail_lug_mode;

    int SysModeNum;
    int StrategyNum;   //走行戦略
    int VirtualGateNum;//仮想ゲート

    //手足モード

    float x;
    float y;
    float x_A;
    float y_A;
    float x_B;
    float y_B;
    float x_C;
    float y_C;
    float x_D;
    float y_D;


private:
    void GetStrategy(int strategy_num, int virtualgate_num);//走行戦略を取得
    void GetCalcResult(int forward_calc,
		       float yawratecmd_calc,
		       float anglecommand_calc,
		       bool tail_stand_mode_calc,
		       bool tail_lug_mode_calc);

    StrategyDet *gStrategyDet = new StrategyDet();
    CommandCalc *gCommandCalc = new CommandCalc();

    int   Mmode;
    int   mLinevalue;  //ライン検出値
    float mXvalue;     //x座標
    float mYvalue;     //y座標
    float mOdo;        //Total Distance from Start point
    float mSpeed;      //速度
    float mYawrate;    //ヨーレート
    float mYawangle;   //ヨー角
    int   mTail_angle;
    //signals for robo movement
    bool    mRobo_stop       = 0;
    bool    mRobo_forward    = 0;
    bool    mRobo_back       = 0;
    bool    mRobo_turn_left  = 0;
    bool    mRobo_turn_right = 0;
    bool    mDansa;      //段差検出値
    int16_t mSonar_dis;
    bool    mRobo_balance_mode;
    bool    mRobo_lug_mode;

    enum Sys_Mode{
      SYS_INIT = 110,
      BT_CONECT = 120,
      DISPLAY_SELECT_APLI = 210,
      CALIB_COLOR_SENSOR = 310,
      WAIT_FOR_START = 410,
      START = 510,
      RUN = 530,
      LINE_TRACE_MODE = 520,
      GOAL = 610,
      DANSA = 710,
      GARAGE = 810,
      LUP = 910,
      STOP = 1010,
      RESET = 0
    };

    Sys_Mode SysMode;

    enum enumStrategy{
      StartDash=510,
      LineTrace1=520,
      MapTrace=530,
      Goal=610,
      Goal2Step=650,
      Step=710,
      LookUpGate=810,
      Garage=910,
      Stop=1010
    };

    enum enumVirtualGate{
      Gate12=531,
      Gate23=532,
      Gate34=533,
      Gate45=534,
      Gate56=535,
      Gate67=536,
      Gate78=537,
      Gate89=538,
      None=539
    };

    enumStrategy Strategy;
    enumVirtualGate VirtualGate;

};

#endif  // EV3_APP_ANAGOBRAIN_H_
