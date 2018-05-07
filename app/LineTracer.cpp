/******************************************************************************
 *  LineTracer.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/26
 *  Implementation of the Class LineTracer
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "LineTracer.h"

/**
 * コンストラクタ
 * @param lineMonitor     ライン判定
 * @param balancingWalker 倒立走行
 */
LineTracer::LineTracer(const LineMonitor* lineMonitor,
                       BalancingWalker* balancingWalker,
                       PidController* mPidController,
					   ev3api::Motor& leftWheel,
					    ev3api::Motor& rightWheel)
    : mLineMonitor(lineMonitor),
      mBalancingWalker(balancingWalker),
      mPidController(mPidController),
	  mLeftWheel(leftWheel),
	  mRightWheel(rightWheel),
      mIsInitialized(false) {
}

/**
 * デストラクタ
 */
LineTracer::~LineTracer() {
}

/**
 * ライントレースする
 */
/* NORMAL*/
void LineTracer::run() {
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    // 走行体の向きを計算する
    mPidController->setPidFactor(1.40, 0.8, 0.1);//1.0, 0.2, 0.02
    int direction = calcDirection();

    mBalancingWalker->setCommand(80, direction);//速度

    // 倒立走行を行う
    mBalancingWalker->run();
}
void LineTracer::run2() {
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    // 走行体の向きを計算する
    mPidController->setPidFactor(3.80, 0.8, 0.15);
    int direction = calcDirection();

    mBalancingWalker->setCommand(80, direction);

    // 倒立走行を行う
    mBalancingWalker->run();
}
void LineTracer::run3() {
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    // 走行体の向きを計算する
    mPidController->setPidFactor(1.0, 0.01, 0.1);
    int direction = calcDirection();

    mBalancingWalker->setCommand(35, direction);

    // 倒立走行を行う
    mBalancingWalker->run();
}

/* 90 */
void LineTracer::run90(){
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    // 走行体の向きを計算する
    mPidController->setPidFactor(0.5, 0.01, 0.05);
    int direction = calcDirection();

    mBalancingWalker->setCommand(95, direction);

    // 倒立走行を行う
    mBalancingWalker->run();
}
/* LOW */
void LineTracer::runLOW(){
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    // 走行体の向きを計算する
    mPidController->setPidFactor(1.0, 0.01, 0.1);
    int direction = calcDirection();

    mBalancingWalker->setCommand(BalancingWalker::LOW, direction);

    // 倒立走行を行う
    mBalancingWalker->run();
}

void LineTracer::run80(){
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

	// 走行体の向きを計算する
    mPidController->setPidFactor(1.40, 0.8, 0.1);//2.80, 0.10, 0.5);
    int direction = calcDirection();

    mBalancingWalker->setCommand(80, direction);

    // 倒立走行を行う
    mBalancingWalker->run();
}

/* バランス崩す状態の走行 */
void LineTracer::runOnOff(){
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    bool isOnLine = mLineMonitor ->isOnLine99();//ライン上判定（ジャイロセンサ）
	//走行を行う
	if(isOnLine == true){
		mLeftWheel.setPWM(15);
		mRightWheel.setPWM(5);
	}
	else{
		mLeftWheel.setPWM(5);
		mRightWheel.setPWM(15);
	}
}
/* バランス崩す状態の走行 */
int LineTracer::runOnOff2(){
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    int isOnLine = mLineMonitor->isOnLine3();//ライン上判定（カラーセンサ）
	//走行を行う
	if(isOnLine == 2 ){//灰色見つける
		mLeftWheel.setPWM(0);
		mRightWheel.setPWM(0);
		return 1;
	}
	if(isOnLine == 1){
		mLeftWheel.setPWM(40);
		mRightWheel.setPWM(5);
	}
	if(isOnLine == 3 || isOnLine == 0){
		mLeftWheel.setPWM(5);
		mRightWheel.setPWM(15);
        return 3;
	}

	return 0;
}
void LineTracer::runOnOff99(){
    /*if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }*/
    bool isOnLine = mLineMonitor->isOnLine();

    if(isOnLine == true){
        mLeftWheel.setPWM(13);
        mRightWheel.setPWM(-6);
    }
    else{
        mLeftWheel.setPWM(-6);
        mRightWheel.setPWM(13);
    }
}
/* バランス崩す状態の走行 */
int LineTracer::runOnOff3(){
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    int isOnLine = mLineMonitor->isOnLine3();//ライン上判定（カラーセンサ）
	//走行を行う
	if(isOnLine == 3 ){//黒色見つける
		mLeftWheel.setPWM(0);
		mRightWheel.setPWM(0);
        return 3;
	}
	if(isOnLine == 8 || isOnLine == 2){//灰色見つける
		mLeftWheel.setPWM(5);
		mRightWheel.setPWM(10);
		return 1;
	}
	if(isOnLine == 1  ){
		mLeftWheel.setPWM(30);
		mRightWheel.setPWM(5);
	}


	return 0;
}
void LineTracer::runOnOff4(){
    /*if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }*/
    bool isOnLine = mLineMonitor->isOnLine();

    if(isOnLine == true){
        mLeftWheel.setPWM(15);
        mRightWheel.setPWM(-5);
    }
    else{
        mLeftWheel.setPWM(-5);
        mRightWheel.setPWM(15);
    }
}
/* バランス崩す状態ライン探す */
void LineTracer::runFind(){
    static int find = 0;
    static int time1 = 0;
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }
    bool isOnLine = mLineMonitor ->isOnLine();//ライン上判定（ジャイロセンサ）
	//回転を行う
    time1++;
    if(isOnLine == false){
            mLeftWheel.setPWM(0);
		    mRightWheel.setPWM(0);
    }
    if(find == 0 && isOnLine == true){
        mLeftWheel.setPWM(25);
	    mRightWheel.setPWM(-25);
    }
    if(find == 1 && isOnLine == true){
        mLeftWheel.setPWM(-25);
	    mRightWheel.setPWM(25);
    }
    if(time1 > 100 && isOnLine == true)find = 1;
    if(time1 > 300 && isOnLine == true){
        find = 0;
        time1 = 0;
    }
	
}
/* バランス崩す状態の回転 */
void LineTracer::runBack1(){
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }
    bool isOnLine = mLineMonitor ->isOnLine99();//ライン上判定（ジャイロセンサ）
	//回転を行う
    if(isOnLine == true){
		mLeftWheel.setPWM(-25);
		mRightWheel.setPWM(25);
    }else{
        mLeftWheel.setPWM(0);
		mRightWheel.setPWM(0);
    }
	
}
/* バランス崩す状態の回転 */
void LineTracer::runBack2(){
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }
    bool isOnLine = mLineMonitor ->isOnLine99();//ライン上判定（ジャイロセンサ）
	//回転を行う
    if(isOnLine == true){
		mLeftWheel.setPWM(0);
		mRightWheel.setPWM(0);
    }else{
        mLeftWheel.setPWM(-25);
		mRightWheel.setPWM(25);
    }
	
}
// バランス崩し
void LineTracer::runAnBalance(){
	if (mIsInitialized == false) {
		mBalancingWalker->init();
		mIsInitialized = true;
	}

    // 走行体の向きを計算する
	mPidController->setPidFactor( 0.0, 0.0, 0.0);
	int direction = calcDirection();

    mBalancingWalker->setCommand(0, direction);

    // 倒立走行を行う
    mBalancingWalker->runAnBalance();
}

//すとっぷ
void LineTracer::runStop(){
    if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }

    // 走行体の向きを計算する
	mPidController->setPidFactor( 1.00, 0.0, 0.0);
    //int direction = calcDirection();

    mBalancingWalker->setCommand(0, 0);

    // 倒立走行を行う
    mBalancingWalker->run();
}

//ターン
void LineTracer::runTurn(){
//    if (mIsInitialized == false) {
//          mBalancingWalker->init();
//          mIsInitialized = true;
//    }

//    // 走行体の向きを計算する
//	mPidController->setPidFactor( 0.001, 0.0, 0.0);
//    int direction = calcDirection();

    mBalancingWalker->setCommand(0, 20);

    // 倒立走行を行う
    mBalancingWalker->run();
}

/**
 * 走行体の向きを計算する
 * @param isOnLine true:ライン上/false:ライン外
 * @retval 30  ライン上にある場合(右旋回指示)
 * @retval -30 ライン外にある場合(左旋回指示)
 */
int LineTracer::calcDirection() {

	return mPidController->calControlledVariable(mLineMonitor->getDeviation());

}

void LineTracer::edgeChage(int edge) {
	mPidController->setEdge(edge);
}

int LineTracer::getAngle(){
	return mBalancingWalker->jyro();
}
void LineTracer::Init(){
    mIsInitialized = false;
}