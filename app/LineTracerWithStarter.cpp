/******************************************************************************
 *  LineTracerWithStarter.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/26
 *  Implementation of the Class LineTracerWithStarter
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "LineTracerWithStarter.h"
#include "ev3api.h"
#include "SonarSensor.h"
#include "ConstParam.h"

/**
 * コンストラクタ
 * @param lineTracer ライントレーサ
 * @param starter    スタータ
 */
LineTracerWithStarter::LineTracerWithStarter(LineTracer* lineTracer,
											const Starter* starter,
											Calibration* calibration,
											LineMonitor* lineMonitor,
											ev3api::Motor& tailWheel,
											ev3api::SonarSensor& sonarSensor,
											ev3api::Motor& leftWheel,
											ev3api::Motor& rightWheel,
											char* bt_data)
	: mLineTracer(lineTracer),
	  mStarter(starter),
	  mCalibration(calibration),
	  mLineMonitor(lineMonitor),
	  mTailWheel(tailWheel),
	  mSonarSensor(sonarSensor),
	  mLeftWheel(leftWheel),
	  mRightWheel(rightWheel),
	  mState(UNDEFINED){
		mBt_data = bt_data;
}

/**
 * デストラクタ
 */
LineTracerWithStarter::~LineTracerWithStarter() {
}

/**
 * ライントレースする
 */
void LineTracerWithStarter::run() {
    switch (mState) {
    case UNDEFINED:
        execUndefined();
        break;
    case CALIBRATION_GYRO:
        execCalibrationGyro();
        break;
    case CALIBRATION_BLACK:
        execCalibrationBlack();
        break;
    case CALIBRATION_WHITE:
        execCalibrationWhite();
        break;
    case CALIBRATION_GRAY:
        execCalibrationGray();
        break;
    case WAITING_FOR_START:
        execWaitingForStart();
        break;
    case WALKING:
        execWalking();
        break;

	default:
        break;
    }
}

/**
 * 未定義状態の処理
 */
void LineTracerWithStarter::execUndefined() {
	mTailWheel.reset();
	mTailWheel.setCount(0);
	mortorControll(mTailWheel,85,50); //テイル降ろす
	mCalibration->init();
	mState = CALIBRATION_GYRO;
}

/**
 * ジャイロセンサのキャリブレーション
 */
void LineTracerWithStarter::execCalibrationGyro() {
//	if (mCalibration->calibrateGyro(mStarter->isPushed()) == true) {
	if (mCalibration->calibrateGyro(1) == true) {
		mState = CALIBRATION_BLACK;
	}
}

/**
 * 黒キャリブレーション
 */
void LineTracerWithStarter::execCalibrationBlack() {
//	if (mCalibration->calibrateBlack(mStarter->isPushed()) == true) {
	if (mCalibration->calibrateBlack(1) == true) {
		mState = CALIBRATION_WHITE;
	}
}

/**
 * 白キャリブレーション
 */
void LineTracerWithStarter::execCalibrationWhite() {
	if (mCalibration->calibrateWhite(mStarter->isPushed()) == true) {
		mState = CALIBRATION_GRAY;
	}
}
/**
 * 灰キャリブレーション
 */
void LineTracerWithStarter::execCalibrationGray() {
	if (mCalibration->calibrationColor(mStarter->isPushed()) == true) {
		while(mStarter->isPushed());
		mState = WAITING_FOR_START;
	}
}

/* スタートダッシュ */
void LineTracerWithStarter::execWaitingForStart() {
	if (mStarter->isPushed() || *mBt_data == '1') {//ボタン押すＯＲブルーツースから１送るとスタート
		mCalibration->calibrateLineThreshold();
		mortorControll(mTailWheel,100,50);
		// tailモータの押し出す力によって、左右ホイールが3°動いたら次のStateに移る
		mTailWheel.setPWM(-50);
		mState = WALKING;
	}
}

/*  走行中  */
void LineTracerWithStarter::execWalking() {
	static int count = 0;
	switch(count) {
		//走行を開始する
		// 第一直線
		case 0:
			mLineTracer->taskNormal(ConstParam::PID_1,ConstParam::SPEED_HIGH);
			if (mLeftWheel.getCount() > 2000) { count = 1; }
			break;

		case 1:
			mLineTracer->taskNormal(ConstParam::PID_2,ConstParam::SPEED_MIDDLE);
			break;
	}
}

// degは目的の角度。PWMはモータの速さで、正の値で正回転、負の値で逆回転。
void LineTracerWithStarter::mortorControll(ev3api::Motor& motor, int deg, int pwm) {
	motor.setPWM(pwm);
	if (deg > 0){
		while(motor.getCount() < deg);
	}else{
		while(motor.getCount() > deg);
	}
	motor.setPWM(0);
}
