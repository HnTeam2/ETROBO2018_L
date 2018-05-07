/******************************************************************************
 *  BalancingWalker.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class BalancingWalker
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "BalancingWalker.h"

// 定数宣言
const int BalancingWalker::LOW    = 20;    // 低速
const int BalancingWalker::NORMAL = 50;    // 通常
const int BalancingWalker::HIGH   = 70;    // 高速

/**
 * コンストラクタ
 * @param gyroSensor ジャイロセンサ
 * @param leftWheel  左モータ
 * @param rightWheel 右モータ
 * @param balancer   バランサ
 */
BalancingWalker::BalancingWalker(const ev3api::GyroSensor& gyroSensor,
                                 ev3api::Motor& leftWheel,
                                 ev3api::Motor& rightWheel,
                                 Balancer* balancer)
    : mGyroSensor(gyroSensor),
      mLeftWheel(leftWheel),
      mRightWheel(rightWheel),
      mBalancer(balancer),
      mForward(LOW),
      mTurn(LOW),
      mTimerFallDown(0){
}

/**
 * デストラクタ
 */
BalancingWalker::~BalancingWalker() {
}

/**
 * バランス走行する
 */
void BalancingWalker::run() {
    int16_t angle = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    int rightWheelEnc = mRightWheel.getCount();       // 右モータ回転角度
    int leftWheelEnc  = mLeftWheel.getCount();        // 左モータ回転角度

    mBalancer->setCommand(mForward, mTurn);

    int battery = ev3_battery_voltage_mV();
    mBalancer->update(angle, rightWheelEnc, leftWheelEnc, battery);

    // 左右モータに回転を指示する
    if (detectFall() == false) {
        mLeftWheel.setPWM(mBalancer->getPwmLeft());
        mRightWheel.setPWM(mBalancer->getPwmRight());
    }
    else {
        mLeftWheel.setPWM(0);
        mRightWheel.setPWM(0);
    }
}

/**
 * バランス崩し
 */
void BalancingWalker::runAnBalance() {
    int16_t angle = mGyroSensor.getAnglerVelocity() - 30 ;  // ジャイロセンサ値をangleに代入
    int rightWheelEnc = mRightWheel.getCount();       // 右モータ回転角度を代入
    int leftWheelEnc  = mLeftWheel.getCount();        // 左モータ回転角度を代入

    mBalancer->setCommand(mForward, mTurn);

    int battery = ev3_battery_voltage_mV();
    mBalancer->update(angle, rightWheelEnc, leftWheelEnc, battery);

    // 左右モータに回転を指示する
    if (detectFall() == false) {
        mLeftWheel.setPWM(mBalancer->getPwmLeft());
        mRightWheel.setPWM(mBalancer->getPwmRight());
    }
    else {
        mLeftWheel.setPWM(0);
        mRightWheel.setPWM(0);
    }
}

/**
 * バランス走行に必要なものをリセットする
 */
void BalancingWalker::init() {
    int offset = 0;

    // モータエンコーダをリセットする
    mLeftWheel.reset();
    mRightWheel.reset();

    // 倒立振子制御初期化
    mBalancer->init(offset);
}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void BalancingWalker::setCommand(int forward, float turn) {
    mForward = forward;
    mTurn    = turn;
}

int BalancingWalker::getAngle() {
    int16_t angle = mGyroSensor.getAnglerVelocity();

    if(angle <= -20){
        return false;
    }else{
        return true;
    }
}

/**
 * 走行体のフェール状態として、転倒したかどうかを判断する
 * ★フェールセーフ演習修正箇所
 */
bool BalancingWalker::detectFall()
{
#define KtimeFallDown   (250)

    if (   ((mBalancer->getPwmLeft() >= 100)
         && (mBalancer->getPwmRight() >= 100))
        || ((mBalancer->getPwmLeft() <= -100)
         && (mBalancer->getPwmRight() <= -100))  ) {

        mTimerFallDown++;
    }
    else {
        if(mTimerFallDown <= KtimeFallDown){
            mTimerFallDown = 0;
        }
    }

    if (mTimerFallDown > KtimeFallDown) {
        return true;
    }
    else {
        return false;
    }
}
int  BalancingWalker::jyro(){
	return mGyroSensor.getAnglerVelocity();
}

