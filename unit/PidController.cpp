/******************************************************************************
 *  PidController.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/06/12
 *  Implementation of the Class BalancingWalker
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "PidController.h"

/**
 * コンストラクタ
 */
PidController::PidController()
    : mDeviation(0),
      mIntegral(0) {
    KPID_P_FACTOR   =   (float)( 1.00);
    KPID_I_FACTOR   =   (float)( 0.00);
    KPID_D_FACTOR   =   (float)( 0.00);
}

/**
 * デストラクタ
 */
PidController::~PidController() {
}

/**
 * PID制御でのTurn値を算出する
 * @param deviation  ラインの閾値との差分（偏差）
 */
float PidController::calControlledVariable(int8_t deviation) {
#define DELTA_T             (float)(0.004)      // 4ms周期でのトレース

#define KPID_EDGE_FACTOR    (int32_t)(-1)        // ライントレース方向 1 or -1

#define KPID_TURN_LIMIT     (int32_t)(100)      // 旋回指示値 限界値

    float brightness_P;                       // P成分
    float brightness_I;                       // I成分
    float brightness_D;                       // D成分
    float turn_P;                             // P項演算値
    float turn_I;                             // I項演算値
    float turn_D;                             // D項演算値
    float turn;                               // 旋回命令: -100 (左旋回) ～ 100 (右旋回)

/*
    brightness_P = (int32_t)deviation;
    brightness_I = mIntegral + brightness_P;
    brightness_D = brightness_P - mDeviation;

    mDeviation = brightness_P;                              // 次回演算用に記憶
    mIntegral = brightness_I;*/

    //D
    brightness_D = ( deviation - mDeviation) / DELTA_T;
    
    //I
    mIntegral += (mDeviation + deviation) / 2.0 * DELTA_T;
    brightness_I = mIntegral;
    
    //P
    brightness_P = deviation;
    mDeviation = brightness_P;
    

    /* P項演算 */
    turn_P = (KPID_P_FACTOR * (float)brightness_P);
    turn_P *= KPID_EDGE_FACTOR;

    /* I項演算 */
    turn_I = (KPID_I_FACTOR * (float)brightness_I);
    turn_I *= KPID_EDGE_FACTOR;

    /* D項演算 */
    turn_D = (KPID_D_FACTOR * (float)brightness_D);
    turn_D *= KPID_EDGE_FACTOR;

    /* 旋回指示値設定   */
    turn = turn_P + turn_I + turn_D;

    if (turn >= KPID_TURN_LIMIT) {
        turn = KPID_TURN_LIMIT;
    } else if (turn <= (-1 * KPID_TURN_LIMIT)) {
        turn = -1 * KPID_TURN_LIMIT;
    }

    return turn;
}

void PidController::setPidFactor(float kp, float ki, float kd){
    KPID_P_FACTOR = kp;
    KPID_I_FACTOR = ki;
    KPID_D_FACTOR = kd;
}

void PidController::setEdge(int edge) {
    mEdge = edge;
}