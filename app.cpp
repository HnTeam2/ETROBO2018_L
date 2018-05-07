/******************************************************************************
 *  app.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Task main_task
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "app.h"
#include "LineTracerWithStarter.h"
#include "SonarSensor.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// using宣言
using ev3api::SonarSensor;
using ev3api::ColorSensor;
using ev3api::GyroSensor;
using ev3api::TouchSensor;
using ev3api::Motor;

// Device objects
// オブジェクトを静的に確保する
SonarSensor gSonarSensor(PORT_2);
ColorSensor gColorSensor(PORT_3);
GyroSensor  gGyroSensor(PORT_4);
TouchSensor gTouchSensor(PORT_1);
Motor       gLeftWheel(PORT_C);
Motor       gRightWheel(PORT_B);
Motor       gTailWheel(PORT_A);

// オブジェクトの定義
static LineMonitor     *gLineMonitor;
static Balancer        *gBalancer;
static BalancingWalker *gBalancingWalker;
static LineTracer      *gLineTracer;
static Starter         *gStarter;
static Calibration     *gCalibration;
static PidController   *gPidController;
static LineTracerWithStarter *gLineTracerWithStarter;

char gBt_data = 0;

/**
 * EV3システム生成
 */
static void user_system_create() {
	tslp_tsk(2);
    // オブジェクトの作成
    gBalancer        = new Balancer();
    gBalancingWalker = new BalancingWalker(gGyroSensor,
                                           gLeftWheel,
                                           gRightWheel,
                                           gBalancer);
    gLineMonitor     = new LineMonitor(gColorSensor);
    gStarter         = new Starter(gTouchSensor);
    gPidController   = new PidController();
    gLineTracer      = new LineTracer(gLineMonitor, 
                                      gBalancingWalker, 
                                      gPidController, 
                                      gLeftWheel, 
                                      gRightWheel);
    gCalibration     = new Calibration(gColorSensor, 
                                       gGyroSensor, 
                                       gLineMonitor);
    gLineTracerWithStarter = new LineTracerWithStarter(gLineTracer, 
                                                       gStarter, 
                                                       gCalibration, 
                                                       gLineMonitor, 
                                                       gTailWheel, 
                                                       gSonarSensor, 
                                                       gLeftWheel, 
                                                       gRightWheel,
                                                       &gBt_data);

    // 初期化完了通知
    ev3_led_set_color(LED_ORANGE);
}

/**
 * EV3システム破棄
 */
static void user_system_destroy() {
    gLeftWheel.reset();
    gRightWheel.reset();
    gTailWheel.reset();

    delete gLineTracerWithStarter;
    delete gLineTracer;
    delete gStarter;
    delete gLineMonitor;
    delete gBalancingWalker;
    delete gBalancer;
    delete gCalibration;
    delete gPidController;
}

/**
 * トレース実行タイミング
 */
void ev3_cyc_tracer(intptr_t exinf) {
    act_tsk(TRACER_TASK);
}

/**
 * メインタスク
 */
void main_task(intptr_t unused) {
    user_system_create();  // センサやモータの初期化処理

    // 周期ハンドラ開始
    ev3_sta_cyc(EV3_CYC_TRACER);
    act_tsk(BT_TASK);

    slp_tsk();  // バックボタンが押されるまで待つ

    // 周期ハンドラ停止
    ev3_stp_cyc(EV3_CYC_TRACER);

    user_system_destroy();  // 終了処理

    ext_tsk();
}

/**
 * ライントレースタスク
 */
void tracer_task(intptr_t exinf) {
    if (ev3_button_is_pressed(BACK_BUTTON)) {
        wup_tsk(MAIN_TASK);  // バックボタン押下
    } else {
        gLineTracerWithStarter->run();  // 倒立走行
    }
    ext_tsk();
}

/**
 * BlueToothタスク
 */
void bt_task(intptr_t unused){
	int btwheel=0;
	FILE *bt = ev3_serial_open_file(EV3_SERIAL_BT);
    while(1) {
        gBt_data = fgetc(bt); /* 受信 */
		fprintf(bt, "%c\r",gBt_data);
    	if(gBt_data == '1'){
    		btwheel=gLeftWheel.getCount();
    		fprintf(bt, "%c  Lwheel=%d\r\n",gBt_data,btwheel);
    		tslp_tsk(100);
    	}
    }
}
