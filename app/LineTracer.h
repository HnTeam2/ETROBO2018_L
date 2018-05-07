/******************************************************************************
 *  LineTracer.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class LineTracer
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_APP_LINETRACER_H_
#define EV3_APP_LINETRACER_H_

#include "LineMonitor.h"
#include "BalancingWalker.h"
#include "PidController.h"

class LineTracer {
public:
    LineTracer(const LineMonitor* lineMonitor,
               BalancingWalker* balancingWalker,
               PidController* pidController,
			   ev3api::Motor& leftWheel,
			   ev3api::Motor& rightWheel);
    virtual ~LineTracer();

    void run();
	void run2();
	void run3();
	void runOnOff();
    void runOnOff4();
	void runOnOff99();
	int runOnOff2();
    int runOnOff3();
    void runFind();
    void runBack1();
    void runBack2();
	void run90();
    void runLOW();
    void run80();
	void runAnBalance();
    void runStop();
    void runTurn();
	void edgeChage(int edge); 
	int getAngle();
	void Init();

private:
    const LineMonitor* mLineMonitor;
    BalancingWalker* mBalancingWalker;
    PidController* mPidController;
	ev3api::Motor mLeftWheel;
	ev3api::Motor mRightWheel;
    bool mIsInitialized;

	int calcDirection();
};

#endif  // EV3_APP_LINETRACER_H_
