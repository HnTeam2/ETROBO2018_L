/******************************************************************************
 *  LineTracerWithStarter.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/26
 *  Implementation of the Class LineTracerWithStarter
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_APP_LINETRACERWITHSTARTER_H_
#define EV3_APP_LINETRACERWITHSTARTER_H_

#include "ev3api.h"
#include "Starter.h"
#include "LineTracer.h"
#include "Calibration.h"
#include "SonarSensor.h"

class LineTracerWithStarter {
public:
	
    LineTracerWithStarter(LineTracer* lineTracer,
				const Starter* starter,
				Calibration* calibration,
				LineMonitor* lineMonitor,
				ev3api::Motor& tailWheel,
				ev3api::SonarSensor& sonarSensor,
                ev3api::Motor& leftWheel,
                ev3api::Motor& rightWheel,
				char* bt_data
    );
	
    virtual ~LineTracerWithStarter();

    void run();

private:
    enum State {
        UNDEFINED,
        CALIBRATION_GYRO,
        CALIBRATION_BLACK,
        CALIBRATION_WHITE,
    	CALIBRATION_GRAY,
        WAITING_FOR_START,
        WALKING,
    	FIGURE,
    	LAST
    };

	LineTracer* mLineTracer;
	const Starter* mStarter;
	Calibration* mCalibration;
	LineMonitor* mLineMonitor;
	ev3api::Motor mTailWheel;
	ev3api::SonarSensor mSonarSensor;
    ev3api::Motor mLeftWheel;
    ev3api::Motor mRightWheel;
    State mState;
	char* mBt_data;

    void execUndefined();
    void execCalibrationGyro();
    void execCalibrationBlack();
    void execCalibrationWhite();
	void execCalibrationGray();
    void execWaitingForStart();
    void execWalking();
    void execFigure();
	void execLast();
	void tailControll(int deg);
	void tailControll2(int deg);
};

#endif  // EV3_APP_LINETRACERWITHSTARTER_H_
