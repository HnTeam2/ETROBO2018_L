/******************************************************************************
 *  Calibration.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/06/12
 *  Implementation of the Class BalancingWalker
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_UNIT_CALIBLATION_H_
#define EV3_UNIT_CALIBLATION_H_

#include "GyroSensor.h"
#include "ColorSensor.h"
#include "LineMonitor.h"


class Calibration
{
public:

    Calibration(const ev3api::ColorSensor& colorSensor,
                  ev3api::GyroSensor& gyroSensor,
                  LineMonitor* lineMonitor);
    virtual ~Calibration();

    void init();
    bool calibrateGyro(bool startTrigger);
    bool calibrateBlack(bool startTrigger);
    bool calibrateWhite(bool startTrigger);
	bool calibrationColor( bool startTrigger );
    void calibrateLineThreshold();

private:
    static const int NUM_OF_GYRO_CALIBRATION;
    static const int NUM_OF_BLACK_CALIBRATION;
    static const int NUM_OF_WHITE_CALIBRATION;

    const ev3api::ColorSensor& mColorSensor;
    ev3api::GyroSensor& mGyroSensor;
    LineMonitor* mLineMonitor;
    bool mIsStartedGyro;
    bool mIsStartedBlack;
    bool mIsStartedWhite;
    int16_t  mCalCount;
    int32_t  mSum;

};

#endif  // EV3_UNIT_CALIBLATION_H_
