/******************************************************************************
 *  LineMonitor.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class LineMonitor
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_UNIT_LINEMONITOR_H_
#define EV3_UNIT_LINEMONITOR_H_

#include "ColorSensor.h"

// 定義
class LineMonitor {
public:
    explicit LineMonitor(const ev3api::ColorSensor& colorSensor);
    virtual ~LineMonitor();

    bool isOnLine() const;
    bool isOnLine2() const;
	int isOnLine3() const;
	void setGray( rgb_raw_t rgb );
	bool isOnLine99() const;
    void setLineThreshold(int8_t threshold);
    int8_t getDeviation() const;
    void calLineThreshold();
    void setBlackThreshold(int8_t threshold);
    void setWhiteThreshold(int8_t threshold);


private:
    static const int8_t INITIAL_THRESHOLD;

    const ev3api::ColorSensor& mColorSensor;
    int8_t mLineThreshold;
    int8_t mWhiteThresh;
    int8_t mBlackThresh;
	
	rgb_raw_t	mGrayThresh;
};

#endif  // EV3_UNIT_LINEMONITOR_H_
