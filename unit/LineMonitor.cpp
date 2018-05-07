/******************************************************************************
 *  LineMonitor.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class LineMonitor
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/


#include "LineMonitor.h"

// 定数宣言
const int8_t LineMonitor::INITIAL_THRESHOLD = 15;  // 黒色の光センサ値


/**
 * コンストラクタ
 * @param colorSensor カラーセンサ
 */
LineMonitor::LineMonitor(const ev3api::ColorSensor& colorSensor)
    : mColorSensor(colorSensor),
      mLineThreshold(INITIAL_THRESHOLD),
      mWhiteThresh(INITIAL_THRESHOLD),
      mBlackThresh(INITIAL_THRESHOLD) {
}

/**
 * デストラクタ
 */
LineMonitor::~LineMonitor() {
}

/**
 * ライン上か否かを判定する
 * @retval true  ライン上
 * @retval false ライン外
 */
bool LineMonitor::isOnLine() const {
    // 光センサからの取得値を見て
    // 黒以上であれば「true」を、
    // そうでなければ「false」を返す
    if (mColorSensor.getBrightness() >= (mLineThreshold / 4.4)) {
        return true;
    } else {
        return false;
    }
}

bool LineMonitor::isOnLine2() const {
    // 光センサからの取得値を見て
    // 黒以上であれば「true」を、
    // そうでなければ「false」を返す

    if (mColorSensor.getBrightness() >= (mLineThreshold / 2)) {
        return true;
    } else {
        return false;
    }
}
// 灰色の設定
void LineMonitor::setGray( rgb_raw_t rgb ) {
	mGrayThresh = rgb;
}

// 色の判定
int LineMonitor::isOnLine3() const {
    // 光センサからの取得値を見て
    // 黒以上であれば「true」を、
    // そうでなければ「false」を返す

	static int count = 0;
	static float rcolor = 55;//mGrayThresh.r*bt+30 +- 5
	static float gcolor = 70;//mGrayThresh.g*bt+30 +- 5
	static float bcolor = 52;//mGrayThresh.b*bt+30 +- 5
	rgb_raw_t	rgb;
	mColorSensor.getRawColor( rgb );

	int8_t	cr = ( rgb.r + rgb.g + rgb.b ) / 3;
	
	// 灰	( B10,10,10; W120,110,140; G60,60,80; )
	if( rgb.r > rcolor - 5 && rgb.r < rcolor + 5 &&
		rgb.g > gcolor - 5 && rgb.g < gcolor + 5 &&
		rgb.b > bcolor - 5 && rgb.b < bcolor + 5 ){
			ev3_speaker_play_tone( 440, 100 );
			count ++;
			if( count > 40 ) return 2;
	}
	// 黒
	else if( cr < 40 ) {
		count = 0;
        return 3;
    }
    // 白
    else if ( cr >= 40 ) {
        count = 0;
        if(cr>=60)return 1;
        else if(cr < 60)return 8;
    }
	
	return 0;
}
bool LineMonitor::isOnLine99() const {
    // 光センサからの取得値を見て
    // 黒以上であれば「true」を、
    // そうでなければ「false」を返す

    if (mColorSensor.getBrightness() >= (mLineThreshold / 2.8) ){
        return true;
    } else {
        return false;
    }
}
/**
 * ライン閾値を設定する。
 * @param threshold ライン閾値
 */
void LineMonitor::setLineThreshold(int8_t threshold) {
    mLineThreshold = threshold;
}


/**
 * ライン閾値からの偏差を取得する
 */
int8_t LineMonitor::getDeviation() const {
    return (mLineThreshold - mColorSensor.getBrightness());
}

/**
 * ライン閾値を算出して設定する
 */
void LineMonitor::calLineThreshold() {
    int16_t  cal;

    cal = (mBlackThresh + mWhiteThresh) / 3 * 2;
    setLineThreshold(cal);
}

/**
 * 黒色の閾値を設定する
 */
void LineMonitor::setBlackThreshold(int8_t threshold) {
    mBlackThresh = threshold;
}


/**
 * 白色の閾値を設定する
 */
void LineMonitor::setWhiteThreshold(int8_t threshold) {
    mWhiteThresh = threshold;
}