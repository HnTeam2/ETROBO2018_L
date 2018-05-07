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
    case FIGURE:
        execFigure();
        break;
    case LAST:
        execLast();
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
	tailControll(95); //テイル降ろす
	//tailControll2(80);
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
	if( mCalibration->calibrationColor(mStarter->isPushed()) == true ){
		while(mStarter->isPushed());
		mState = WAITING_FOR_START;
	}
}
/**
 * 開始待ち状態の処理
 */
void LineTracerWithStarter::execWaitingForStart() {
	if (mStarter->isPushed() || *mBt_data == '1') {//ボタン押すＯＲブルーツースから１送るとスタート
		mCalibration->calibrateLineThreshold();
		mState = WALKING;
	}
}

/**
 * 走行中状態の処理
 */
void LineTracerWithStarter::execWalking() {
	static int count = 0;
	if(mTailWheel.getCount() < 1) mTailWheel.setPWM(0);
	switch(count){
		case 0:									// テイル回収
			mLineTracer->runLOW();
			if(mLeftWheel.getCount() > 3){
				mTailWheel.setPWM(-50);
				count++;
			}
			break;
		case 1:									// テイル回収したあと走行する
			mLineTracer->runLOW();
			if(mTailWheel.getCount() == 0) mTailWheel.setPWM(0);
			if(mLeftWheel.getCount() > 50){
				count=2;
				//mState = FIGURE;
			}
			break;
		case 2:									// 第一直線
			mLineTracer->run();
			//if(mLeftWheel.getCount() > 2800){
				//count=3;
			//}
			break;
		case 3:									// 第1カーブ
			mLineTracer->run();
			if(mLeftWheel.getCount() > 7000){
				count++;
			}
			break;
		case 4:									// 第2カーブ
			mLineTracer->run();
			if(mLeftWheel.getCount() > 12200){
				count++;
			}
			break;
		case 5:									// 第3カーブ
			mLineTracer->run();
			if(mLeftWheel.getCount() > 12200){
				count++;
			}
			break;
		case 6:									// 第2直線
			mLineTracer->run90();
			if(mLeftWheel.getCount() > 14600){
				count++;
			}
			break;
		case 7:									// フィギュア前まで
			mLineTracer->run3();
			if(mLeftWheel.getCount() > 16000){  //10000
				mState = FIGURE;
				//mLeftWheel.setCount(0);
			}
			break;
	}
}

/**
 * 難所１の処理
 */

void LineTracerWithStarter::execFigure() {
	static int32_t countF = 0;
	static int32_t time = 0;
	static int32_t x=73;
	switch(countF){
		case 0:
			time++;
			if( time > 225 ) {
				countF++;							//初期化
				time = 0;
				mLeftWheel.setCount(0);
				mRightWheel.setCount(0);
			}
			mLineTracer->runStop();					// その場で止まる
			break;
		case 1:
		if( mTailWheel.getCount() >= 80 ) {			//尻尾角度確認
			mTailWheel.setPWM(0);
			if(mLineTracer->getAngle() <= -30){
				mLeftWheel.setCount(0);
				while(time<1000)time++;
				time=0;
				countF++;
			}
		}
			mLineTracer->runStop();
			mTailWheel.setPWM(10);
			break;
		case 2:
			mLeftWheel.setPWM(10);					//前進、尻尾再確認
			mRightWheel.setPWM(10);
			time++;
			if(time > 50) {
				time = 0;
				while(mTailWheel.getCount() >= 80)mTailWheel.setPWM(-2);
				mTailWheel.setPWM(0);
				mLeftWheel.setCount(0);
				countF++;    
			}
			break;
        case 3:
          	mLineTracer->runOnOff4();					//道（黒い線）を探す
			time++;
            if(time > 5000){
                countF=150;
            	time=0;
            }
            break;
		 case 150:
			time++;
			mLeftWheel.setPWM(35);						//左タイヤ登る
            mRightWheel.setPWM(0);
			 if(time > 150){
                countF++;
            	time=0;
            }
			break;
		 case 151:
			time++;
			mLeftWheel.setPWM(0);						//右タイヤ登る
            mRightWheel.setPWM(35);
			 if(time > 280){
                countF=4;
            	time=0;
            }
			break;
		 case 4:
          	mLeftWheel.setPWM(10);						//登る速度（調整速度）
			mRightWheel.setPWM(10);
			time++;
            if(time > 1500){
                countF++;
            	time=0;
            }
            break;
		  case 5:										//尻尾登る、case ５から８
            time++;
            mLeftWheel.setPWM(-5);
            mRightWheel.setPWM(-5);
            mTailWheel.setPWM(10);
            if(time >= 250 || mTailWheel.getCount() > 97){
                countF ++;
            }
            break;
        case 6:
            if(mTailWheel.getCount() > 97){
                mLeftWheel.setPWM(10);
                mRightWheel.setPWM(10);
                countF ++;
            }
            break;
        case 7:
            time++;
            mRightWheel.setPWM(15);
            mLeftWheel.setPWM(15);
            mTailWheel.setPWM(-4);
            if(time > 125 || mTailWheel.getCount() < 85){
                countF ++;
                time = 0;
            }
            break;
        case 8:
            mTailWheel.setPWM(-4);
            if(mTailWheel.getCount() < 85){
                //mLineTracer->runOnOffStop();
                mTailWheel.setPWM(0);
                countF ++;
                mLeftWheel.setCount(0);
            }
            break;
        case 9:									//真ん中にする
            mLeftWheel.setPWM(10);
            mRightWheel.setPWM(10);
            if(mLeftWheel.getCount() > 125){
                countF ++;
            }
            break;
		case 10:								//止まるand尻尾調整
            mLeftWheel.setPWM(0);
            mRightWheel.setPWM(0);
            mTailWheel.setPWM(1);
            if(mTailWheel.getCount() >= 85){
                countF ++;
            	mTailWheel.setPWM(0);
            	time=0;
            }
            break;
        case 11:								//回転の準備
            time++;
            mLeftWheel.setPWM(0);
            mRightWheel.setPWM(0);
            if(time > 250){
                countF ++;
                time = 0;
                mLeftWheel.setCount(0);
            }
            break;
		case 12:								//1回転目、case 12,case 122
            mLeftWheel.setPWM(10);
            mRightWheel.setPWM(-10);
            if(mLeftWheel.getCount() > 900){
                countF =122;
                mLeftWheel.setCount(0);
            	mRightWheel.setCount(0);
            }
            break;
		case 122:
            mLeftWheel.setPWM(-10);
            mRightWheel.setPWM(10);
            if(mRightWheel.getCount() > 220){
                countF =13;
                mLeftWheel.setCount(0);
            }
            break;
        //sen sagasu
        case 13:							//case13から16、回転後の調整
            time++;
            //mLineTracer->runFind();
            mLeftWheel.setPWM(5);
            mRightWheel.setPWM(5);
            if(time > 255){
                countF ++;
                time = 0;
                mLeftWheel.setPWM(0);
                mRightWheel.setPWM(0);
            }
            break;
        case 14:
            time++;
            mLeftWheel.setPWM(0);
            mRightWheel.setPWM(0);
            if(time > 250){
                countF ++;
                time = 0;
                mLineTracer->Init();
            }
            break;
        case 15:
            //mLineTracer->Init();
            time++;
            mLineTracer->runStop();
            if(time > 250){
                countF ++;
                time = 0;
            }
            break;
        case 16:
            //mLineTracer->Init();
            time++;
            mLeftWheel.setPWM(-5);
            mRightWheel.setPWM(-5);
            if(time > 0){
                countF ++;
                time = 0;
            }
            break;
		case 17:							//道（黒い線）を探す
          	mLineTracer->runOnOff4();
			time++;
            if(time > 2500){
                countF=152;
            	time=0;
            }
            break;
		 case 152:							//左タイヤ登る
			time++;
			mLeftWheel.setPWM(35);
            mRightWheel.setPWM(0);
			 if(time > 150){
                countF++;
            	time=0;
            }
			break;
		 case 153:							//右タイヤ登る
			time++;
			mLeftWheel.setPWM(0);
            mRightWheel.setPWM(35);
			 if(time > 280){
                countF=18;
            	time=0;
            }
			break;
		 case 18:
          	mLeftWheel.setPWM(15);					//登る速度（調整速度）
			mRightWheel.setPWM(15);
			time++;
            if(time > 1500){
                countF++;
            	time=0;
            }
            break;
		  case 19:									//尻尾登る、case １９から２２
            time++;
            mLeftWheel.setPWM(-5);
            mRightWheel.setPWM(-5);
            mTailWheel.setPWM(8);
            if(time >= 250 || mTailWheel.getCount() > 97){
                countF ++;
            }
            break;
        case 20:
            if(mTailWheel.getCount() > 97){
                mLeftWheel.setPWM(10);
                mRightWheel.setPWM(10);
                countF ++;
            }
            break;
        case 21:
            time++;
            mRightWheel.setPWM(10);
            mLeftWheel.setPWM(10);
            mTailWheel.setPWM(-4);
            if(time > 125 || mTailWheel.getCount() < 85){
                countF ++;
                time = 0;
            }
            break;
        case 22:
            mTailWheel.setPWM(-4);
            if(mTailWheel.getCount() < 85){
                //mLineTracer->runOnOffStop();
                mTailWheel.setPWM(0);
                countF ++;
                mLeftWheel.setCount(0);
            }
            break;
        case 23:								//真ん中にする
            mLeftWheel.setPWM(10);
            mRightWheel.setPWM(10);
            if(mLeftWheel.getCount() > 125){
                countF ++;
            }
            break;
		case 24:								//止まるand尻尾調整
            mLeftWheel.setPWM(0);
            mRightWheel.setPWM(0);
            mTailWheel.setPWM(1);
            if(mTailWheel.getCount() >= 87){
                countF =244;
            	mTailWheel.setPWM(0);
            	time=0;
            }
            break;
		case 244:								//回転の準備
            time++;
            mLeftWheel.setPWM(0);
            mRightWheel.setPWM(0);
            if(time > 250){
                countF =25;
                time = 0;
                mLeftWheel.setCount(0);
            }
            break;
		case 25:								//1and3/4回転
            mLeftWheel.setPWM(10);
            mRightWheel.setPWM(-10);
            if(mLeftWheel.getCount() > 1268){
                countF ++;
                mLeftWheel.setCount(0);
            }
            break;
        //sen sagasu
        case 26:								//階段を降りる準備、26から299
            time++;
            //mLineTracer->runFind();
            mLeftWheel.setPWM(5);
            mRightWheel.setPWM(5);
			//if(time > 870){
            if(mLeftWheel.getCount() > 226){
                countF ++;
                time = 0;
            	mLeftWheel.setCount(0);
                mLeftWheel.setPWM(0);
                mRightWheel.setPWM(0);
            }
            break;
        case 27:
            time++;
            mLeftWheel.setPWM(0);
            mRightWheel.setPWM(0);
            if(time > 250){
                countF ++;
                time = 0;
                mLineTracer->Init();
            }
            break;
        case 28:
            //mLineTracer->Init();
            time++;
            mLineTracer->runStop();
            if(time > 250){
            	tailControll(85);
                countF =291;
                time = 0;
            }
            break;
		case 291:
			if(mTailWheel.getCount() > 80)tailControll2(80);
			mTailWheel.setPWM(0);
	        countF =2911;
	        time = 0;
	        mLeftWheel.setCount(0);
            break;
		case 2911:
			time++;
			if(time > 250){
          
	            countF =292;
	            time = 0;
	           	mLeftWheel.setCount(0);
            }
            break;
		case 292:
			if(mTailWheel.getCount() > 75)tailControll2(75);
			mTailWheel.setPWM(0);
         	countF =2921;
	        time = 0;
	        mLeftWheel.setCount(0);
            break;
		case 2921:
			time++;
			if(time > 250){
          
	            countF =293;
	            time = 0;
	           	mLeftWheel.setCount(0);
            }
            break;
		case 293:
			if(mTailWheel.getCount() > 75)tailControll2(75);
			mTailWheel.setPWM(0);
			countF =2931;
	        time = 0;
	        mLeftWheel.setCount(0);
            break;
		case 2931:
			time++;
			if(time > 250){
          
	            countF =29;
	            time = 0;
	           	mLeftWheel.setCount(0);
            }
            break;
		 case 29:                            //階段を降りる
			time++;
          	mLeftWheel.setPWM(28);
            mRightWheel.setPWM(28);
			tailControll2(x);
            if(time > 2){
                x=x-1;
            	if(x<60){
            		countF =289;
            		mTailWheel.setPWM(0);
            	}
                time = 0;
            }
			//mTailWheel.setPWM(1);	
			break;
		 case 289:                           
			time++;
          	mLeftWheel.setPWM(28);
            mRightWheel.setPWM(28);
			tailControll2(x);
            if(	mLeftWheel.getCount()>100){
            	countF =30;
            	mTailWheel.setPWM(0);
            	mLeftWheel.setPWM(0);
            	mRightWheel.setPWM(0);
                time = 0;
            }
			//mTailWheel.setPWM(1);	
			break;
				case 30:
			time++;
			//while( mTailWheel.getCount() < 45)mTailWheel.setPWM(10);
			if(time<30 ){
				mRightWheel.setPWM(-5);
            	mLeftWheel.setPWM(-5);
			}
			/*if(time > 2 && time<10 ){
				mRightWheel.setPWM(30);
            	mLeftWheel.setPWM(30);
			}*/
			if(time>30){
				countF++;
                time = 0;
			}
			break;
		case 31:
			time++;
			//while( mTailWheel.getCount() < 45)mTailWheel.setPWM(10);	
			//mTailWheel.setPWM(0);
			mRightWheel.setPWM(0);
            mLeftWheel.setPWM(0);
			/*if(time > 2 && time<10 ){
				mRightWheel.setPWM(30);
            	mLeftWheel.setPWM(30);
			}*/
			if(time>500){
				countF ++;
                time = 0;
			}
			break;
		case 32:
			time++;
			//while( mTailWheel.getCount() < 45)mTailWheel.setPWM(10);	
			//mTailWheel.setPWM(0);
			mRightWheel.setPWM(-1);
            mLeftWheel.setPWM(-1);
			/*if(time > 2 && time<10 ){
				mRightWheel.setPWM(30);
            	mLeftWheel.setPWM(30);
			}*/
			if(time>550){
				countF ++;
                time = 0;
			}
			break;
		case 33:
            time++;
            mRightWheel.setPWM(10);
            mLeftWheel.setPWM(10);
            if(time > 140 ){
                countF ++;
                time = 0;
            }
            break;
		 case 34:
            time++;
            mRightWheel.setPWM(0);
            mLeftWheel.setPWM(0);
            if(time > 555 ){
               	countF ++;
                time = 0;
            }
            break;
        case 35:
            time++;
            static int hoge=0;
            mRightWheel.setPWM(15);
            mLeftWheel.setPWM(15);
           	tailControll(x);
            if(time > 5 ){
                x=x+1;
                hoge++;
            	if(hoge>20)countF ++;
                time = 0;
            }
            break;
		case 36:
            time++;
            mRightWheel.setPWM(0);
            mLeftWheel.setPWM(0);
            if(time > 10 ){
               	countF ++;
                time = 0;
            }
            break;
		case 37:
			mLineTracer->runFind();
			time++;
			if(time > 2000){
				time = 0;
				countF ++;
				//mState = LAST;
            }
            break;
        case 38:
			mLineTracer->runOnOff2();
			time++;
			if(time > 1000){
				time = 0;
				countF ++;
				//mState = LAST;
            }
            break;
            
		case 39:
			mLeftWheel.setPWM(-10);
			mRightWheel.setPWM(-10);
			mTailWheel.setPWM(3);
			if(mTailWheel.getCount()>80){
				mLeftWheel.setPWM(0);
				mRightWheel.setPWM(0);
				mTailWheel.setPWM(0);
				mLeftWheel.setCount(0);
				time++;
			}
			time++;
			if(time > 450){
				mTailWheel.setPWM(0);
				time = 0;
				countF = 100;
				//sonarInt = 0;
			}
		break;
	}
}

/**
 * 難所２の処理
 */
void LineTracerWithStarter::execLast(){
	static int8_t countL = 0;
	static int32_t time = 0;
	switch(countL){
		case 0:
			mLineTracer->runOnOff2();
		if(mLineTracer->runOnOff2() == 1){
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
			mLeftWheel.setCount(0);
			countL++;
		}
		break;
		case 1:
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
			time++;
			if(time >1000){
				time = 0;
				countL++;
				mLeftWheel.setCount(0);
			}
		break;
		case 2:
		mLineTracer->runOnOff3();
		if(mLineTracer->runOnOff3() == 3){
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
			mLeftWheel.setCount(0);
			countL++;
		}
		break;
		case 3:
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
			time++;
			if(time >1000){
				time = 0;
				countL++;
				mLeftWheel.setCount(0);
			}
		break;
		case 4:
		mLineTracer->runOnOff2();
		if(mLeftWheel.getCount() > 150){
			countL++;
		}
		break;
	case 5:
			mLineTracer->runOnOff();
			mLeftWheel.setPWM(0);
			mRightWheel.setPWM(0);
		break;
	}


}
void LineTracerWithStarter::tailControll(int deg) {
	if(deg >= 10)mTailWheel.setPWM(50);
	else if(deg == 0)mTailWheel.setPWM(-50);
	while(1){
		if(mTailWheel.getCount() > deg)break;
	}
	mTailWheel.setPWM(0);
}
//尻尾戻る
void LineTracerWithStarter::tailControll2(int deg) {
	if(deg >= 10)mTailWheel.setPWM(-1);
	else if(deg == 0)mTailWheel.setPWM(50);
	while(1){
		if(mTailWheel.getCount() < deg)break;
	}
	mTailWheel.setPWM(1);
}