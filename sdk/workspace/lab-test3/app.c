/**
 * TOPPERS/EV3 RT
 *
 * 簡単なプログラミングによるMindstorms EV3操作
 *
 * 2017/3/8 TCS：S-NAKA
 * 
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static const sensor_port_t
    sonar_sensorF	= EV3_PORT_1,
    sonar_sensorB	= EV3_PORT_4;

static const motor_port_t 
    left_motor		= EV3_PORT_C,
    right_motor		= EV3_PORT_D,
    front_motor		= EV3_PORT_A;

#define SONAR_ALERT_DISTANCE1 80	/* 超音波センサによる障害物検知距離[cm] レベル1 減速 */
#define SONAR_ALERT_DISTANCE2 30	/* 超音波センサによる障害物検知距離[cm] レベル2 停止 */

static int Global_Count;

/* 関数プロトタイプ宣言 */
static int sonar_alert(void);

//*****************************************************************************
// 概要：1000msルーチン
//
// 周期：1000(ms)
//*****************************************************************************
void Counter_1000cyc(intptr_t  unused) {

		Global_Count++;
}

//*****************************************************************************
// 概要：メイン処理
//
//
//*****************************************************************************

void main_task(intptr_t unused) {

	/* センサーポートの設定 */
	ev3_sensor_config( sonar_sensorF,ULTRASONIC_SENSOR );
	ev3_sensor_config( sonar_sensorB,ULTRASONIC_SENSOR );
	
	/* モーター出力ポートの設定 */
	ev3_motor_config( left_motor, MEDIUM_MOTOR );
	ev3_motor_config( right_motor, LARGE_MOTOR );
	ev3_motor_config( front_motor, LARGE_MOTOR );
	
	/* モーターエンコーダリセット */
	ev3_motor_reset_counts( left_motor );
	ev3_motor_reset_counts( right_motor );
	ev3_motor_reset_counts( front_motor );

	/* メインタスク */
	while ( 1 ) {
		
		/*------------------------*/
		/* 車両状態別自動運転制御 */
		/*------------------------*/
		if ( sonar_alert() == 2 ) {	// 走行状態1
			
			// 停止
			ev3_motor_stop( left_motor, true ); // ブレーキモード
			ev3_motor_stop( right_motor, true );
		
		}
		else {						// 走行状態0
			// 自動走行 前進 POWER 100%
			ev3_motor_set_power( left_motor, -100 ); 
	    	ev3_motor_set_power( right_motor, -100 );
		}
	}
}

//*****************************************************************************
// 概要：超音波センサによる障害物検知
//
// 戻り値 1:レベル1、2:レベル2、3:レベル3
//*****************************************************************************
static int sonar_alert(void)
{
    static int alert = 0;
    signed int distance;

    distance = ev3_ultrasonic_sensor_get_distance( sonar_sensorF );

    if ( ( distance < SONAR_ALERT_DISTANCE1 ) && ( distance >= SONAR_ALERT_DISTANCE2 ) ) {
		alert = 1; // 障害物を検知
	}
	else if ( ( distance < SONAR_ALERT_DISTANCE2 ) && ( distance >= 0 ) ) {
		alert = 2; // 障害物を検知
	}
	else{
		alert = 0; // 障害物無し
	}
    return alert;
}

