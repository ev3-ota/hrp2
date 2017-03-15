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
static int G_Distance;

/* 関数プロトタイプ宣言 */
static int sonar_alert(void);

//*****************************************************************************
// 概要：1000msルーチン
//
// 周期：1000(ms)
//*****************************************************************************
void Counter_1000cyc(intptr_t  unused) {

	Global_Count = Global_Count + 1;
}

//*****************************************************************************
// 概要：メイン処理
//
//
//*****************************************************************************

void main_task(intptr_t unused) {

	int Back_Mode; // バックモード
	int Back_Init; // バックモード初期処理

	char buff_val[30];
	
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
	Back_Mode = 0;
	Back_Init = 1;
	G_Distance = 0;
	while ( 1 ) {
		
		/*------------------------*/
		/* 車両状態別自動運転制御 */
		/*------------------------*/
		if ( Back_Mode == 0 ) {				// 前進モード ---------->
			
			if ( sonar_alert() == 1 ) { 	// 減速
				// LED：オレンジ
				ev3_led_set_color( LED_ORANGE );

				// 減速
				ev3_motor_set_power( left_motor, -40 );
				ev3_motor_set_power( right_motor, -40 );
			}
			else if ( sonar_alert() == 2 ) {// 停止
				// LED：赤
				ev3_led_set_color( LED_RED );
			
				// 停止
				ev3_motor_stop( left_motor, true ); // ブレーキモード
				ev3_motor_stop( right_motor, true );
				
				// 操舵
				ev3_motor_reset_counts( front_motor );
				ev3_motor_rotate( front_motor, -50, 50, true );
				
				// バックモード移行
				Back_Mode = 1; // バックモード移行
				Back_Init = 1;				
			}
			else {							// 通常走行
				// LED：緑
				ev3_led_set_color( LED_GREEN );
				
				// 自動走行 前進 POWER 100%
				ev3_motor_set_power( left_motor, -100 ); 
	    		ev3_motor_set_power( right_motor, -100 );
			}
		}
		else {								// バックモード -------->
			// バックモード初期処理
			if ( Back_Init == 1 ) {
				// 周期ハンドラ起動
				Global_Count = 0;
				ev3_sta_cyc( COUNT_CYC1 );
				
				// 初期処理終了
				Back_Init = 0;
			}
			
			// バックモード実行
			ev3_motor_set_power( left_motor, 20 );
			ev3_motor_set_power( right_motor, 20 );
			
			// バックモード開始後、約2秒後に自動停止、操舵後前進モードへ移行
			if ( Global_Count > 2 ) {
				
				// 停止
				ev3_motor_stop( left_motor, true ); // ブレーキモード
				ev3_motor_stop( right_motor, true );

				// 操舵
				ev3_motor_reset_counts( front_motor );
				ev3_motor_rotate( front_motor, 50, 50, true );
				
				// 周期ハンドラ停止
				ev3_stp_cyc( COUNT_CYC1 );
				
				// 前進モード移行
				Back_Mode = 0;
			}
		}
		
		// 画面表示
		//sprintf( buff_val, "Distance：%d ", G_Distance );
		//ev3_lcd_draw_string( buff_val, 0, 10 );
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
	G_Distance = distance;

	if ( distance == 0 ) { // 初期状態の調整
		alert = 0;
	}
    else if ( distance < SONAR_ALERT_DISTANCE1 ) {
		alert = 1; // 障害物を検知
		if ( distance < SONAR_ALERT_DISTANCE2 ) {
			alert = 2; // 障害物を検知
		}
    }
	else{
		alert = 0; // 障害物無し
	}
    return alert;
}

