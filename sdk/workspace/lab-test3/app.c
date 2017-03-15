/**
 * TOPPERS/EV3 RT
 *
 * �ȒP�ȃv���O���~���O�ɂ��Mindstorms EV3����
 *
 * 2017/3/8 TCS�FS-NAKA
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

#define SONAR_ALERT_DISTANCE1 80	/* �����g�Z���T�ɂ���Q�����m����[cm] ���x��1 ���� */
#define SONAR_ALERT_DISTANCE2 30	/* �����g�Z���T�ɂ���Q�����m����[cm] ���x��2 ��~ */

static int Global_Count;
static int G_Distance;

/* �֐��v���g�^�C�v�錾 */
static int sonar_alert(void);

//*****************************************************************************
// �T�v�F1000ms���[�`��
//
// �����F1000(ms)
//*****************************************************************************
void Counter_1000cyc(intptr_t  unused) {

	Global_Count = Global_Count + 1;
}

//*****************************************************************************
// �T�v�F���C������
//
//
//*****************************************************************************

void main_task(intptr_t unused) {

	int Back_Mode; // �o�b�N���[�h
	int Back_Init; // �o�b�N���[�h��������

	char buff_val[30];
	
	/* �Z���T�[�|�[�g�̐ݒ� */
	ev3_sensor_config( sonar_sensorF,ULTRASONIC_SENSOR );
	ev3_sensor_config( sonar_sensorB,ULTRASONIC_SENSOR );
	
	/* ���[�^�[�o�̓|�[�g�̐ݒ� */
	ev3_motor_config( left_motor, MEDIUM_MOTOR );
	ev3_motor_config( right_motor, LARGE_MOTOR );
	ev3_motor_config( front_motor, LARGE_MOTOR );
	
	/* ���[�^�[�G���R�[�_���Z�b�g */
	ev3_motor_reset_counts( left_motor );
	ev3_motor_reset_counts( right_motor );
	ev3_motor_reset_counts( front_motor );

	/* ���C���^�X�N */
	Back_Mode = 0;
	Back_Init = 1;
	G_Distance = 0;
	while ( 1 ) {
		
		/*------------------------*/
		/* �ԗ���ԕʎ����^�]���� */
		/*------------------------*/
		if ( Back_Mode == 0 ) {				// �O�i���[�h ---------->
			
			if ( sonar_alert() == 1 ) { 	// ����
				// LED�F�I�����W
				ev3_led_set_color( LED_ORANGE );

				// ����
				ev3_motor_set_power( left_motor, -40 );
				ev3_motor_set_power( right_motor, -40 );
			}
			else if ( sonar_alert() == 2 ) {// ��~
				// LED�F��
				ev3_led_set_color( LED_RED );
			
				// ��~
				ev3_motor_stop( left_motor, true ); // �u���[�L���[�h
				ev3_motor_stop( right_motor, true );
				
				// ����
				ev3_motor_reset_counts( front_motor );
				ev3_motor_rotate( front_motor, -50, 50, true );
				
				// �o�b�N���[�h�ڍs
				Back_Mode = 1; // �o�b�N���[�h�ڍs
				Back_Init = 1;				
			}
			else {							// �ʏ푖�s
				// LED�F��
				ev3_led_set_color( LED_GREEN );
				
				// �������s �O�i POWER 100%
				ev3_motor_set_power( left_motor, -100 ); 
	    		ev3_motor_set_power( right_motor, -100 );
			}
		}
		else {								// �o�b�N���[�h -------->
			// �o�b�N���[�h��������
			if ( Back_Init == 1 ) {
				// �����n���h���N��
				Global_Count = 0;
				ev3_sta_cyc( COUNT_CYC1 );
				
				// ���������I��
				Back_Init = 0;
			}
			
			// �o�b�N���[�h���s
			ev3_motor_set_power( left_motor, 20 );
			ev3_motor_set_power( right_motor, 20 );
			
			// �o�b�N���[�h�J�n��A��2�b��Ɏ�����~�A���ǌ�O�i���[�h�ֈڍs
			if ( Global_Count > 2 ) {
				
				// ��~
				ev3_motor_stop( left_motor, true ); // �u���[�L���[�h
				ev3_motor_stop( right_motor, true );

				// ����
				ev3_motor_reset_counts( front_motor );
				ev3_motor_rotate( front_motor, 50, 50, true );
				
				// �����n���h����~
				ev3_stp_cyc( COUNT_CYC1 );
				
				// �O�i���[�h�ڍs
				Back_Mode = 0;
			}
		}
		
		// ��ʕ\��
		//sprintf( buff_val, "Distance�F%d ", G_Distance );
		//ev3_lcd_draw_string( buff_val, 0, 10 );
	}
}

//*****************************************************************************
// �T�v�F�����g�Z���T�ɂ���Q�����m
//
// �߂�l 1:���x��1�A2:���x��2�A3:���x��3
//*****************************************************************************
static int sonar_alert(void)
{
    static int alert = 0;
    signed int distance;
	
	distance = ev3_ultrasonic_sensor_get_distance( sonar_sensorF );
	G_Distance = distance;

	if ( distance == 0 ) { // ������Ԃ̒���
		alert = 0;
	}
    else if ( distance < SONAR_ALERT_DISTANCE1 ) {
		alert = 1; // ��Q�������m
		if ( distance < SONAR_ALERT_DISTANCE2 ) {
			alert = 2; // ��Q�������m
		}
    }
	else{
		alert = 0; // ��Q������
	}
    return alert;
}

