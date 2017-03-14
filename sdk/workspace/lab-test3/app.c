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

/* �֐��v���g�^�C�v�錾 */
static int sonar_alert(void);

//*****************************************************************************
// �T�v�F1000ms���[�`��
//
// �����F1000(ms)
//*****************************************************************************
void Counter_1000cyc(intptr_t  unused) {

		Global_Count++;
}

//*****************************************************************************
// �T�v�F���C������
//
//
//*****************************************************************************

void main_task(intptr_t unused) {

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
	while ( 1 ) {
		
		/*------------------------*/
		/* �ԗ���ԕʎ����^�]���� */
		/*------------------------*/
		if ( sonar_alert() == 2 ) {	// ���s���1
			
			// ��~
			ev3_motor_stop( left_motor, true ); // �u���[�L���[�h
			ev3_motor_stop( right_motor, true );
		
		}
		else {						// ���s���0
			// �������s �O�i POWER 100%
			ev3_motor_set_power( left_motor, -100 ); 
	    	ev3_motor_set_power( right_motor, -100 );
		}
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

    if ( ( distance < SONAR_ALERT_DISTANCE1 ) && ( distance >= SONAR_ALERT_DISTANCE2 ) ) {
		alert = 1; // ��Q�������m
	}
	else if ( ( distance < SONAR_ALERT_DISTANCE2 ) && ( distance >= 0 ) ) {
		alert = 2; // ��Q�������m
	}
	else{
		alert = 0; // ��Q������
	}
    return alert;
}

