/*
 * drv881x.h
 *
 * Created: 04.09.2015 17:28:55
 *  Author: HSO
 */ 


#ifndef DRV881X_H_
#define DRV881X_H_

//obselete
#define RUN_MOTOR 1



#define MOTOR_ENABLE 0x01
#define MOTOR_DISABLE 0x00
#define FULL_STEP 0x00
#define HALF_STEP 0x01
#define STOP_MOTOR 0
#define MOVECW 1
#define MOVECCW -1



/*
Sets bridge A current: 00 = 100%,
Bridge A current set 01 = 71%, 10 = 38%, 11 = 0
AI1 25 I Internal pulldown.
*/
#define DRV_PWM_100 0x00
#define DRV_PWM_71 0x01
#define DRV_PWM_38 0x02
#define DRV_PWM_00 0x03

#define N_STEPS_MAX 7

typedef struct{
	
	volatile uint8_t *port;
	int8_t nMaxIndex;
	int8_t currentIndex;
	int8_t currentStepMode;
	int8_t currentDir;
	int8_t currentPwm;
	int8_t currentPeriodms;
	int8_t currentState;
	uint16_t currentTickDiv;
	uint16_t currentStepCount;
	// new parameters
	int8_t newStepMode;
	int8_t flagNewMode;
	int8_t newDir;
	int8_t newPwm;
	int8_t newPeriodms;
	int8_t newState;
	uint16_t newTickDiv;
	
	
}motor_t;




// current ones
//extern void DRV_setNewMotorMode(motor_t *m, int enable, int dir, int stepMode, int pwm,  uint16_t div);
extern void DRV_setNewMotorMode(motor_t *m, int enable, int dir, int stepMode, int pwm, uint16_t div);
extern int DRV_updateNextSeq(motor_t *m);
extern void DRV_initMotorStruct(motor_t *m, uint8_t port);
extern void DRV_setOutputPins(motor_t *m);

extern void DRV_executeNSteps(motor_t *m, uint32_t nSteps, int dir, int stepMode);
extern void DRV_setMotorStepMode(motor_t *m , int stepMode);
extern void DRV_setMotorDir(motor_t *m, int dir);
extern void DRV_setMotorPWM(motor_t *m, int pwm);
extern void DRV_setMotorState(motor_t *m, int enable);
extern void DRV_setMotorSpeed(motor_t *m , uint16_t div);




#endif /* DRV881X_H_ */