/*
 * a498x.h
 *
 * Created: 04.09.2015 17:29:55
 *  Author: HSO
 */ 

#include <inttypes.h>

#ifndef A498X_H_
#define A498X_H_

#define MOTOR_PORT PORTA
#define STEP_PIN 5
#define MOTOR_ENABLE 0x00
#define MOTOR_DISABLE 0x01
#define FULL_STEP 0x00
#define HALF_STEP 0x01
#define QUARTER_STEP 0X02
#define EIGHT_STEP 0X03
#define SIXTEENTH_STEP 0X07
#define STOP_MOTOR 0
#define MOVECW 1
#define MOVECCW -1

typedef struct{
	
	volatile uint8_t *port;
	
	uint8_t portDirValue;
	uint8_t portStepModeValue;
	
	uint8_t currentStepMode;
	int currentDir;
	int8_t currentPeriodms;
	int8_t currentState;
	uint16_t currentTickDiv;
	uint16_t currentStepCount;
	// new parameters
	uint8_t newStepMode;
	int8_t flagNewMode;
	int newDir;
	int8_t newPeriodms;
	int8_t newState;
	uint16_t newTickDiv;
	
	
}motor_t;

extern void A498x_setNewMotorMode(motor_t *m,int enable, int dir, uint8_t stepMode, uint16_t div);
extern void A498x_initMotorStruct(motor_t *m, uint8_t port);
extern void A498x_moveMotor(motor_t *m);

extern void A498x_executeNSteps(motor_t *m, uint32_t nSteps, int dir, uint8_t stepMode);
extern void A498x_setMotorStepMode(motor_t *m , uint8_t stepMode);
extern void A498x_setMotorDir(motor_t *m, int dir);
extern void A498x_setMotorState(motor_t *m, int enable);
extern void A498x_setMotorSpeed(motor_t *m , uint16_t div);







#endif /* A498X_H_ */