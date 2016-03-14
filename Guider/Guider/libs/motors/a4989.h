/*
 * a4989.h
 *
 * Created: 22-02-2016 19:40:46
 *  Author: Nelson
 */ 


#ifndef A4989_H_
#define A4989_H_
#include <inttypes.h>

#define MOTOR_PORT PORTA
#define STEP_PIN 5
#define MOTOR_ENABLE 0x00
#define MOTOR_DISABLE 0x01
#define FULL_STEP 0x00
#define HALF_STEP 0x01
#define QUARTER_STEP 0X02
#define SIXTEENTH_STEP 0X03
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

extern void A4989_setNewMotorMode(motor_t *m,int enable, int dir, uint8_t stepMode, uint16_t div);
extern void A4989_initMotorStruct(motor_t *m, uint8_t port);
extern void A4989_moveMotor(motor_t *m);

extern void A4989_executeNSteps(motor_t *m, uint32_t nSteps, int dir, uint8_t stepMode);
extern void A4989_setMotorStepMode(motor_t *m , uint8_t stepMode);
extern void A4989_setMotorDir(motor_t *m, int dir);
extern void A4989_setMotorState(motor_t *m, int enable);
extern void A4989_setMotorSpeed(motor_t *m , uint16_t div);

#endif /* A4989_H_ */