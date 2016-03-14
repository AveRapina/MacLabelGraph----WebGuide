/*
 * a4989.c
 *
 * Created: 22-02-2016 19:40:59
 *  Author: Nelson
 */ 


#include <inttypes.h>
#include <avr/io.h>

#include "a498x.h"

#include "../usart/usart.h"
#include "../utils/my_utils.h"
#include "../utils/bit_tools.h"

// Local prototypes
void A498x_sendStep(void);
void A498x_setOutputPins(motor_t *m);
void A498x_decodeFunction(motor_t *m);
void A498x_updateNextStep(motor_t *m);


/************************************************************************/
/*Init Motor Struct                                                     */
/************************************************************************/
void A498x_initMotorStruct(motor_t *m, uint8_t port){
	m->portDirValue=0;
	m->portStepModeValue=0;
	m->currentDir= STOP_MOTOR;
	m->currentStepMode=HALF_STEP;
	m->port=port;
	m->flagNewMode=0;
}



char bufferdummy[10]={0};
/************************************************************************/
/* @ Move Motor                                                                     */
/************************************************************************/
void A498x_moveMotor(motor_t *m){

	A498x_updateNextStep(m);
	// check if dir is different zero
	if(m->currentDir !=STOP_MOTOR){
		
		A498x_setOutputPins(m);
		A498x_sendStep(); // issue step
	}
	// update next run
	
	
}


/************************************************************************/
/* @Do Step                                                             */
/************************************************************************/
void A498x_sendStep(){
	
	_delay_us(100);
	bit_set(MOTOR_PORT,STEP_PIN);
	_delay_us(100);
	bit_clear(MOTOR_PORT,STEP_PIN);
}

/************************************************************************/
/*@put value on port                                                    */
/************************************************************************/
void A498x_setOutputPins(motor_t *m){
	//(7-6) input limit switch,5-step,4-dir,3-MS3,2-MS2,1-MS1,0-enable
	A498x_decodeFunction(m); // decode function
	*m->port =	m->currentState | (m->portStepModeValue<<1) | (m->portDirValue<<4);//DRV_updateNextSeq(m);
}


/************************************************************************/
/*@decode Function														*/
/************************************************************************/
void A498x_decodeFunction(motor_t *m){
	
	if(m->currentDir < 0){
		m->portDirValue =0;
	}
	else{
		m->portDirValue =1;
	}
	
	m->portStepModeValue = m->currentStepMode;
}

/************************************************************************/
/*@update next sequence                                                 */
/************************************************************************/
void A498x_updateNextStep(motor_t *m){

	// Check for new mode parameters
	if(m->flagNewMode>0){
		
		// lets copy to current fields
		m->currentDir =m->newDir;
		m->currentStepMode=m->newStepMode;
		m->currentState =m->newState;
		m->currentTickDiv =m->newTickDiv;
		
		
		m->flagNewMode=0; // clear flag
	}
	
}

/************************************************************************/
/*@update new motor mode                                                */
/************************************************************************/

void A498x_setNewMotorMode(motor_t *m, int enable, int dir, uint8_t stepMode, uint16_t div){
	
	m->newDir=dir;
	m->newStepMode=stepMode;
	m->newState=enable;
	m->newTickDiv=div; // by default
	m->flagNewMode++; // activate flag for next update
}

/************************************************************************/
/*@execute N steps                                                                    */
/************************************************************************/
void A498x_executeNSteps(motor_t *m, uint32_t nSteps, int dir, uint8_t stepMode){
	


	uint32_t step_cnt=0;
	while(step_cnt!=nSteps){
		A498x_setNewMotorMode(m,MOTOR_ENABLE,dir,stepMode,1);
	}
}

/************************************************************************/
/* @set motor speed														*/
/************************************************************************/
void A498x_setMotorSpeed(motor_t *m , uint16_t div){
	m->newTickDiv =div;
	m->flagNewMode++;
}

/************************************************************************/
/* @set motor stepMode                                                  */
/************************************************************************/
void A498x_setMotorStepMode(motor_t *m , uint8_t stepMode){
	m->newStepMode=stepMode;
	m->flagNewMode++;
}

/************************************************************************/
/* @set dir                                                             */
/************************************************************************/
void A498x_setMotorDir(motor_t *m, int dir){
	m->newDir=dir;
	m->flagNewMode++;

}

/************************************************************************/
/* @set motor mode                                                                     */
/************************************************************************/
void A498x_setMotorState(motor_t *m, int enable){
	m->newState =enable;
	m->flagNewMode++;
	
}