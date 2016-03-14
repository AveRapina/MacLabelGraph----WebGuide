/*
 * drv881x.c
 *
 * Created: 04.09.2015 17:28:31
 *  Author: HSO
 */ 

#include <inttypes.h>
#include <avr/io.h>

#include "drv881x.h"

#include "../usart/usart.h"
#include "../utils/my_utils.h"


//#define DEBUG_MOTOR


/* const char sequences[8]={0b00001100,
						 0b00000100,
					     0b00000110,
						 0b00000010,
				         0b00000011,
						 0b00000001,
						 0b00001001,
						 0b00001000}; */
						 
//				Enable A, Dir A, Enable B, Dir B
const char sequences[8]={0b00001110,
						 0b00000010,
						 0b00001010,
						 0b00001001,
						 0b00001011,
						 0b00000111,
						 0b00001111,
						 0b00001100};



#ifdef DEBUG_MOTOR
char buf[15];
#endif

//local prototypes
void DRV_setOutputPins(motor_t *m);
void DRV_updateParam(motor_t *m);
void DRV_setOutputCoils(uint8_t index);
//uint8_t DRV_computeNextIdx(int currentIdx, int dir, int stepMode,int maxIdx);
void DRV_computeNextState(motor_t * m, int dir, uint8_t stepMode, uint8_t speed, uint8_t torque);
int DRV_compNextIdx(motor_t *m);

/************************************************************************/
/* @Put the values on port                                                                    */
/************************************************************************/
void DRV_setOutputPins(motor_t *m){

//port =&PORTA;
	//first update the values
	
	// set the output values
	//*m->port = (sequences[m->currentIndex] |(m->currentPwm <<4)) & (m->currentState);
	*m->port = sequences[m->currentIndex] |(m->currentPwm <<4);// | (m->currentState<<7);
	// compute next idx
	DRV_updateNextSeq(m);
	
	
	
}



/************************************************************************/
/* @init struct                                                                     */
/************************************************************************/
void DRV_initMotorStruct(motor_t *m, uint8_t port){
	m->nMaxIndex =  N_STEPS_MAX;
	m->currentDir= STOP_MOTOR;
	m->currentIndex=0;
	m->currentStepMode=HALF_STEP;
	m->currentPwm=DRV_PWM_00;
	m->port=port;
	m->flagNewMode=0;
	
}

/************************************************************************/
/*@Update next sequence                                                 */
/************************************************************************/
int DRV_updateNextSeq(motor_t *m){
	
	// Check for new mode parameters
	if(m->flagNewMode>0){
		// lets copy to current fields
		m->currentDir =m->newDir;
		m->currentStepMode=m->newStepMode;
		m->currentPwm =m->newPwm;
		m->currentState =m->newState;
		m->currentTickDiv =m->newTickDiv;
		
		m->flagNewMode=0; // clear flag
	}
	
	uint8_t idx;

	m->currentTickDiv=1;
	// update next index only of valid
	m->currentStepCount++;
	// for each engine
	if(m->currentStepCount % m->currentTickDiv ==0){ // prescaler motor
		
	
	
		// procced with computation of next idx
		if(m->currentStepMode == HALF_STEP){
		
			if(m->currentDir >= MOVECW){
				m->currentIndex++;
				if(m->currentIndex >7)m->currentIndex=0;
			
				#ifdef DEBUG_MOTOR
				idx=m->currentIndex;
				sprintf(buf,"H-CW id: %i\n\r",idx);
				USART1_sendStr(buf);
				#endif DEBUG_MOTOR
			
				return 1;
			}
			if(m->currentDir <= MOVECCW){
				m->currentIndex--;
				if(m->currentIndex <0)m->currentIndex=7;
				idx=m->currentIndex;
			
				#ifdef DEBUG_MOTOR
				idx=m->currentIndex;
				sprintf(buf,"H-CCW id: %i\n\r",idx);
				USART1_sendStr(buf);
				#endif DEBUG_MOTOR
				return 1;
			}
			if(m->currentDir == STOP_MOTOR){
				#ifdef DEBUG_MOTOR
				USART1_sendStr("STOP\n\r");
				#endif
				return 1;
			}
		
		}else{
		
			int8_t rest = m->currentIndex %2;
			if(m->currentDir >= MOVECW){
				if(rest==0) m->currentIndex+=2;
				else m->currentIndex++;
				if(m->currentIndex >6)m->currentIndex=0;
			
				#ifdef DEBUG_MOTOR
				idx=m->currentIndex;
				sprintf(buf,"F-CW id: %i\n\r",idx);
				USART1_sendStr(buf);
				#endif DEBUG_MOTOR
			
				return 1;
			}
			if(m->currentDir <= MOVECCW){
				if(rest==0) m->currentIndex-=2;
				else m->currentIndex--;
				if(m->currentIndex <0)m->currentIndex=6;
			
				#ifdef DEBUG_MOTOR
				idx=m->currentIndex;
				sprintf(buf,"F-CCW id: %i\n\r",idx);
				USART1_sendStr(buf);
				#endif DEBUG_MOTOR
				return 1;
			}
			if(m->currentDir == STOP_MOTOR){
				#ifdef DEBUG_MOTOR
				USART1_sendStr("STOP\n\r");
				#endif
				return 1;
			}
		
		}
	}
	
	return -1; // anythiing fail, debug only
}


/************************************************************************/
/* @update new motor mode                                                                     */
/************************************************************************/
void DRV_setNewMotorMode(motor_t *m, int enable, int dir, int stepMode, int pwm, uint16_t div){
	
	m->newDir=dir;
	m->newPwm=pwm;
	m->newStepMode=stepMode;
	m->newState=enable;
	m->newTickDiv=div; // by default
	m->flagNewMode++; // activate flag for next update


}



// Small primitives

/************************************************************************/
/* CHECK                                                                     */
/************************************************************************/
void DRV_executeNSteps(motor_t *m, uint32_t nSteps, int dir, int stepMode){
	uint32_t step_cnt=0;
	while(step_cnt!=nSteps){
		DRV_setNewMotorMode(m,MOTOR_ENABLE,dir,stepMode,DRV_PWM_71,1);
	}
}

/************************************************************************/
/* @set motor speed
/************************************************************************/
void DRV_setMotorSpeed(motor_t *m , uint16_t div){
	m->newTickDiv =div;
	m->flagNewMode++;
}

/************************************************************************/
/* @set motor stepMode                                                                     */
/************************************************************************/
void DRV_setMotorStepMode(motor_t *m , int stepMode){
	m->newStepMode=stepMode;
	m->flagNewMode++;
}


/************************************************************************/
/* @set dir                                                                     */
/************************************************************************/
void DRV_setMotorDir(motor_t *m, int dir){
	m->newDir=dir;
	m->flagNewMode++;

}

/************************************************************************/
/* @set pwm                                                                     */
/************************************************************************/
void DRV_setMotorPWM(motor_t *m, int pwm){
	m->newPwm =pwm;
	m->flagNewMode++;
	
}

/************************************************************************/
/* @set motor mode                                                                     */
/************************************************************************/
void DRV_setMotorState(motor_t *m, int enable){
	m->newState =enable;
	m->flagNewMode++;
	
}





