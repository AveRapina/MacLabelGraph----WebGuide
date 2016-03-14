/*
 * Guider.c
 *
 * Created: 15-09-2015 19:03:17
 *  Author: HSO
 */ 
 
 

#define F_CPU 16000000UL

//#define DEBUG_MAIN 

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <inttypes.h>

#include "libs/motors/a498x.h"
//#include "libs/motors/drv881x.h"
#include "libs/adc/adc_analog.h"
#include "libs/usart/usart.h"
#include "libs/timer/timer_utils.h"
#include "libs/utils/my_utils.h"
#include "libs/lcds/hd774x.h"


#define CHCELL 1


#define SETPOINT_DEFAULT 1024/2
#define THRESHOLD_HIGH_DEFAULT 300
#define THRESHOLD_SLOW_DEFAULT 50
#define SENSOR_INVERTED_DEFAULT 1
#define N_SENSOR_FAILS 10
#define SENSOR_LEFT_LIMIT -1
#define SENSOR_RIGHT_LIMIT 1
#define SENSOR_FREE_LIMIT 0
#define SENSOR_ERROR 2
#define SERVO_CENTER 1
#define MOTOR_SETPOINT_DEFAULT 0


uint16_t nSensorFail;
uint16_t thSlow;
uint16_t thHigh;
uint16_t setPoint;
int8_t sensorInverted;

typedef struct{
	
	uint8_t initEeprom;
	uint16_t nSensorFail;
	uint16_t thSlow;
	uint16_t thHigh;
	uint16_t setPoint;
	int8_t sensorInverted;
	
}eeParamStruct_t;

typedef struct{
	
	uint16_t thHigh;
	uint16_t thSlow;
	int8_t sensorInverted;
	uint16_t cellReads;
	uint16_t setpoint;
	uint16_t currentTickDiv;
	
}showParams;

	
eeParamStruct_t EEMEM eeParamStruct_eemem;
eeParamStruct_t eeParamStruct;

//Global Vars
volatile uint8_t flagTaskControl = 0;
volatile uint8_t flagTaskButtons=0;
volatile uint8_t flagTaskMotor=0;
volatile uint8_t flagTaskLcd=0;
uint32_t stepCnt=0;


volatile char bufferDummy[20];

// Task periodicity 1ms base tick
volatile uint16_t taskButtonsPeriod = 100;
volatile uint16_t taskMotorPeriod =2;
volatile uint16_t taskControlPeriod =100;
volatile uint16_t taskLcdPeriod =500;

//prototypes
void configGPIO(void);
void schedulerInit(void);
void loadDefaultCtrlParameters();
uint8_t loadParamFromEeprom();
void eepromWriteParameters();
int checkErrorDir(uint16_t setPoint, uint16_t feedBack, uint16_t histLow, uint16_t histHigh, int rev);
uint16_t readSensors(uint8_t  ch);
void updateLcdStruct();
void calibrateMotorPos(motor_t *m );
void setCenter(motor_t *m);

/************************************************************************/
/* SCHEDULER STUFF                                                                     */
/************************************************************************/
// Timer setup for control loop
// freq = (FCPU /prescaler) /timerscale
// timerscale timer0 8bits = 256
//
//#define TIMER0_TICK 0.001
#define TIMER0_SCHED_PRESC TIMER0_PRESC128
#define TIMER0_SCHED_RELOAD 125//// 125 timer inc=1ms

//#define SAMPLING_PERIOD 0.002 // 1ms base time
volatile uint16_t schedulerMaxCount=5000;


/************************************************************************/
/* @Scheduler Init                                                                     */
/************************************************************************/
void schedulerInit(void){
	
	TCCR0 |= TIMER0_SCHED_PRESC;
	TCCR0 |= TIMER0_WAVEFORM_MODE_CTC;
	OCR0  = TIMER0_SCHED_RELOAD; // timer count reload
	TIMSK |= (1<< OCIE0); // Enable timer compare interrupt
	
}


/************************************************************************/
/* @Config GPIO                                                                     */
/************************************************************************/
void configGPIO(void){
	

	DDRA = 0b00111111;//(7-6) input limit switch,5-step,4-dir,3-MS3,2-MS2,1-MS1,0-enable
	PORTA =	0b11000000;//activate pullups for limit switch and outputs low
	DDRB = 0x00; //Defined as input for buttons (buttons can be modbus/can communication will be on the same wireness of lcd)
	PORTB = 0xFF; //Enable pullups
	
	// CHCK DEFINE PINS SENSOR INPUT
	
}



/************************************************************************/
/* checl limits sensors, -1 left, 0 free, 1 right                                                                     */
/************************************************************************/
int checkLimits(volatile uint8_t *p, uint8_t firstPin){
uint8_t portVal=0;

	portVal = ((((~*p))>>firstPin) &0x03); // Gosto disto
	
	if(portVal == 1) return SENSOR_LEFT_LIMIT;
	if(portVal == 2) return SENSOR_RIGHT_LIMIT;
	if(portVal == 3) return SENSOR_ERROR;
	
	return SENSOR_FREE_LIMIT;
}



/************************************************************************/
/* @compute error direction  and return -2 -1 0  1 2                                                                   */
/************************************************************************/
int checkErrorDir(uint16_t setPoint, uint16_t feedBack, uint16_t histLow, uint16_t histHigh, int rev){
int error=0;

	// basta ver a deadzone
	error = setPoint - feedBack;
	error *=rev; 
	if(abs(error) < histLow) {
		return 0;
	}
	if(abs(error) >= histHigh){
		if(error <0){ return -2;}
		else {return 2;}
	}
	if(abs(error)>= histLow){
		if(error <0) {return -1;}
		else {return 1;}
	}
}



/************************************************************************/
/* decode dir (redundante)                                                                     */
/************************************************************************/
int decodeDirMode(int error){
	if (error <0) return MOVECCW;
	if (error >0) return MOVECW;
	
	return STOP_MOTOR; 
}

/************************************************************************/
/* @decode mode                                                                      */
/************************************************************************/
int decodeStepMode(int error){
	
	if(error !=0){
		if (abs(error)>1) return FULL_STEP;
		else return HALF_STEP;
	}
	return HALF_STEP;
}


/************************************************************************/
/* read sensors                                                                     */
/************************************************************************/
uint16_t readSensors(uint8_t  ch){
uint16_t val=0;	
	val=ADC_readAndWAIT(ch);	
	return val;
	
}



// depois faz-se, isto e de acrodo com o que depois se quer
uint8_t readButtons(void){


	return (~PINB) &0x07;
}
/************************************************************************/
/* @eeprom write                                                                     */
/************************************************************************/

void eepromWriteParameters(){
	// save paramenetrs on the run
	
	eeParamStruct.initEeprom = 1; // eeprom init
	eeParamStruct.setPoint = setPoint;
	eeParamStruct.thHigh = thHigh;
	eeParamStruct.thSlow = thSlow;
	eeParamStruct.sensorInverted = sensorInverted;
	eeParamStruct.nSensorFail = nSensorFail;

	eeprom_write_block((const void*)&eeParamStruct,(void*)&eeParamStruct_eemem,sizeof(eeParamStruct_t));

}


/************************************************************************/
/* @restore Parameters from EEPROM                                                                     */
/************************************************************************/
uint8_t loadParamFromEeprom(){
	uint8_t temp=0;
	// read from emprom
	eeprom_read_block((void*)&eeParamStruct, (const void*)&eeParamStruct_eemem,sizeof(eeParamStruct_t));

	// test the first field to check if it was written else use default and load
	if(!eeParamStruct.initEeprom) return 0;
	
	else{
		// write to the global variables
		nSensorFail = eeParamStruct.nSensorFail;
		thSlow = eeParamStruct.thSlow;
		thHigh = eeParamStruct.thHigh;
		setPoint = eeParamStruct.setPoint;
		sensorInverted = eeParamStruct.sensorInverted;
	}
	
}


/************************************************************************/
/* @load default parameters to variables                                                                      */
/************************************************************************/
void loadDefaultCtrlParameters(){
		
		thSlow = THRESHOLD_SLOW_DEFAULT;
		thHigh = THRESHOLD_HIGH_DEFAULT;
		setPoint = SETPOINT_DEFAULT;
		nSensorFail = N_SENSOR_FAILS;
		sensorInverted = SENSOR_INVERTED_DEFAULT;
	
	
}


/*
void updateLcdStruct(){
	
	showParams.cellReads = readSensors(CHCELL);
	showParams.currentTickDiv = motor1.currentTickDiv;
	showParams.sensorInverted = sensorInverted;
	showParams.setpoint = setPoint;
	showParams.thHigh = thHigh;
	showParams.thSlow = thSlow;
	
}

*/

void showLcdSplash(void){
	
	LCD_clr();
	_delay_ms(200);
	LCD_gotoXY(5,0);
	LCD_sendString("WEB GUIDE");
	
	LCD_gotoXY(4,1);
	LCD_sendString("HSO & NELSON");
	
	
	LCD_gotoXY(4,3);
	LCD_sendString("Version: 1.0");
	
	_delay_ms(2000); // to show the message
	
	LCD_clr();
	
}



/************************************************************************/
/* Initial LCD form                                                                     */
/************************************************************************/
void showLcdInitial(void){
	LCD_gotoXY(0,0); // 20 
	LCD_sendString("  TH      TL      I " );
	
	LCD_gotoXY(0,2); // 20
	LCD_sendString(" REF    FEED     X  " );
	
}

void updateLcdValues(uint16_t setPoint, uint16_t feedBack, uint16_t histLow, uint16_t histHigh, int rev){
char buff[5];

	//1	
	LCD_gotoXY(0,1);
	sprintf(buff,"%3d",histHigh);
	LCD_sendString(buff);
	
	LCD_gotoXY(7,1);
	sprintf(buff,"%4d",histLow);
	LCD_sendString(buff);
	
	LCD_gotoXY(17,1);
	sprintf(buff,"%d",rev);
	LCD_sendString(buff);
	
	// 3
	LCD_gotoXY(1,3);
	sprintf(buff,"%d",setPoint);
	LCD_sendString(buff);
	
	LCD_gotoXY(8,3);
	sprintf(buff,"%4d",feedBack);
	LCD_sendString(buff);
	
	
}
/************************************************************************/
/* @Calibrate Center position                                                 */
/************************************************************************/
void calibrateMotorPos(motor_t *m){
	uint32_t currentCnt=0;
	//set values to load on PORT
	A498x_setNewMotorMode(m,MOTOR_ENABLE,MOVECW,FULL_STEP,1);//direcao directa procura sensor
	
	while (!((checkLimits(&PINA,6))==SENSOR_RIGHT_LIMIT)){
			
		if (flagTaskMotor){
			USART1_sendStr("DIREITA\r\n");
			
			A498x_moveMotor(m);
			flagTaskMotor=0; 
		}
	}
	//set values to load on PORT
	A498x_setNewMotorMode(m,MOTOR_ENABLE,MOVECCW,FULL_STEP,1);// direcao oposta procura outro sensor
	USART1_sendStr("ESQUERDA\r\n");
	while (!((checkLimits(&PINA,6))==SENSOR_LEFT_LIMIT)){
		
		if(flagTaskMotor){
			//A498x_setOutputPins(m);//load value on PORT
			A498x_moveMotor(m);//shift value to internal DAC
			flagTaskMotor=0;
			stepCnt++;
		}
	}
	sprintf(bufferDummy,"%u\r\n",stepCnt);
	USART1_sendStr(bufferDummy);
	stepCnt/=2;
	//set values to load on PORT
	A498x_setNewMotorMode(m,MOTOR_ENABLE,MOVECW,FULL_STEP,1);//direcao directa procura sensor
	USART1_sendStr("CENTRANDO\r\n");
	while (currentCnt <= stepCnt){
		if(flagTaskMotor){
			//A498x_setOutputPins(m);//load value on PORT
			A498x_moveMotor(m);//shift value to internal DAC
			flagTaskMotor=0;
			currentCnt++;
		}
	}
	sprintf(bufferDummy,"%u\r\n",currentCnt);
	USART1_sendStr(bufferDummy);
	USART1_sendStr("END\r\n");
	
}
/************************************************************************/
/* @ Set motor on center displacement                                   */
/************************************************************************/
void setCenter(motor_t *m){
uint32_t i=0;
		A498x_setNewMotorMode(m,MOTOR_ENABLE,MOVECW,FULL_STEP,1);//set values to load on PORT
		while (!((checkLimits(&PINA,6))==SENSOR_RIGHT_LIMIT)){
			if (flagTaskMotor){
				A498x_moveMotor(m);//load values on port & move motor
				flagTaskMotor=0;
			}
 		}
		A498x_setNewMotorMode(m,MOTOR_ENABLE,MOVECCW,FULL_STEP,1);//set values to load on PORT
		for(i=0;i<=stepCnt;i++){
			while(!flagTaskMotor);
				A498x_moveMotor(m);//load values on port & move motor
				flagTaskMotor=0;
		}
}
int main(void)
{	

	#define N_MOTORS 1
	motor_t motorStep[N_MOTORS];
	A498x_initMotorStruct(&motorStep[0], &PORTA);	
	
	USART1_config(USART1_MY_UBBRN,USART_DATA_FORMAT_8BITS|USART_STOP_BITS_1,USART_TRANSMIT_ENABLE|USART_RECEIVE_ENABLE| USART_INTERRUPT_ENABLE);
	//Config IO Ports
	configGPIO();
	//Init Scheduler
	schedulerInit();
	//Init ADC
	ADC_init(ADC_ENABLE,ADC_REF_VCC,ADC_MAX_FREQ); //CHECK PARAMS TO FILL IN
	//Init LCD
	LCD_init();
	LCD_clr();
	//LCD_sendString(&temp);
	
	showLcdSplash(); // show splash
	showLcdInitial(); // show initial lcd Form
	
	//A498x_setNewMotorMode(&motorStep[0],MOTOR_ENABLE,MOVECW,HALF_STEP,1); // comeca parado
	
	loadDefaultCtrlParameters();

	//if nothing on eeprom load default Parameters
	//if (!loadParamFromEeprom()) loadDefaultCtrlParameters();
	
	//enable interrupts
	sei();
	
	USART1_sendStr("HELLO\n\r");

// 	A498x_setNewMotorMode(&motorStep[0],1,MOVECW,FULL_STEP,1);
// 	while(1){
// 		_delay_ms(10);
// 		A498x_moveMotor(&motorStep[0]);
// 	}
//	uint8_t autocontrol=1;
	calibrateMotorPos(&motorStep[0]); // ele vai ao centro 
	_delay_ms(3000);
	setCenter(&motorStep[0]); //TESTED
	
	//while(1){checkErrorDir(setPoint,readSensors(CHCELL),thSlow,thHigh,sensorInverted);}
		
    while(1)
    {	
// 		if(flagTaskButtons&0x00){	//remove mask
// 		
// 			flagTaskButtons=0;
// 		}
// 			
 		if(flagTaskControl){
 			
 			int dir = checkErrorDir(setPoint,readSensors(CHCELL),thSlow,thHigh,sensorInverted);
 			int mode = decodeStepMode(dir);
// 			#ifdef DEBUG_MAIN
// 			sprintf(bufferDummy,"%d\n\r",dir);
// 			USART1_sendStr(bufferDummy);
// 			#endif
// 			
 			int dirJam=checkLimits(&PINA,6);//will receive -1 as left,1 as right,0 as sensor free,2 as sensor error
 			
 			if(dirJam!=0 ){
				 A498x_setMotorDir(&motorStep[0],STOP_MOTOR);
 				if(dirJam == SENSOR_ERROR);//send error msg to lcd- FILL IN THE MESSAGE
 					 if ( SIGN(dirJam)!=SIGN(dir) ){
							A498x_setMotorDir(&motorStep[0],dir);
 							A498x_setMotorStepMode(&motorStep[0],mode);
 					 }						
 			}
			 else{
 				A498x_setMotorDir(&motorStep[0],dir);
 				A498x_setMotorStepMode(&motorStep[0],mode);
			 }					
			
			flagTaskControl=0;
		}
// 		
// 		
// 		// update motor
		if(flagTaskMotor){	
			// update motor
			A498x_moveMotor(&motorStep[0]);
			flagTaskMotor=0;
		}
// 		
// 		
		if(flagTaskLcd){
			// update lcd here
		
			updateLcdValues(setPoint,readSensors(CHCELL),thSlow,thHigh,sensorInverted);
			
			// reset flag for next run
			flagTaskLcd=0;
		}
     }
}
	
	