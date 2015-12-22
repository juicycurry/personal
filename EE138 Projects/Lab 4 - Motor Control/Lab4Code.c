/**
 * Lab 4
 * Colin Chen & Daniel Graham
 *
 **/

#include <clock.h>
#include <conf_clocks.h>

int u0[4] = {0,0,0,0};
int inputStorage[4] = {0,0,0,0};

Tc *TCptr2 = (Tc*)0x42002800UL;
Tc *TCptr4 = (Tc*)0x42003000UL;

Adc *ADC_Ptr = (Adc*)0x42004000; 

Eic *EICptr1 = (Eic*)0x40001800UL;

void enable_port(void);
void enable_tc_clocks(void);
void enable_EIC(void);
void enable_tc(void);

void enable_adc_clocks(void);
void init_adc(void);
unsigned int read_adc(void);

void displayInit(void);
void digit(int position, int displayValue, int decimal_place, PortGroup *porA, PortGroup *porB);
void display(int value, int decimal_place, PortGroup *porA, PortGroup *porB);
void display2(int value, int errorFlag, PortGroup *porA, PortGroup *porB);

void read_keypad(void);
void brake(void);

void zeroEverything(void);

int encodercount,adcvalue;
int digitCounter=0;
int counterCounter=0, counterCounter2, displaycounter;
float rpms, rpm1,rpm2,rpm3;
float workingValue2;
float potvalue,correctedvalue,integrator,rpmerror,rpmvalue,lasterror,lastcorrectedvalue;
float duty1,duty2;
int count1, count2, number1, number2, operation, singedValue1,signedValue2;
int displayvalue;
float kp,ki;
float motorangle,angleerror,finalangleerror,tempangleerror,finalangle,finalspeed;
int mode = 3;	//Display mode
int derivitive;

int programstate=0;  //0 = idle, 2 = speed, 3 = position
int inputStorageCombined=0;
int inputMode = 1;  //Pot
int currentButton = 0;
int column = 1;
int keypress = 0;
int keycheck = 0;
int keystate = 0;
int inputFlag = 0;
int buttonCount = 0;
int calcAng = 0;
int finalAng = 0;
int finalSpe = 0;
int goalState = 0;
int brakeFlag = 0;
int storeRpms = 0;
int storedSpeed = 0;
int signValue = 0;
int storedInput = 0;
int storedAngle = 0;
int displaySign = 0;
int hasRun = 0;
int enterPressed = 0;
float scaledSpeed = 0;
float scaledAngle = 0;
int speedScaler = 0;
int angleScaler = 0;
int signFlag = 0;
int directionCounter = 0;
int hasRun2 = 0;
int adctest = 0;
int desiredspeed = 0;

void enable_port(void)		//setup pins
{
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);
	

	porA->DIRSET.reg = PORT_PA04|PORT_PA05|PORT_PA06|PORT_PA07; //Transistor outputs, ACTIVE LOW	
	porA->DIRCLR.reg = PORT_PA16|PORT_PA17|PORT_PA18|PORT_PA19; //Keypad Inputs
	
	for(int i=16; i<20; i++){		//Configure keypad as input
		porA->PINCFG[i].reg=PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	}
	
	porB -> PMUX[30].bit.PMUXE = 0x5; //sets PB30 slaved to TC4    <---PB30 is the GPIO I2C display Reset?
	porB -> PINCFG[30].bit.PMUXEN = 0x1; //enables this pin
	
	porA->PMUX[14].bit.PMUXE = 0x0; //sets PA28 to the EXTINT[8]
	porA->PINCFG[28].bit.PMUXEN = 0x1; //enables this pin for EIC
	
	porB->PMUX[7].bit.PMUXE = 0x0; //sets PB14 to EXTINT[14]
	porB->PINCFG[14].bit.PMUXEN = 0x1;  //enables this pin for EIC
	
	porA->PMUX[11].bit.PMUXE = 0x5;			//Port 22 - Peripheral Group
	porA->PMUX[11].bit.PMUXO = 0x5;			//Port 23 - Peripheral Group
	porA->PINCFG[22].bit.PMUXEN = 0x1;		//Port 22 - Use PMUXEN
	porA->PINCFG[23].bit.PMUXEN = 0x1;		//Port 23 - Use PMUXEN
}

void enable_tc_clocks(void)
{
	/* Perform Clock configuration to source the TC
	1) ENABLE THE APBC CLOCK FOR THE CORREECT MODULE
	2) WRITE THE PROPER GENERIC CLOCK SELETION ID*/
	
	PM->APBAMASK.reg |=0x1 << 6;        // PM_APBAMASK for the EIC
	PM->APBCMASK.reg |=0x1 << 10;		// PM_APBCMASK for TC2
	PM->APBCMASK.reg |=0x1 << 12;		// PM_APBC MASK for TC4
	
	uint32_t temp=0x14;					// ID for ________ is __________ (see table 14-2)
	temp |= 0<<8;						// Selection Generic clock generator 0
	GCLK->CLKCTRL.reg=temp;				// Setup in the CLKCTRL register
	GCLK->CLKCTRL.reg |= 0x1u << 14;	// enable it.	
	
	temp=0x15;							// ID for ________ is __________ (see table 14-2)
	temp |= 0<<8;						// Selection Generic clock generator 0
	GCLK->CLKCTRL.reg=temp;				// Setup in the CLKCTRL register
	GCLK->CLKCTRL.reg |= 0x1u << 14;	// enable it.
	
	temp=0x03;							// ID for ________ is __________ (see table 14-2)
	temp |= 0<<8;						// Selection Generic clock generator 0
	GCLK->CLKCTRL.reg=temp;				// Setup in the CLKCTRL register
	GCLK->CLKCTRL.reg |= 0x1u << 14;	// enable it.

}

void enable_adc_clocks(void)
{
	struct system_gclk_chan_config gclk_chan_conf;
	
	gclk_chan_conf.source_generator = GCLK_GENERATOR_0;
	system_gclk_chan_set_config(ADC_GCLK_ID , &gclk_chan_conf);
	
	system_gclk_chan_enable(ADC_GCLK_ID );
}

void init_adc(void)
{
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	
	ADC_Ptr -> CTRLA.reg = 0x0;  //adc disabled + reset operation ongoing
	
	porA->DIRCLR.reg = PORT_PA16|PORT_PA17|PORT_PA18|PORT_PA19; //Keypad Inputs
	
	ADC_Ptr -> REFCTRL.reg = 0x2;
	ADC_Ptr -> AVGCTRL.reg = 0x5|0x5 <<4 ;
	ADC_Ptr -> SAMPCTRL.reg =0x1F;
	ADC_Ptr -> CTRLB.reg = ADC_CTRLB_RESSEL_12BIT|ADC_CTRLB_PRESCALER_DIV32;          //(12 bit resolution running with differential mode)
	ADC_Ptr -> INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2|ADC_INPUTCTRL_MUXNEG_GND|ADC_INPUTCTRL_MUXPOS_PIN19 ; //(gain , muxneg, muxpos)
		
	porA -> DIRSET.reg = PORT_PA13;
	porA -> OUTSET.reg = PORT_PA13;
	
	porA -> PMUX[5].bit.PMUXO = 0x1;
	porA -> PINCFG[11].bit.PMUXEN =0x1;
	
	
	ADC_Ptr -> CTRLA.reg = 0x2;  //adc enabled + no reset operation ongoing
}

void enable_EIC(void){	
	enable_port();
	enable_tc_clocks();
	
	EICptr1->CTRL.reg= 0x0; //DISABLES EIC
	EICptr1->EVCTRL.reg=0x1<<8|0x1<<14; //enables EXTINT[8], EXTINT[14]
	EICptr1 ->CONFIG[1].bit.SENSE0 = 0x1; //defines the triggering of EXTINT[8]
	EICptr1 ->CONFIG[1].bit.SENSE6 |= 0x1; //defines the triggering of EXTINT[14]
	EICptr1 ->INTENSET.reg = 0x1 << 8 | 0x1 << 14;	//sets EXTINT[8] as a target for interrupt
	
	while(EICptr1->STATUS.reg & EIC_STATUS_SYNCBUSY) {} //wait for the the EIC to finish sync
	
	EICptr1 ->CTRL.reg = 0x2;  //enable EIC
	
	NVIC_EnableIRQ(EIC_IRQn);
}

void enable_tc(void)
{
	/* Configure the basic timer/counter to have a period of________ or a
	frequency of _________ */
	
	enable_port();

	TCptr2 ->COUNT8.CTRLA.bit.MODE=0x1; //normal frequency operation
	TCptr2 ->COUNT8.CTRLA.bit.PRESCALER=0x6;
	TCptr2 ->COUNT8.CTRLA.bit.PRESCSYNC=0x1;
	TCptr2 ->COUNT8.PER.reg = 90;   //90 period for 0x6 prescaler for 500 Hz
	TCptr2 ->COUNT8.INTENSET.bit.OVF = 0x1;
	
	while(TCptr2->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY) {}
	TCptr2 ->COUNT8.CTRLA.reg |= 0x2;

	TCptr4 ->COUNT8.CTRLA.bit.MODE=0x1; //normal frequency operation
	TCptr4 ->COUNT8.CTRLA.bit.PRESCALER=0x6; 
	TCptr4 ->COUNT8.CTRLA.bit.PRESCSYNC=0x1;
	TCptr4 ->COUNT8.CTRLA.bit.WAVEGEN=0x2;
	TCptr4 ->COUNT8.CC[1].reg = 87;
	TCptr4 ->COUNT8.CC[0].reg = 87;
	TCptr4 ->COUNT8.PER.reg = 174;   //174 period for 0x6 prescaler for 500 Hz
	TCptr4 ->COUNT8.INTENSET.bit.OVF = 0x1;
	
	while(TCptr4->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY) {}
	TCptr4 ->COUNT8.CTRLA.reg |= 0x2;
	
	//TCptr4 ->COUNT16.CTRLA.bit.MODE=0x0; //normal frequency operation
	//TCptr4 ->COUNT16.CTRLA.bit.PRESCALER=0x0;
	//TCptr4 ->COUNT16.CTRLA.bit.PRESCSYNC=0x1;
	//TCptr4 ->COUNT16.CTRLA.bit.WAVEGEN=0x2;
	//TCptr4 ->COUNT16.CC[1].reg = 31000;
	//TCptr4 ->COUNT16.CC[0].reg = 34000;
	//TCptr4 ->COUNT16.INTENSET.bit.OVF = 0x1;

	//while(TCptr4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY) {}
	//TCptr4 ->COUNT16.CTRLA.reg |= 0x2;
	
	NVIC_EnableIRQ(TC2_IRQn);
	NVIC_EnableIRQ(TC4_IRQn);
	NVIC -> IP[3] = 0xC0000000;
	NVIC -> IP[4] = 0x00008000;
}

unsigned int read_adc(void)
{
	// start the conversion
	ADC_Ptr -> SWTRIG.reg = 0x2; // starts adc conversion but does not flush pipeline
	
	while(!(ADC_Ptr->INTFLAG.bit.RESRDY));  //wait for conversion to be available
	
	return(ADC_Ptr-> RESULT.reg ); //insert register where ADC store value
}

void EIC_Handler( void ){
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);

	if(EICptr1 ->INTFLAG.reg & (0x1 <<8)){ //A
		if(porB->IN.reg & PORT_PB14 ){
			encodercount++;
			
		}
		if(!(porB->IN.reg & PORT_PB14)){
			encodercount--;
			
		}
	}
		
	if(EICptr1 ->INTFLAG.reg & (0x1 <<14)){ //B
		if(porA->IN.reg & PORT_PA28){
			encodercount--;
		}
		if(!(porA->IN.reg & PORT_PA28)){
			encodercount++;
		}
		
	}
	
	EICptr1->INTFLAG.reg = 0x1 << 14|0x1 << 8;
	NVIC->IP[1] = 0x00000040;
}

void brake(){
	if(inputMode==1){
		storedInput=1;
	}
	inputMode = 0;
	storedSpeed = finalspeed;
	storeRpms = rpms;
	
	if(finalspeed>87){
		storedSpeed--;
	}
	if(finalspeed<87){
		storedSpeed++;
	}
	
	if(storeRpms==0){
		brakeFlag = 0;
		programstate = 3;
		inputMode = storedInput;
		encodercount = 0;
	}
}

void zeroEverything(){
	encodercount = 0;
}

void TC2_Handler( void ){
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);
	
	TCptr2->COUNT8.INTFLAG.bit.OVF = 0x1; //clears the interrupt flag
	
	read_keypad();
	
	//State machine Part 1
	
	if(inputMode == 0 && !enterPressed){			//Handle Keyboard Input Mode
		
		display(inputStorageCombined,1,porA,porB);
		
		if(hasRun == 0){
			hasRun++;
			storedSpeed = potvalue;
			storedAngle = potvalue;
		}
		if(currentButton == 30){		//Toggle sign for keypad input
			signFlag = !signFlag;
		}
		if(currentButton == 22){		//Enter for keypad input, set converted speeds and angles, clear input buffer
			enterPressed = 1;
			storedSpeed = inputStorageCombined;
			storedAngle = inputStorageCombined;
			hasRun = 0;
			inputStorage[0] = inputStorage[1] = inputStorage[2] = inputStorage[3] = 0;
		}	
	}
	
	if(programstate == 3){				//If position mode
		if(hasRun2 == 0){				//If it hasn't been set 'home'
			if(currentButton == 30){	//If # button, rotate right until enter
				duty1 = 92;
				duty2 = 82;
				hasRun2 = 1;			//Continue endlessly, could put error condition thing
			}
			if(currentButton == 32){	//If * button, rotate left until enter
				duty1 = 82;
				duty2 = 92;
				hasRun2 = 1;
			}
		}
		if(hasRun2 == 1){				//If spinning, wait for enter (D) press
			if(currentButton == 22){
				hasRun2 = 2;
				zeroEverything();		//Reset counters to set 'home' to current position
			}
		}
	}
	
	if(currentButton<10){		//Continue Keyboard input after first input
		enterPressed = 0;
	}
	
	if(currentButton == 26){			//Toggle the Display Mode
		mode = mode + 1;
		if(mode>3){
			mode = 1;
		}
	}
	
	if(currentButton == 24){			//Toggle Output mode, brake if spinning
		if(programstate == 0){			//Idle State
			programstate = 2;
		}
		else if(programstate == 2){			//Speed State
			brakeFlag = 1;
		}
		else if(programstate == 3){			//Position State
			programstate = 0;
		}
		hasRun2 = 0;
		inputStorage[0] = inputStorage[1] = inputStorage[2] = inputStorage[3] = 0; //Wipe input Buffer
	}
	
	if(currentButton == 28){			//Toggle input mode between pot and keyboard
		inputMode = !inputMode;
		hasRun = 0;
		hasRun2 = 0;
		enterPressed = 0;
		inputStorage[0] = inputStorage[1] = inputStorage[2] = inputStorage[3] = 0; 
	}
	
	if(inputMode == 1 || (inputMode == 0 && enterPressed)){
		if(programstate == 0){
			display2(10,0,porA,porB);
		}
		if(programstate == 2){				//Display Speed Information if Speed State
			if(mode == 1){
				display(rpms,1,porA,porB);
			}
			if(mode == 2){
				if(inputMode == 1){
					display(desiredspeed,1,porA,porB);
				}
				if(inputMode == 0){
					display(storedSpeed,1,porA,porB); //storedSpeed
				}
			}
			if(mode == 3){
				display2(9,0,porA,porB);
			}
		}
	
		if(programstate == 3){				//Display Position Information if Position State
			if(mode == 1){
				display((motorangle-360),1,porA,porB);
			}
			if(mode == 2){
				if(inputMode == 1){
					display((finalangle-360),1,porA,porB);
				}
				if(inputMode == 0){
					display((finalangle-360),1,porA,porB);
				}
			}
			if(mode == 3){
				display2(8,0,porA,porB);
			}
		}
	}
	//End State Machine Part 1
}

void TC4_Handler(void){		//State Machine Handler

/////////////////////////////////
//	State 2: Speed Control  //
/////////////////////////////////

	if(programstate == 0){
		duty1 = 87;
		duty2 = 87;
	}

	if(programstate == 2){  //things to display are RPM variable is called "rpms", rpm error is called "rpmerror",

		adcvalue = read_adc();
		adcvalue = -adcvalue + 4096;
		potvalue = ((adcvalue)*174)>>12;
		
		//finalSpe = -(finalspeed)*87/5000+174;
		
		if(inputMode == 1){
			finalspeed = potvalue;	//store whatever into finalspeed if you want it to be the final value, for now pot value is stored in it,
								//create states so that either potvalue or final speed stores values into the final speed state.
			desiredspeed = -2*(potvalue*4750/174)+4700;
		}
		if(inputMode == 0){
			
			finalspeed = -(storedSpeed*87/4975)+87; //Convert inputStorage to integer
		}
		
		rpmvalue=-(rpms*87/5000)+87;
		
		rpmerror = (finalspeed-rpmvalue);
		
		if(integrator > 40){ //limiting values for integrators so that they do not ramp too high
			integrator = 40;
		}
		if(integrator < -40){
			integrator = -40;
		}

		kp = 0.01;
		ki = 0.01;
		
		if((abs(rpmerror) < 17.4) && (abs(rpms) < 1000)){
			kp = 0.001;
			
		}
		
		
		derivitive = rpmerror - lasterror;
		correctedvalue = lastcorrectedvalue + kp*rpmerror+ ki*integrator - 0.0012*(rpmerror - lasterror)/.0055999 ;  //PID equation for speed control
		lastcorrectedvalue = correctedvalue;
		integrator=integrator+0.5*kp*rpmerror; //scaling integrator
		
		
		if(correctedvalue > 173){ //PWM limits so that the motor does not recieve too high or too low values
			correctedvalue = 173;
		}
		
		if(correctedvalue < 1){
			correctedvalue=1;
		}
		
		if((79 < correctedvalue)&&(correctedvalue < 95)){
			correctedvalue = 87;
		}
		
		duty1 = -correctedvalue+174; //potvalue*174/1000+174;
		duty2 = -duty1+174;
		
		
		workingValue2 = (((encodercount)/0.005599)/350)*(60);
		rpm3 = rpm2;
		rpm2 = rpm1;
		rpm1 = workingValue2;
		rpms = (rpm1+rpm2+rpm3)/3; //rpm averaging as it was very jumpy throughout testing
		encodercount=0;
		lasterror = rpmerror; //saving the error to be used in the derivitive portion of the controller
	}

/////////////////////////////////
//	State 3: Position Control  //
/////////////////////////////////
	
	if(programstate == 3 && hasRun2 == 2){
		adcvalue = read_adc();
		
		potvalue = (adcvalue*720)>>12;
		
		if(inputMode == 1){
			finalangle = potvalue; // final angle will be the value that you plug in the
				if(potvalue<=360){
					finalAng = -(360-(potvalue));
					//calcAng =  -(360-(encodercount*360/400)); //*1.16129
				}
				if(potvalue>360){
					finalAng = ((potvalue-360)*1.16129);
					//calcAng = (((encodercount*360/400)-360));
				}
				
		}
		if(inputMode == 0){
			if(storedAngle > 360){
				storedAngle = 360;
				//calcAng =  -(360-(encodercount*360/425)*1.16129);
			}
			if(storedAngle < -360){
				storedAngle = -360;
				//calcAng = (((encodercount*384/425)-360)*1.16129);
			}
			finalangle = storedAngle;//calculated from inputStorage
		}
		
	
		motorangle = encodercount*360/400; // 360 degrees per 426 count, (changed from 400 count to account for some inaccuracy)
		angleerror = motorangle - finalangle;

		correctedvalue = abs(angleerror)*kp;
		integrator = integrator + .5*correctedvalue;
		derivitive = 0.007*((angleerror - lasterror)/0.005599);
		
		if(integrator>500){
			integrator = 500;
		}
		
		if(derivitive > 170){
			derivitive = 170;
		}
		
		if(derivitive < -170){
			derivitive = -170;
		}
		
		//if(abs(angleerror)< 50){
		//if(integrator > 200){
		//integrator=200;
		//}
		//ki = 0.011;
		//kp = 0.005;
		//}
		//if(abs(angleerror)>=50){
			
		ki = 0.1;
		kp = 0.4;
		
		//}
		
		if(angleerror > 2){
			duty1 = 87-correctedvalue - ki*integrator*0 + derivitive;
			duty2 = -duty1+174;
		}
		if(angleerror < -2){
			duty1 = 87+correctedvalue+ ki*integrator*0 - derivitive;
			duty2 = -duty1+174;
		}
		if((angleerror<=1)&&(angleerror>=-1)){
			duty1 = 87;
			duty2 = 87;
		}
		lasterror = angleerror;
	}
	
	//End State Machine Handler
	
	if(duty1 > 174){
		duty1 = 174;
		duty2 = 0;
	}
	
	if(duty1 < 0){
		duty1 = 0;
		duty2 = 174;
	}
	
	if(brakeFlag){
		brake();
	}
	
	//scaledSpeed = storedSpeed*87/4900+87;
	//scaledAngle = ;
	
	TCptr4 ->COUNT8.CC[1].reg = duty1;
	TCptr4 ->COUNT8.CC[0].reg = duty2;

	TCptr4->COUNT8.INTFLAG.bit.OVF = 0x1; //clears the interrupt flag
}

void read_keypad(void){
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	
	currentButton = 50;
	
	if((porA->IN.reg & PORT_PA16)){    //A,B,C,D
		switch(column){
			case 1:					//Change Input Mode
			keypress = 28;
			break;
			
			case 2:
			keypress = 24;        //Change Output Mode
			break;
			
			case 3:						//Change Display Mode
			keypress = 26;
			break;
			
			case 4:						// Press Enter
			keypress = 22;
			break;
			
			default:
			break;
		}
	}
	
	if((porA->IN.reg & PORT_PA17)){         //3,6,9,#
		
		switch(column){
			case 1:		//#3
			keypress=3;
			break;
			
			case 2:		//#6
			keypress=6;
			break;
			
			case 3:		//#9
			keypress=9;
			break;
			
			case 4:		//# Negative for keypad input
			keypress=30;
			break;
		}
		
	}
	
	if((porA->IN.reg & PORT_PA18)){         //2,5,8,0
		switch(column){
			case 1:
			keypress=2;
			break;
			
			case 2:
			keypress=5;
			break;
			
			case 3:
			keypress=8;
			break;
			
			case 4:
			keypress=0;
			break;
		}
	}
	
	if((porA->IN.reg & PORT_PA19)){ //1,4,7,*
		switch(column){
			case 1:
			keypress=1;
			break;
			
			case 2:
			keypress=4;
			break;
			
			case 3:
			keypress=7;
			break;
			
			case 4:
			keypress=32;
			break;
		}
	}
	
	
	if (keystate==0){                                  //Begin state machine
		if((porA->IN.reg&(PORT_PA16|PORT_PA17|PORT_PA18|PORT_PA19))){
			keycheck=keypress;
			keystate=1;
			count2=0;
			count1=0;
		}
	}
	
	if(keystate==1){
		
		if(keycheck==keypress){                       //Debounce code
			count2++;
			if(count2>15){ //thing is pressed
				keystate=2;
				count2=0;
			}
		}
		
		if(!(keycheck==keypress)){
			keystate=0;
		}
	}
	
	if(keystate==2){           //End of debounce, check for lack of press
		if(!(porA->IN.reg&(PORT_PA16|PORT_PA17|PORT_PA18|PORT_PA19))){
			count2++;
			if(count2>15){
				count2=0;
				keystate=0;
				inputFlag=1;
				
				currentButton=keycheck;
				if(keycheck < 10){
					inputStorage[0]=inputStorage[1];
					inputStorage[1]=inputStorage[2];
					inputStorage[2]=inputStorage[3];
					inputStorage[3]=keycheck;
				}
			}
		}
		if((porA->IN.reg&(PORT_PA16|PORT_PA17|PORT_PA18|PORT_PA19))){
			count2=0;
		}
		
	}

	inputStorageCombined = inputStorage[0]*1000 + inputStorage[1]*100 + inputStorage[2]*10 + inputStorage[3];
	if(signFlag == 1){
		inputStorageCombined = -inputStorageCombined;
	}
	
	if(programstate == 2){					//Set upper limit for RPM (4500 in either direction)
		if(inputStorageCombined > 4800){
			inputStorageCombined = 4800;
		}
	}
	if(programstate == 3){
		if(inputStorageCombined > 360){	//Set upper limit for Rotations (20 Rotations in a direction)
			inputStorageCombined = 360;
		}
	}
	
	column++;
	if(column>4){
		column = 1;
	}
	
}

void displayInit(void){
	
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);
	
	porA->DIRSET.reg = PORT_PA02|PORT_PB30|PORT_PA13;
	porA->OUTSET.reg = PORT_PA02|PORT_PB30|PORT_PA13;
	
	porA->DIRSET.reg = PORT_PA04|PORT_PA05|PORT_PA06|PORT_PA07; //Transistor outputs, ACTIVE LOW
	porB->DIRSET.reg = PORT_PB00|PORT_PB01|PORT_PB02|PORT_PB03|PORT_PB04|PORT_PB05|PORT_PB06|PORT_PB07|PORT_PB09; //7 Segment Display Pins, ACTIVE LOW
	
	porB->OUTSET.reg = PORT_PB00|PORT_PB01|PORT_PB02|PORT_PB03|PORT_PB04|PORT_PB05|PORT_PB06|PORT_PB07|PORT_PB09; //Reseting Pins to all 0
	porA->OUTSET.reg = PORT_PA04|PORT_PA05|PORT_PA06|PORT_PA07;
	
	for(int i=0; i<7; i++){                 //Toggle drive strength high for LED output
		porB->PINCFG[i].reg=PORT_PINCFG_DRVSTR;
	}
	porB->PINCFG[9].reg=PORT_PINCFG_DRVSTR;
	for(int i=16; i<20; i++){               //Configure keypad as input
		porA->PINCFG[i].reg=PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	}
}

void digit(int position, int displayValue, int decimal_place, PortGroup *porA, PortGroup *porB){                //Digit function: Scans for input, displays values, and performs mathematic operations
	porA->OUTSET.reg = PORT_PA04|PORT_PA05|PORT_PA06|PORT_PA07;             //Reset all used ports
	porB->OUTSET.reg = PORT_PA00|PORT_PB01|PORT_PB02|PORT_PB03|PORT_PB04|PORT_PB05|PORT_PB06|PORT_PB07|PORT_PB09;
	
	switch(position){							//Determine which digit to illuminate based on the passed "position" value
		case 1:
		porA->OUTCLR.reg = PORT_PA07;
		column=1;
		break;
		
		case 2:
		porA->OUTCLR.reg = PORT_PA06;
		column=2;
		break;
		
		case 3:
		porA->OUTCLR.reg = PORT_PA05;
		column=3;
		break;
		
		case 4:
		porA->OUTCLR.reg = PORT_PA04;
		column = 4;
		break;
	}
	
	switch(displayValue){                   //Displays numeric values 0-9, case '11' is blank
		case 1:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB01 | PORT_PB02 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB01 | PORT_PB02;
		break;
		
		case 2:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB03 | PORT_PB04 | PORT_PB06 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB03 | PORT_PB04 | PORT_PB06;
		break;
		
		case 3:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB06 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB06;
		break;
		
		case 4:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB01 | PORT_PB02 | PORT_PB05 | PORT_PB06 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB01 | PORT_PB02 | PORT_PB05 | PORT_PB06;
		break;
		
		case 5:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 | PORT_PB05 | PORT_PB06 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 | PORT_PB05 | PORT_PB06;
		break;
		
		case 6:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06;
		break;
		
		case 7:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02;
		break;
		
		case 8:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06;
		break;
		
		case 9:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB05 | PORT_PB06 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB05 | PORT_PB06;
		break;
		
		case 0:
		if (decimal_place == 1){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB09;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05;
		break;
		
		case 12: //E
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06;
		break;
		
		case 13: //r
		porB->OUTCLR.reg = PORT_PB04 | PORT_PB06;
		break;
		
		case 14: //o
		porB->OUTCLR.reg = PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB06;
		break;
		
		case 15: //C
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB03 | PORT_PB04 | PORT_PB05;
		break;
		
		case 16: //A
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB04 | PORT_PB05 | PORT_PB06;
		break;
		
		case 17: //L
		porB->OUTCLR.reg = PORT_PB03 | PORT_PB04 | PORT_PB05;
		break;
		
		case 18: //P
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB04 | PORT_PB05 | PORT_PB06;
		break;
		
		case 19: //V
		porB->OUTCLR.reg = PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05;
		break;
		
		case 20: //d
		porB->OUTCLR.reg = PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 |  PORT_PB06;
		break;
		
		case 21: //L
		porB->OUTCLR.reg = PORT_PB03 | PORT_PB04 | PORT_PB05;
		break;
		
	}	
}

void display(int value, int decimal_place, PortGroup *porA, PortGroup *porB){
	
	int workingValue = abs(value);
	signValue = 0;
	
	if(value<0){
		signValue = 1;
	}
	
	if(displaycounter>50){
	
	u0[0] = workingValue / 1000;
	u0[1] = (workingValue - (u0[0]*1000)) / 100;
	u0[2] = (workingValue - (u0[0]*1000) - (u0[1]*100)) / 10;
	u0[3] = (workingValue - (u0[0]*1000) - (u0[1]*100) - (u0[2]*10));
	displaycounter = 0;
	}
	displaycounter++;
	digitCounter++;
	if(digitCounter>=4){
		digitCounter=0;
	}
	
	digit(digitCounter+1,u0[digitCounter],signValue,porA,porB); //slightly modified display code, I changed the display function so that it will only do the calculations
	
}

void display2(int value, int errorFlag, PortGroup *porA, PortGroup *porB){
	
	signValue = 0;
	if(errorFlag){
		u0[0] = 12;
		u0[1] = 13;
		u0[2] = 13;
		u0[3] = value;
	}
	
	if(!errorFlag){
		switch(value){
			case 6:	u0[0] = 18;	//Pro
			u0[1] = 13;
			u0[2] = 14;
			u0[3] = 11;
			break;
			
			case 7:	u0[0] = 15;	//CALC
			u0[1] = 16;
			u0[2] = 17;
			u0[3] = 15;
			break;
			
			case 8:	u0[0] = 18; //POS
			u0[1] = 0;
			u0[2] = 5;
			u0[3] = 11;
			break;
			
			case 9:	u0[0] = 5;	//SPE
			u0[1] = 18;
			u0[2] = 12;
			u0[3] = 11;
			break;
			
			case 10: u0[0] = 1; //idle
			u0[1] = 20;
			u0[2] = 21;
			u0[3] = 12;
			break;
		}
	}
	
	digitCounter++;
	if(digitCounter>=4){
		digitCounter=0;
	}
	
	digit(digitCounter+1,u0[digitCounter],signValue,porA,porB); //slightly modified display code, I changed the display function so that it will only do the calculations
}

int main (void)
{
	displayInit();
	system_clock_init();
	enable_EIC();
	enable_tc();
	enable_adc_clocks();
	init_adc();
	
	while(1){
		
	}
}