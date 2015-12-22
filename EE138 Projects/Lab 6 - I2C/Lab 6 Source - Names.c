#include <asf.h>
#include <clock.h>
#include <conf_clocks.h>

Sercom *I2C_ptr = 0x42000800;
						//D    A     N    I    E   L    Sp   G     R   A   H     A    M   Sp   Sp   Sp   Sp
int global_array[50] = {0x44,0x61,0x6E,0x69,0x65,0x6C,0x02,0x47,0x72,0x61,0x68,0x61,0x6D,0x02,0x02,0x02,0x02,
						0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,
						0x02,0x02,0x02,0x02,0x02,0x02,0x43,0x6F,0x6C,0x69,0x6E,0x02,0x43,0x68,0x65,0x6E};
						//								C    o    l    i   n    sp   C     h    e    n
volatile int state=0;
volatile int init_state=0;
volatile int global_state = 0;
volatile bool hasRun = false;
void checkIt(void){
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	porA->OUTTGL.reg = PORT_PA13;
}

void init_ports(void){
	
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);
	
	porA->PMUX[4].bit.PMUXO = 0x2; 		//Set output pin 9 (odd) to use group 2
	porA->PINCFG[9].bit.PMUXEN = 0x1; 	//Use I2C
	
	porA->PMUX[4].bit.PMUXE = 0x2; 		//Set output pin 8 (even) to use group 2
	porA->PINCFG[8].bit.PMUXEN = 0x1;	//Use I2C
	
	porB->DIRSET.reg = PORT_PB30;
	porA->DIRSET.reg = PORT_PA13;
	porB->OUTSET.reg = PORT_PB30;
}

void init_clock(void){
	PM->APBCMASK.reg |=0x1 << 2;
	uint32_t temp=0x0D;					// ID for ________ is __________ (see table 14-2)
	temp |= 0<<8;						// Selection Generic clock generator 0
	GCLK->CLKCTRL.reg=temp;				// Setup in the CLKCTRL register
	GCLK->CLKCTRL.reg |= 0x1u << 14;	// enable it.
}
void init_I2C(void){
	
	I2C_ptr ->I2CM.CTRLA.reg = 0x0 <<1;
	I2C_ptr ->I2CM.CTRLA.reg |= 0x05 << 2;
	I2C_ptr ->I2CM.BAUD.bit.BAUD = 100;
	I2C_ptr ->I2CM.CTRLA.bit.SDAHOLD = 0x2;
//	I2C_ptr ->I2CM.CTRLB.bit.CMD = 0x2;
	I2C_ptr ->I2CM.INTENSET.bit.MB = 0x1;
	I2C_ptr ->I2CM.CTRLB.bit.SMEN = 0x1;
	
	
	
	while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
	I2C_ptr ->I2CM.CTRLA.reg |= 0x1 << 1;
	
	while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
	I2C_ptr ->I2CM.STATUS.bit.BUSSTATE = 0x1;
	
	NVIC_EnableIRQ(SERCOM0_IRQn);
}

void init_LCD(){
	
	
	switch(init_state){
		
		case 1:
			I2C_ptr ->I2CM.DATA.reg = 0x00;
		break;
		
		case 2:
			while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
			I2C_ptr ->I2CM.DATA.reg = 0x38;
		break;
		
		case 3:
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr ->I2CM.DATA.reg = 0x39;
		break;
		case 4:
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr ->I2CM.DATA.reg = 0x14;
		break;
		case 5:
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr ->I2CM.DATA.reg = 0x78;
		break;
		case 6:
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr ->I2CM.DATA.reg = 0x5E;
		break;
		case 7:
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr ->I2CM.DATA.reg = 0x6D;
		break;
		case 8:
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr ->I2CM.DATA.reg = 0x0F;
		break;
		case 9:
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr ->I2CM.DATA.reg = 0x01;
		break;
		case 10:
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr ->I2CM.DATA.reg = 0x06;
		break;
		case 11:
		I2C_ptr ->I2CM.CTRLB.bit.CMD = 0x03;
		hasRun = true;
		break;
	}
	init_state++;
	
	
	
	
}


SERCOM0_Handler(void){
   /*******************
   LCD DISPLAY INITIALIZATION
   ********************/
   
	if(global_state == 0){
		init_LCD();
	}
	
	
	if(global_state == 1){
		if(state ==0){
			while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
			I2C_ptr ->I2CM.DATA.reg = 0x40;
			state++;
		}
		if(state<=51){
   			while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
   			I2C_ptr ->I2CM.DATA.reg = global_array[state-2];
			//   I2C_ptr ->I2CM.DATA.reg = 0x41;
			
		}

		if(state==52){
			I2C_ptr ->I2CM.CTRLB.bit.CMD = 0x03;
		}
   		state++;
		  
		   
	}
	

	
}

int main (void)
{
	system_clock_init();
	init_ports();
	init_clock();
	init_I2C();
	init_state = 0;
	global_state = 0;
	I2C_ptr ->I2CM.ADDR.reg = 0x3C<<1;
	while(!hasRun){};
	global_state++;
	I2C_ptr ->I2CM.ADDR.reg = 0x3C<<1;
	checkIt();
	while(1){
		
	}
	// Insert application code here, after the board has been initialized.
}
