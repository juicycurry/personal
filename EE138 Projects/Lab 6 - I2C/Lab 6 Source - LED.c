/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**

 */

#include <asf.h>
#include <clock.h>
#include <conf_clocks.h>

Sercom *I2C_ptr = 0x42000800;

int state=0;

void checkIt(void){
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	porA->OUTTGL.reg = PORT_PA13;
}

void init_ports(void){
	
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);
	
	porA->PMUX[4].bit.PMUXO = 0x2; 
	porA->PINCFG[9].bit.PMUXEN = 0x1; 
	
	porA->PMUX[4].bit.PMUXE = 0x2; 
	porA->PINCFG[8].bit.PMUXEN = 0x1;
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

SERCOM0_Handler(void){

	if(state==3){
		state=0;
	}
	if(state==0){
		//I2C_ptr->I2CM.CTRLB.bit.CMD = 0x00000000;
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr->I2CM.DATA.reg = 0x00;
	//	I2C_ptr->I2CM.INTENCLR.bit.MB = 0x1; //clear the interrupt
	}
	
	if(state==1){
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr->I2CM.DATA.reg = 0xFF;
		//I2C_ptr->I2CM.INTENCLR.bit.MB = 0x1; //clear the interrupt
	}
	
	//if(state==2){
	//I2C_ptr->I2CM.DATA.bit.DATA = 0x1;
	//I2C_ptr->I2CM.INTENCLR.bit.MB = 0x1; //clear the interrupt
	//}
	
	if(state==2){
		while(I2C_ptr -> I2CM.STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY){}
		I2C_ptr->I2CM.CTRLB.bit.CMD = 0x3;
	//	I2C_ptr->I2CM.INTENCLR.bit.MB = 0x1; //clear the interrupt
	}
	//I2C_ptr ->I2CM.INTENCLR.bit.MB = 0x1; //clear the interrupt Master
	//I2C_ptr ->I2CM.INTENCLR.bit.SB = 0x1; //clear the interrupt
	state++;
	
}

int main (void)
{
	system_clock_init();
	init_ports();
	init_clock();
	init_I2C();
	
	I2C_ptr ->I2CM.ADDR.reg = 0x2C<<1;
	while(1){
		
	}
	// Insert application code here, after the board has been initialized.
}
