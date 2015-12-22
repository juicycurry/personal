#include <asf.h>

void wait(int t);
void displayInit(PortGroup *porA, PortGroup *porB);
void digit(int position, int displayValue, int decimal_place, PortGroup *porA,PortGroup *porB);
void display(int value, int decimal_place, PortGroup *porA, PortGroup *porB);

/* initialize ADC pointer here */ // define a pointer to the ADCblock
Adc *ADC_Ptr = (Adc *)0x42004000UL;

void enable_adc_clocks(void);
void init_adc(void);
unsigned int read_adc(void);

// set up generic clock for ADC
void enable_adc_clocks(void)
{
	struct system_gclk_chan_config gclk_chan_conf;
	
	gclk_chan_conf.source_generator = GCLK_GENERATOR_0;
	system_gclk_chan_set_config(ADC_GCLK_ID , &gclk_chan_conf);
	
	//Enable the generic clock for ADC
	system_gclk_chan_enable(ADC_GCLK_ID );
}

// initialize the on-board ADC system
void init_adc(void)
{
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	ADC_Ptr -> CTRLA.reg = 0x0; //adc disabled + reset operation ongoing
	
	ADC_Ptr -> REFCTRL.reg = 0x2;
	ADC_Ptr -> AVGCTRL.reg = 0x0;
	ADC_Ptr -> SAMPCTRL.reg =ADC_AVGCTRL_SAMPLENUM_1_Val;
	ADC_Ptr -> CTRLB.reg = ADC_CTRLB_RESSEL_12BIT|ADC_CTRLB_PRESCALER_DIV32;
	//(12 bit resolution running with differential mode)
	ADC_Ptr -> INPUTCTRL.reg =
	ADC_INPUTCTRL_GAIN_DIV2|ADC_INPUTCTRL_MUXNEG_GND|ADC_INPUTCTRL_MUXPOS_PIN0 ; //(gain ,muxneg, muxpos)

// config PA02 to be owned by ADC Peripheral

	porA -> DIRSET.reg = PORT_PA13;
	porA -> OUTSET.reg = PORT_PA13;
	
	porA -> PMUX[1].bit.PMUXE = 0x1;
	porA -> PINCFG[2].bit.PMUXEN =0x1;
	
	
	ADC_Ptr -> CTRLA.reg = 0x2; //adc enabled + no reset operation ongoing
	
}

unsigned int read_adc(void)
{
	
	 // start the conversion
	ADC_Ptr -> SWTRIG.reg = 0x2; // starts adc conversion but does not flushpipeline

	while(!(ADC_Ptr->INTFLAG.bit.RESRDY)); //wait for conversion to be available
	
	return(ADC_Ptr-> RESULT.reg ); //insert register where ADC store value

}


void wait(int t) //Wait function: Simple wait function
{
	int count = 0;
	while (count < t)
	{
		count++;
	}
}

oid displayInit(PortGroup *porA, PortGroup *porB){
	porA->DIRSET.reg = PORT_PA04|PORT_PA05|PORT_PA06|PORT_PA07; //Transistor outputs, ACTIVE LOW
	porB->DIRSET.reg = PORT_PB00|PORT_PB01|PORT_PB02|PORT_PB03|PORT_PB04|PORT_PB05|PORT_PB06|PORT_PB07|PORT_PB09; //7 Segment Display Pins, ACTIVE LOW

	porB->OUTSET.reg = PORT_PB00|PORT_PB01|PORT_PB02|PORT_PB03|PORT_PB04|PORT_PB05|PORT_PB06|PORT_PB07|PORT_PB09; //Reseting Pins to all 0
	porA->OUTSET.reg = PORT_PA04|PORT_PA05|PORT_PA06|PORT_PA07;
	
	for(int i=0; i<7; i++){ //Toggle drive strength high for LED output
		porB->PINCFG[i].reg=PORT_PINCFG_DRVSTR;
	}
	for(int i=16; i<20; i++){ //Configure keypad as input
		porA->PINCFG[i].reg=PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	}
}

void digit(int position, int displayValue, int decimal_place, PortGroup *porA,
PortGroup *porB){ //Digit function: Scans for input, displays values,and performs mathematic operations
	porA->OUTSET.reg = PORT_PA04|PORT_PA05|PORT_PA06|PORT_PA07;
	//Reset all used ports
	porB->OUTSET.reg =PORT_PA00|PORT_PB01|PORT_PB02|PORT_PB03|PORT_PB04|PORT_PB05|PORT_PB06|PORT_PB07|PORT_PB09;
	switch(position){
		//Determine which digit to illuminate based on the passed "position" value
		case 1:
		porA->OUTCLR.reg = PORT_PA07;
		break;
		
		case 2:
		porA->OUTCLR.reg = PORT_PA06;
		break;
		
		case 3:
		porA->OUTCLR.reg = PORT_PA05;
		break;
		
		case 4:
		porA->OUTCLR.reg = PORT_PA04;
		break;
	}
	
	switch(displayValue){ //Displays numeric values 0-9, case '11' is blank
		case 1:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB01 | PORT_PB02 | PORT_PB07;
		}
		else
		porB->OUTCLR.reg = PORT_PB01 | PORT_PB02;
		break;
		
		case 2:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB03 |PORT_PB04 | PORT_PB06 | PORT_PB07;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB03 | PORT_PB04 |PORT_PB06;
		break;
		
		case 3:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB06 | PORT_PB07;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB06;
		break;
		
		case 4:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB01 | PORT_PB02 | PORT_PB05 | PORT_PB06 | PORT_PB07;
		}
		else
			porB->OUTCLR.reg = PORT_PB01 | PORT_PB02 | PORT_PB05 | PORT_PB06;
		break;
		
		case 5:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 | PORT_PB05 | PORT_PB06 | PORT_PB07;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 | PORT_PB05 | PORT_PB06;
		break;
		
		case 6:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 |PORT_PB04 | PORT_PB05 | PORT_PB06 | PORT_PB07;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06;
		break;
		
		case 7:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB07;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02;
		break;
		
		case 8:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06 | PORT_PB07;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB06;
		break;
		
		case 9:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB05 | PORT_PB06 | PORT_PB07;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB05 | PORT_PB06;
		break;
	
		case 0:
		if (decimal_place == position){
			porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05 | PORT_PB07;
		}
		else
		porB->OUTCLR.reg = PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03 | PORT_PB04 | PORT_PB05;
		break;
	}
	
	wait(500);
}

void display(int value, int decimal_place, PortGroup *porA, PortGroup *porB){
	
	int u0,u1,u2,u3;
	
	u0 = value / 1000;
	digit(1,u0,1,porA,porB);
	u1 = (value - (u0*1000)) / 100;
	digit(2,u1,1,porA,porB);
	u2 = (value - (u0*1000) - (u1*100)) / 10;
	u3 = (value - (u0*1000) - (u1*100) - (u2*10));
	
	digit(4,u3,1,porA,porB);
	digit(3,u2,1,porA,porB);
	
}

int main (void)
{
	Port *ports = PORT_INSTS;
	PortGroup *porA = &(ports->Group[0]);
	PortGroup *porB = &(ports->Group[1]);

	system_clock_init();
	enable_adc_clocks();
	init_adc();
	displayInit(porA,porB);
	
	int g=0;
	int x;
	int y;
	int z=0;
	int timer1=0;
	
	while(1)
	{
		x = read_adc(); //store variable from ADC into variable "x"
		
		y = x;
		z = abs(y-x);
		if(z>20||timer1>1000){
			x = x*3285;
			g = x>>12;
			timer1=0;
		}
		
		display(g,1,porA,porB); //(integer value, decimal place, portgroup, port group);
		timer1++;
	 }
}
