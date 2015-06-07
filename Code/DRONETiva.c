//*****************************************************************************
//
// interrupts.c - Interrupt preemption and tail-chaining example.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>


#include "driverlib/adc.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "driverlib/i2c.h"
//#include "inc/tm4c1294ncpdt.h";
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Interrupts (interrupts)</h1>
//!
//! This example application demonstrates the interrupt preemption and
//! tail-chaining capabilities of Cortex-M4 microprocessor and NVIC.  Nested
//! interrupts are synthesized when the interrupts have the same priority,
//! increasing priorities, and decreasing priorities.  With increasing
//! priorities, preemption will occur; in the other two cases tail-chaining
//! will occur.  The currently pending interrupts and the currently executing
//! interrupt will be displayed on the UART; GPIO pins B3, L1 and L0 (the
//! GPIO on jumper J27 on the left edge of the board) will be asserted upon
//! interrupt handler entry and de-asserted before interrupt handler exit so
//! that the off-to-on time can be observed with a scope or logic analyzer to
//! see the speed of tail-chaining (for the two cases where tail-chaining is
//! occurring).
//
//*****************************************************************************

void writeData(int slave, int sub, int data);
int readData(int slave, int reg);
void calcDate(uint32_t date[4]);
void sendData(void);
void tempWindSensor(void);
//void gyroSensor(void);
void compassSensor(void);
int combine(int a, int b, int c);

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
const int size = 10;
uint32_t g_ui32SysClock;
uint32_t GYRO_ADDRESS = 105;
uint32_t COMPASS_ADDRESS = 30;
uint32_t ACCEL_ADDRESS = 24;
//time
uint32_t count;


//xbee key -- Python script saves each to a file based on the first byte, as follows:
#define T 1;
#define W 2;
#define xG 3;
#define yG 4;
#define zG 5;
#define xA 6;
#define yA 7;
#define zA 8;

//buffers


//16 bit precision
short temp[10];//mvolts
short wind[10];//mvolts
short      xGyro[10];
short      yGyro[10];
short      zGyro[10];
short      xGauss[10];
short      yGauss[10];
short      zGauss[10];
short      xAccel[10];
short      yAccel[10];
short      zAccel[10];
//uint32_t light[100];//Hz

//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{

}
#endif



//*****************************************************************************
//
// Configure the UART and its pins. This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, g_ui32SysClock);
}



void
calcDate(uint32_t date[4]) {
	int seconds = count/100;
    int minutes = seconds / 60;
    int hours = minutes / 60;
    int days = hours / 24;

    date[0] = seconds % 60;
	date[1] = minutes % 60;
	date[2] = hours  % 24;
	date[3] = days;

}

void sendData(void) {
	uint32_t date[4];
	calcDate(date);



	uint32_t i;
	uint32_t x;

	int delay = 25;
//UARTwrite(char pointer, length)
	char holder = T;

	UARTwrite(&holder, 1);
	UARTwrite(temp, size * 2);

	ROM_SysCtlDelay(delay*40000);
    holder = W;
	UARTwrite(&holder, 1);
	UARTwrite(wind, size * 2);

    ROM_SysCtlDelay(delay*40000);
	holder = xG;
	UARTwrite(&holder, 1);
	UARTwrite(xGauss, size * 2);

    ROM_SysCtlDelay(delay*40000);
	holder = yG;
	UARTwrite(&holder, 1);
	UARTwrite(yGauss, size * 2);

    ROM_SysCtlDelay(delay*40000);
	holder = zG;
	UARTwrite(&holder, 1);
	UARTwrite(zGauss, size * 2);

    ROM_SysCtlDelay(delay*40000);
	holder = xA;
	UARTwrite(&holder, 1);
	UARTwrite(xAccel, size * 2);

    ROM_SysCtlDelay(delay*40000);
	holder = yA;
	UARTwrite(&holder, 1);
	UARTwrite(yAccel, size * 2);

    ROM_SysCtlDelay(delay*40000);
	holder = zA;
	UARTwrite(&holder, 1);
	UARTwrite(zAccel, size * 2);


}
int maxX = 0;
int maxY=0;
int minX = 0;
int minY=0;

void compassSensor(void) {

	int c_x1 = readData(COMPASS_ADDRESS, 0x04);
	int c_x2 = readData(COMPASS_ADDRESS, 0x03);

	int c_x = combine(c_x2, c_x1, 16);

	int answer = c_x;

	xGauss[(count / 40) % size] = answer;
	//UARTprintf(" mgauss x= %d    ", answer);
	//UARTprintf(" mgauss maxx= %d\n", maxX);

	int c_z1 = readData(COMPASS_ADDRESS, 0x06);
	int c_z2 = readData(COMPASS_ADDRESS, 0x05);


	int c_z = combine(c_z2, c_z1, 16);
	//UARTprintf(" y = %d\n",  c_y1);
	//UARTprintf(" y = %d\n",  c_y2);

	int answer2 = c_z;
	zGauss[(count / 40) % size] = answer2;

	//UARTprintf(" mgauss z= %d    ", answer2);
	int c_y1 = readData(COMPASS_ADDRESS, 0x08);
	int c_y2 = readData(COMPASS_ADDRESS, 0x07);

	int c_y = combine(c_y2, c_y1, 16);

	int answer3 = c_y;

	yGauss[(count / 40) % size] = answer3;

	//UARTprintf(" mgauss y= %d    ", answer3);
	//UARTprintf(" mgauss miny= %d ", minY);
	//UARTprintf(" mgauss maxy= %d\n", maxY);
	//UARTprintf(" z = %d\n",  c_z1);
	//UARTprintf(" z = %d\n",  c_z2);

	int a_x1 = readData(ACCEL_ADDRESS, 0x29);
	int a_x2 = readData(ACCEL_ADDRESS, 0x28);

	int a_x = combine(a_x2, a_x1, 16);

	xAccel[(count / 40) % size] = a_x;
	//UARTprintf(" x = %d\n",  a_x);

	int a_y1 = readData(ACCEL_ADDRESS, 0x2B);
	int a_y2 = readData(ACCEL_ADDRESS, 0x2A);

	int a_y = combine(a_y2, a_y1, 16);

	yAccel[(count / 40) % size] = a_y;
	int a_z1 = readData(ACCEL_ADDRESS, 0x2D);
	int a_z2 = readData(ACCEL_ADDRESS, 0x2C);

	int a_z = combine(a_z2, a_z1, 16);

	zAccel[(count / 40) % size] = a_z;
}
void tempWindSensor(void){
	int index = (count / 40) % size;
	uint32_t data[4];
	ROM_ADCProcessorTrigger(ADC0_BASE, 1);   //Ask processor to trigger ADC
			while (!ROM_ADCIntStatus(ADC0_BASE, 1, false)){}

			ROM_ADCSequenceDataGet(ADC0_BASE, 1, data);

			// UARTprintf("Channel 1, E2 %d \n", data[0]);
			int tempV = 1000 * 3.3 * data[0] / 4095;

			temp[index] = data[0];

			//UARTprintf(" temp = %d    ", tempV);
			int windV = 1000 * 3.3 * data[1] / 4095;
			//type 2.55ishV
			wind[index] = data[1];

			//UARTprintf(" wind = %d\n ", windV);

}
void writeData(int slave, int sub, int val) {

	ROM_I2CMasterSlaveAddrSet(I2C0_BASE, slave, false); // read = true
	 ROM_SysCtlDelay(1000);

    while(ROM_I2CMasterBusy(I2C0_BASE))  {}

    ROM_I2CMasterDataPut(I2C0_BASE, sub);
    ROM_SysCtlDelay(1000);
	ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    ROM_SysCtlDelay(1000);
	while(ROM_I2CMasterBusy(I2C0_BASE))  {}
    ROM_I2CMasterDataPut(I2C0_BASE, val);
    ROM_SysCtlDelay(1000);
	ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(ROM_I2CMasterBusy(I2C0_BASE))  {}

    ROM_SysCtlDelay(1000);



}
int readData (int slave, int reg) {
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, slave, false); // read = true, write = false
 //   ROM_I2CMasterDataPut(I2C0_BASE, 0x1);


    while(ROM_I2CMasterBusy(I2C0_BASE))  {}
    ROM_I2CMasterDataPut(I2C0_BASE, reg);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    while(ROM_I2CMasterBusy(I2C0_BASE))  {}


    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, slave, true); // read = true
//    ROM_I2CMasterDataPut(I2C0_BASE, 0x1);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    while(ROM_I2CMasterBusy(I2C0_BASE))  {}
    int test = ROM_I2CMasterDataGet(I2C0_BASE);


    ROM_SysCtlDelay(1000);
        return test;
}int combine(int MSB, int LSB, int bit) {
	int result = (MSB << 8) | LSB;
	  if (result / (0x1 << (bit - 1)) == 1) {
		  result = result | (0xFFFFFFFF << bit);

	   }

	  return result;
}
//
//void gyroSensor(void){
//
//
//    	int gyro[3];
//
//    	int xMSB = readData(GYRO_ADDRESS, 0x29);
//    	int xLSB = readData(GYRO_ADDRESS, 0x28);
//        gyro[0] = combine(xMSB, xLSB, 16);
//
//        int yMSB = readData(GYRO_ADDRESS, 0x2B);
//        int  yLSB = readData(GYRO_ADDRESS, 0x2A);
//        gyro[1] =  combine(yMSB, yLSB, 16);
//
//
//        int zMSB = readData(GYRO_ADDRESS, 0x2D);
//        int zLSB = readData(GYRO_ADDRESS, 0x2C);
//        gyro[2] =  combine(zMSB, zLSB, 16);
//
//
//
//        xGyro[(count / 40) % size] = gyro[0];
//
//
//           //scale = (8.75/1000)*;
//           //DVoff = +-10;
//           //milidegrees/second * miliseconds = angle change
////angleX += x*timeElapsed;
//        yGyro[(count / 40) % size] = gyro[1];
//
//


        //UARTprintf(" z = %d\n",  print);

  //      zGyro[(count / 40) % size] = gyro[2];



//}

void
Timer1AIntHandler(void)
{
	 ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);


	count++;

	if (!((count + 1) % 2)){//every 40ms
			//	gyroSensor();

			compassSensor();
			tempWindSensor();

			}


	if (!((count + 10 )% 400)) {//every 4s

		sendData();


		}




}
//*****************************************************************************
//
// This is the main example program.  It checks to see that the interrupts are
// processed in the correct order when they have identical priorities,
// increasing priorities, and decreasing priorities.  This exercises interrupt
// preemption and tail chaining.
//
//*****************************************************************************
int
main(void)
{


    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                SYSCTL_CFG_VCO_480), 120000000);




    //
    // Configure the device pins.0
    //
    PinoutSet(false, false);

    //
    // Configure the UART.
    //
    ConfigureUART();



    //
    // Set up and enable the SysTick timer.  It will be used as a reference
    // for delay loops in the interrupt handlers.  The SysTick timer period
    // will be set up for 100 times per second.
    //
    ROM_SysTickPeriodSet(120000000);//2 MHz * 2000
    ROM_SysTickEnable();


    //
    // Enable interrupts to the processor.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);


    //adc sequence
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
        ROM_ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR , 3);

        ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1);
        ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

        ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
        ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
               ROM_ADCSequenceEnable(ADC0_BASE, 1);

               ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
               ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_4,GPIO_DIR_MODE_OUT);
               ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
               ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_5,GPIO_DIR_MODE_OUT);


    //timer interrupt pins
    ROM_GPIOPinConfigure(GPIO_PD1_T0CCP1);
    ROM_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_1);


	//I2C
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	ROM_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);//clock
	ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);//data
	ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	ROM_I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
	 ROM_SysCtlDelay(100000);


	 //sETUP Gyro
//
////
//	    writeData(GYRO_ADDRESS, 0x20, 0x1F);
//	    ROM_SysCtlDelay(1000);
////
////	 //
//	    writeData(GYRO_ADDRESS, 0x22, 0x08);
//	    	    ROM_SysCtlDelay(1000);
////
//	    	    writeData(GYRO_ADDRESS, 0x23, 0x80);
//	    	    //writeData(0x80);
//	    	    ROM_SysCtlDelay(1000);
//	    //Setup Compass


		writeData(COMPASS_ADDRESS, 0x00, 0x04 << 2);//15 HZ
		ROM_SysCtlDelay(1000);

		writeData(COMPASS_ADDRESS, 0x01, 0x01 << 5);     // +-1.3Gauss
		ROM_SysCtlDelay(1000);
		writeData(COMPASS_ADDRESS, 0x02, 0x0); //continuous
		ROM_SysCtlDelay(1000);
//Setup Accel


		writeData(ACCEL_ADDRESS, 0x20, (0x01 << 5) | 0x07);//normal, xyz
		ROM_SysCtlDelay(1000);

		writeData(ACCEL_ADDRESS, 0x23, (0x01 << 6) | (0x01 << 4));     //+- 4g, at lower MSB
		ROM_SysCtlDelay(1000);

    //ROM_IntEnable(INT_I2C0);
	ROM_IntMasterEnable();





    //
    // Configure the two 32-bit periodic timers.
    //

    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_CAP_TIME);
    //ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_B_CAP_TIME_UP);
    ROM_TimerPrescaleSet(TIMER0_BASE, TIMER_B, 0);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_B, g_ui32SysClock);
    ROM_TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_BOTH_EDGES);

    //one period
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock/100); // every 10 ms
    ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, g_ui32SysClock/10); // every 10 ms


    //
    // Setup the interrupts for the timer timeouts.
    //
    ROM_IntEnable(INT_TIMER1A);
    ROM_IntEnable(INT_TIMER0B);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    //does an interrupt every timerStart seconds-the periodic interuppt


    ROM_TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);//
    //captures time since last interrupt
    //ROM_IntPrioritySet(21, 0x00);
    //ROM_IntPrioritySet(22, 0xF0);
    // Enable the timers.
    //
    ROM_TimerEnable(TIMER1_BASE, TIMER_A);
//    ROM_TimerEnable(TIMER2_BASE, TIMER_A);
  //  ROM_TimerEnable(TIMER0_BASE, TIMER_A);
  //  ROM_TimerEnable(TIMER0_BASE, TIMER_B);


    //
    // Loop forever while the timers run.
    //



    //
    while(1)
    {
    }
}
