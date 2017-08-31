/*
  joystick.c - Sample a 2-axis analog input (joystick)
  Part of LasaurGrbl

  Copyright (c) 2013 Richard Taylor

  LasaurGrbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  LasaurGrbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
*/

#include "grbl.h"

#if 0
// The joystick has no effect when the position is central +/- threshold.
#define ZERO_THRESHOLD	0x50

#define STATUS_CH0_IDLE	0x01
#define STATUS_CH1_IDLE	0x02
#define STATUS_XHAIR_ON	0x04

//
// This array is used for storing the data read from the ADC FIFO. It
// must be as large as the FIFO for the sequencer in use.  This example
// uses sequence 3 which has a FIFO depth of 1.  If another sequence
// was used with a deeper FIFO, then the array size must be changed.
//
static unsigned long joystick_x[1] = {0};
static unsigned long joystick_y[1] = {0};
static unsigned long joystick_center[2] = {0};
static unsigned long status = STATUS_CH0_IDLE | STATUS_CH1_IDLE;
static uint8_t enabled = 0;

static struct task_manual_move_data task_data = {0.0};

static void x_handler(void) {
    //
    // Clear the ADC interrupt flag.
    //
    ADCIntClear(ADC0_BASE, 0);

    //
    // Read ADC Value.
    //
    ADCSequenceDataGet(ADC0_BASE, 0, joystick_x);

    // Set the center position
    if (joystick_center[0] == 0)
    	joystick_center[0] = joystick_x[0];

    status |= STATUS_CH0_IDLE;
}

static void y_handler(void) {
    //
    // Clear the ADC interrupt flag.
    //
    ADCIntClear(ADC0_BASE, 1);

    //
    // Read ADC Value.
    //
    ADCSequenceDataGet(ADC0_BASE, 1, joystick_y);

    // Set the center position
    if (joystick_center[1] == 0)
    	joystick_center[1] = joystick_y[0];

    status |= STATUS_CH1_IDLE;
}

static void button_handler(void) {
	GPIOIntClear(JOY_PORT, JOY_MASK);

	if (GPIOPinRead(JOY_PORT, JOY_MASK) > 0)
	{
		enabled = 0;
		GPIOPinWrite(ASSIST_PORT,  (1<< AUX1_ASSIST_BIT), 0);
		task_enable(TASK_SET_OFFSET, 0);
	}
	else
	{
		enabled = 1;
		GPIOPinWrite(ASSIST_PORT,  (1<< AUX1_ASSIST_BIT), (1<< AUX1_ASSIST_BIT));
	}
}

static void joystick_isr(void) {

	TimerIntClear(JOY_TIMER, TIMER_TIMA_TIMEOUT);

	// Only allow the joystick when AUX1 (Crosshair) is enabled
	if (enabled) {

		unsigned long x = joystick_x[0];
		unsigned long y = joystick_y[0];
		double x_off = 0;
		double y_off = 0;

		if (x > joystick_center[0] + ZERO_THRESHOLD) {
			x_off = (double)(x - joystick_center[0]) / 0x800;
		} else if (x < joystick_center[0] - ZERO_THRESHOLD) {
			x_off = -(double)(joystick_center[0] - x) / 0x800;
		}

		if (y > joystick_center[1] + ZERO_THRESHOLD) {
			y_off = (double)(y - joystick_center[1]) / 0x800;
		} else if (y < joystick_center[1] - ZERO_THRESHOLD) {
			y_off = -(double)(joystick_center[1] - y) / 0x800;
		}

		// Have two speed levels for accuracy and speed when wanted.
		if (fabs(y_off) < 0.5)
			y_off /= 50.0;
		else// if (fabs(y_off) < 1.0)
			y_off /= 5.0;

		if (fabs(x_off) < 0.5)
			x_off /= 50.0;
		else// if (fabs(x_off) < 1.0)
			x_off /= 5.0;

#ifdef JOY_INVERT_Y
		task_data.x_offset = -y_off;
#else
		task_data.x_offset = y_off;
#endif

#ifdef JOY_INVERT_X
		task_data.y_offset = -x_off;
#else
		task_data.y_offset = x_off;
#endif
		task_data.rate = 20000;
		task_enable(TASK_MANUAL_MOVE, &task_data);

		//
	    // Trigger the next ADC conversion.
	    //
		if (status & STATUS_CH0_IDLE)
			ADCProcessorTrigger(ADC0_BASE, 0);
		if (status & STATUS_CH1_IDLE)
			ADCProcessorTrigger(ADC0_BASE, 1);
	}
}

void joystick_init(void) {

	// Register Joystick button isr
	GPIOPinTypeGPIOInput(JOY_PORT, JOY_MASK);
	GPIOPadConfigSet(JOY_PORT, JOY_MASK, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(JOY_PORT, JOY_MASK, GPIO_BOTH_EDGES);
	GPIOIntRegister(JOY_PORT, button_handler);
	GPIOIntEnable(JOY_PORT, JOY_MASK);

    //
    // The ADC0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //
    // Select the analog ADC function for these pins.
    //
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);

    // Use sequences 0 and 1 for x and y.
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

    // Single ended sample on CH3 (X) and CH4 (Y).
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);

    // Enable the sequences.
    ADCSequenceEnable(ADC0_BASE, 0);
    ADCSequenceEnable(ADC0_BASE, 1);

    // Register ISRs.
    ADCIntRegister(ADC0_BASE, 0, x_handler);
    ADCIntRegister(ADC0_BASE, 1, y_handler);
    ADCIntEnable(ADC0_BASE, 0);
    ADCIntEnable(ADC0_BASE, 1);

    // Trigger the first conversion (auto-center)
	ADCProcessorTrigger(ADC0_BASE, 0);
	ADCProcessorTrigger(ADC0_BASE, 1);

	// Configure timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	TimerConfigure(JOY_TIMER, TIMER_CFG_PERIODIC);

	// Create a 10ms timer callback
	TimerLoadSet64(JOY_TIMER, SysCtlClockGet() / 500);
	TimerIntRegister(JOY_TIMER, TIMER_A, joystick_isr);
	TimerIntEnable(JOY_TIMER, TIMER_TIMA_TIMEOUT);
	IntPrioritySet(INT_TIMER3A, CONFIG_JOY_PRIORITY);
	TimerEnable(JOY_TIMER, TIMER_A);
}

void joystick_enable(void){
	GPIOIntEnable(JOY_PORT, JOY_MASK);
    ADCIntEnable(ADC0_BASE, 0);
    ADCIntEnable(ADC0_BASE, 1);
}

void joystick_disable(void){
	enabled = 0;
	GPIOIntDisable(JOY_PORT, JOY_MASK);
    ADCIntDisable(ADC0_BASE, 0);
    ADCIntDisable(ADC0_BASE, 1);
	GPIOPinWrite(ASSIST_PORT,  (1<< AUX1_ASSIST_BIT), 0);
}
#endif
