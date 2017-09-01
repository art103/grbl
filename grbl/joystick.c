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

// The joystick has no effect when the position is central +/- threshold.
#define ZERO_THRESHOLD	0x300

#define STATUS_CH0_IDLE	    0x01
#define STATUS_CH1_IDLE	    0x02

volatile char jog_cmd[32];

//
// This array is used for storing the data read from the ADC FIFO. It
// must be as large as the FIFO for the sequencer in use.  This example
// uses sequence 3 which has a FIFO depth of 1.  If another sequence
// was used with a deeper FIFO, then the array size must be changed.
//
static unsigned long joystick_x[1] = {0};
static unsigned long joystick_y[1] = {0};
static unsigned long joystick_center[2] = {0};
static volatile uint8_t adc_status = STATUS_CH0_IDLE | STATUS_CH1_IDLE;
static volatile uint8_t jog_status = 0;
static volatile uint8_t jog_count = 0;

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

    adc_status |= STATUS_CH0_IDLE;
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

    adc_status |= STATUS_CH1_IDLE;
}

static void button_handler(void) {
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);

	if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0) > 0)
	{
		protocol_inject_line("G10L20P0X0Y0Z0");
	}
}

static void joystick_isr(void) {

	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

	// Only allow the joystick when AUX1 (Crosshair) is enabled
	if (1) {

		unsigned long x = joystick_x[0];
		unsigned long y = joystick_y[0];
		long x_off = 0;
		long y_off = 0;

		if (x > joystick_center[0] + ZERO_THRESHOLD) {
			x_off = 1;
		} else if (x < joystick_center[0] - ZERO_THRESHOLD) {
			x_off = -1;
		}

		if (y > joystick_center[1] + ZERO_THRESHOLD) {
			y_off = 1;
		} else if (y < joystick_center[1] - ZERO_THRESHOLD) {
			y_off = -1;
		}

		if (x_off == 0 && y_off == 0)
		{
			// Back in the centre, send the jog
			jog_status = 0;
			jog_count = 4;
		}
		else// if (jog_status < 5)
		{
			jog_count++;

			// Get up to date ADC readings (to catch return to center),
			// but repeat jog at lower rate.
			if (jog_count > 5)
			{
				jog_count = 0;

				jog_status++;

				if (jog_status != 0)
				{
					int jog_len = 10;

					if (jog_status < 4)
					{
						jog_len = 1;
					}
					else if (jog_status < 8)
					{
						jog_len = 5;
					}

					x_off *= -jog_len;
					y_off *= -jog_len;

					sprintf(jog_cmd, "$J=G91G21X%d.0Y%d.0F1000", y_off, x_off);
					protocol_inject_line(jog_cmd);
				}
			}
		}
		//
	    // Trigger the next ADC conversion.
	    //
		if (adc_status & STATUS_CH0_IDLE)
			ADCProcessorTrigger(ADC0_BASE, 0);
		if (adc_status & STATUS_CH1_IDLE)
			ADCProcessorTrigger(ADC0_BASE, 1);
	}
}

void joystick_init(void) {

	// Register Joystick button isr
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);
	GPIOIntRegister(GPIO_PORTF_BASE, button_handler);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);

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
	TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

	// Create a 50ms timer callback
	TimerLoadSet64(TIMER3_BASE, SysCtlClockGet() / 20);
	TimerIntRegister(TIMER3_BASE, TIMER_A, joystick_isr);
	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	IntPrioritySet(INT_TIMER3A, CONFIG_JOY_PRIORITY);
	TimerEnable(TIMER3_BASE, TIMER_A);
}
