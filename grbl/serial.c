/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

#include "usb_serial_structs.h"

void process_data(void);

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)

#define USE_SERIAL_PROCESS_TIMER

uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint32_t serial_rx_buffer_head = 0;
volatile uint32_t serial_rx_buffer_tail = 0;

// Returns the number of bytes available in the RX serial buffer.
uint32_t serial_get_rx_buffer_available()
{
  uint32_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint32_t serial_get_rx_buffer_count()
{
  uint32_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint32_t serial_get_tx_buffer_count()
{
	return USBBufferDataAvailable(&g_sTxBuffer);
}

void serial_init()
{
  /* Setup pins for USB operation */
  GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

  //
  // Initialize the transmit and receive buffers.
  //
  USBBufferInit(&g_sTxBuffer);
  USBBufferInit(&g_sRxBuffer);

  //
  // Set the USB stack mode to Device mode with VBUS monitoring.
  //
  USBStackModeSet(0, eUSBModeForceDevice, 0);

  //
  // Pass our device information to the USB library and place the device
  // on the bus.
  //
  USBDCDCInit(0, &g_sCDCDevice);

  IntPrioritySet(INT_USB0, CONFIG_USB_PRIORITY);

#ifdef USE_SERIAL_PROCESS_TIMER
  // Configure timer
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);

  // Create a 1us timer, used to process USB data
  // at a lower IRQ priority.
  TimerLoadSet64(TIMER2_BASE, SysCtlClockGet() / 1000000);
  TimerIntRegister(TIMER2_BASE, TIMER_A, process_data);
  TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
  IntPrioritySet(INT_TIMER2A, CONFIG_SENSE_PRIORITY);
#endif
}


// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data) {
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);


  /* Wait for some space */
  while (USBBufferSpaceAvailable(&g_sTxBuffer) < 1) {};

  /* Send the byte (inefficient!) */
  USBBufferWrite(&g_sTxBuffer, &data, 1);
}

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint32_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)

  if (serial_rx_buffer_head == tail) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    serial_rx_buffer_tail = tail;

    return data;
  }
}


void process_data(void)
{
  uint8_t data;
  uint32_t next_head;

#ifdef USE_SERIAL_PROCESS_TIMER
  TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
#endif

  while (USBBufferDataAvailable(&g_sRxBuffer) > 0)
  {
    USBBufferRead(&g_sRxBuffer, &data, 1);

    // Pick off realtime command characters directly from the serial stream. These characters are
	// not passed into the main buffer, but these set system state flag bits for realtime execution.
	switch (data) {
      case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
      case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
      case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
      case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
      default :
        if (data > 0x7F) { // Real-time control characters are extended ACSII only.
          switch(data) {
            case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
            case CMD_JOG_CANCEL:
              if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
                system_set_exec_state_flag(EXEC_MOTION_CANCEL);
              }
              break;
            #ifdef DEBUG
              case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
            #endif
            case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
            case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
            case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
            case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
            case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
            case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
            case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
            case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
            case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
            case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
            case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
            case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
            case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
            case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
            case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
            #ifdef ENABLE_M7
              case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
            #endif
          }
          // Throw away any unfound extended-ASCII character.
        } else { // Write character to buffer
          next_head = serial_rx_buffer_head + 1;
          if (next_head == RX_RING_BUFFER) { next_head = 0; }

          // Write data to buffer unless it is full.
          if (next_head != serial_rx_buffer_tail) {
            serial_rx_buffer[serial_rx_buffer_head] = data;
            serial_rx_buffer_head = next_head;
          } else {
            printf("Buffer Overrun\r\n");
          }
        }
	}
  }
}


void serial_reset_read_buffer()
{
  USBBufferFlush(&g_sRxBuffer);
  serial_rx_buffer_tail = serial_rx_buffer_head;
}


static tLineCoding g_sLineCoding =
{
    115200,                     /* 115200 baud rate. */
    1,                          /* 1 Stop Bit. */
    0,                          /* No Parity. */
    8                           /* 8 Bits of data. */
};

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
ControlHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    //
    // Which event are we being asked to process?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
            break;

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
            break;

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
        	{
				tLineCoding *psLineCoding = pvMsgData;
	            /* Copy the current line coding information into the structure. */
	            psLineCoding->ui32Rate = g_sLineCoding.ui32Rate;
	            psLineCoding->ui8Stop = g_sLineCoding.ui8Stop;
	            psLineCoding->ui8Parity = g_sLineCoding.ui8Parity;
	            psLineCoding->ui8Databits = g_sLineCoding.ui8Databits;
        	}
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
        	{
				tLineCoding *psLineCoding = pvMsgData;
				g_sLineCoding.ui32Rate = psLineCoding->ui32Rate;
				g_sLineCoding.ui8Stop = psLineCoding->ui8Stop;
				g_sLineCoding.ui8Parity = psLineCoding->ui8Parity;
				g_sLineCoding.ui8Databits = psLineCoding->ui8Databits;
        	}
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        	if (ui32MsgValue & (1 << 0)) {
                //
                // Flush our buffers.
                //
                USBBufferFlush(&g_sTxBuffer);
                USBBufferFlush(&g_sRxBuffer);
                //delay_ms(1000);
            	mc_reset();
        	}
        	break;

        //
        // Send a break condition on the serial line.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
            break;

        //
        // Clear the break condition on the serial line.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
            break;

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events.
        //
        default:
            break;

    }

    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
            break;

    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
#ifdef USE_SERIAL_PROCESS_TIMER
            TimerEnable(TIMER2_BASE, TIMER_A);
#else
            process_data();
#endif
            break;

        //
        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the UART is currently idle or 1 if it is
        // in the process of transmitting something. The actual number of
        // bytes in the UART FIFO is not important here, merely whether or
        // not everything previously sent to us has been transmitted.
        //
        case USB_EVENT_DATA_REMAINING:
        {
            return 0;
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return 0;
        }

        //
        // We don't expect to receive any other events.
        //
        default:
            break;
    }

    return(0);
}
