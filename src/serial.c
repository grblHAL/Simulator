/*
  serial.c - low level functions for transmitting bytes via the serial port

  Driver code for simulator MCU

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io

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

#include <string.h>

#include "mcu.h"
#include "simulator.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static void uart_interrupt_handler (void);

//
// serialGetC - returns -1 if no data available
//
static int32_t serialGetC (void)
{
    int32_t data;
    uint_fast16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    data = (int32_t)rxbuffer.data[bptr++];          // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

    return data;
}

static inline uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - serialRxCount();
}

static void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = false;
}

static void serialRxCancel (void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
}

static bool serialPutC (const uint8_t c)
{
    uint_fast16_t next_head;

    // NOTE: If buffer and transmit register are empty buffering may be bypassed.
    //       See actual drivers for examples.

    next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);     // Get and update head pointer

    while(txbuffer.tail == next_head) {                         // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;                           // Add data to buffer
    txbuffer.head = next_head;                                  // and update head pointer

    uart.tx_irq_enable = 1;                                     // Enable TX interrupts

    return true;
}

static void serialWriteS (const char *data)
{
    uint8_t c, *ptr = (uint8_t *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

static uint16_t serialTxCount(void) {

    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (void)
{
        static const io_stream_t stream = {
        .type = StreamType_Serial,
		.is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    mcu_register_irq_handler(uart_interrupt_handler, UART_IRQ);

    uart.rx_irq_enable = 1;
    
    return &stream;
}

static void uart_interrupt_handler (void)
{
    uint_fast16_t bptr;
    int32_t data;

    if(uart.tx_irq) {

        bptr = txbuffer.tail;

        if(txbuffer.head != bptr) {

            uart.tx_data =  txbuffer.data[bptr++];            // Put character in TXT register
            bptr &= (TX_BUFFER_SIZE - 1);                     // and update tmp tail pointer
            uart.tx_irq = 0;
            uart.tx_flag = 1;

            txbuffer.tail = bptr;                             //  Update tail pinter

            if(bptr == txbuffer.head)                         // Disable TX interrups
                uart.tx_irq_enable = 0;                       // when TX buffer empty
        }
    }

    if(uart.rx_irq) {

        bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

        if(bptr == rxbuffer.tail) {                         // If buffer full
            rxbuffer.overflow = 1;                          // flag overflow and
            uart.rx_irq = 0;                                // clear interrupt flag
        } else {
            data = uart.rx_data;
            uart.rx_irq = 0;
            if(data == 0x06)
                sim.exit = exit_REQ;
            if(!enqueue_realtime_command((uint8_t)data)) {
                rxbuffer.data[rxbuffer.head] = (uint8_t)data;  	// Add data to buffer
                rxbuffer.head = bptr;                       	// and update pointer
            }
        }
    }
}
