/*
  mcu.h - peripherals emulator code for simulator MCU

  Part of GrblHAL

  Copyright (c) 2020-2025 Terje Io

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

#ifndef _MCU_H_

#define _MCU_H_

#include <stdint.h>
#include <stdbool.h>

#define MCU_N_TIMERS 3
#define MCU_N_GPIO 10

// Upper bound for clock ticks skipped in one jump by the event-driven main loop.
// Bounds how long a change made from the grbl thread (e.g. stepper wake-up) can go
// unnoticed to 1 ms of simulated time, mirroring the always-on systick period.
#define MCU_MAX_SKIP (F_CPU / 1000)

typedef enum  {
    Systick_IRQ = 0,
    UART_IRQ,
    Timer0_IRQ,
    Timer1_IRQ,
    Timer2_IRQ,
    GPIO0_IRQ,
    GPIO1_IRQ,
    GPIO2_IRQ,
    GPIO3_IRQ,
    GPIO4_IRQ,
    GPIO5_IRQ,
    GPIO6_IRQ,
    GPIO7_IRQ,
    GPIO8_IRQ,
    GPIO9_IRQ,
    IRQ_N_HANDLERS
} irq_num_t;

// NOTE: fields are volatile because peripheral "registers" are accessed both from
// the simulator thread (mcu_master_clock/ISRs) and the grbl thread (HAL calls).
typedef struct
{
    volatile bool enable;
    volatile bool irq_enable;
    volatile uint32_t value;
    volatile uint32_t load;
    volatile uint32_t prescale;
    volatile uint32_t prescaler;
    volatile uint32_t compare;
} mcu_timer_t;

typedef struct
{
    volatile bool rx_irq;
    volatile bool tx_irq;
    volatile bool tx_flag;
    volatile bool rx_irq_enable;
    volatile bool tx_irq_enable;
    volatile uint8_t rx_data;
    volatile uint8_t tx_data;
    volatile uint32_t cdiv;
} mcu_uart_t;

typedef union {
    uint16_t value;
    uint16_t mask;
    struct {
        uint16_t pin0  :1,
                 pin1  :1,
                 pin2  :1,
                 pin3  :1,
                 pin4  :1,
                 pin5  :1,
                 pin6  :1,
                 pin7  :1,
                 pin8  :1,
                 pin9  :1,
                 pin10 :1,
                 pin11 :1,
                 pin12 :1,
                 pin13 :1,
                 pin14 :1,
                 pin15 :1;
    };
} gpio_pins_t;

typedef struct
{
    volatile gpio_pins_t dir;
    volatile gpio_pins_t state;
    volatile gpio_pins_t irq_mask;
    volatile gpio_pins_t irq_state;
    volatile gpio_pins_t rising;
    volatile gpio_pins_t falling;
    volatile gpio_pins_t pullup;
    volatile gpio_pins_t pulldown;
} gpio_port_t;

extern mcu_uart_t uart;
extern mcu_timer_t timer[MCU_N_TIMERS];
extern gpio_port_t gpio[MCU_N_GPIO];
extern mcu_timer_t systick_timer;

typedef void (*interrupt_handler)(void);

void mcu_reset (void);
void mcu_enable_interrupts (void);
void mcu_disable_interrupts (void);
void mcu_master_clock (void);
uint32_t mcu_ticks_to_event (uint32_t max);
void mcu_skip_ticks (uint32_t ticks);
void mcu_timer_enable (uint_fast8_t timer_id, bool on);
void mcu_register_irq_handler (interrupt_handler handler, irq_num_t irq_num);
void mcu_gpio_set (gpio_port_t *port, uint16_t pins, uint16_t mask);
uint8_t mcu_gpio_get (gpio_port_t *port, uint16_t mask);
void mcu_gpio_in (gpio_port_t *port, uint16_t pins, uint16_t mask);
void mcu_gpio_toggle_in (gpio_port_t *port, uint16_t pins);
void simulate_serial (void);

#endif
