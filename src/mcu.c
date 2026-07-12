/*
  mcu.c - peripherals emulator code for simulator MCU

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

#include <stdio.h>
#include <string.h>

#include "mcu.h"
#include "simulator.h"
#include "grbl/hal.h"

static volatile bool irq_enable = false;
static volatile bool booted = false;
static interrupt_handler isr[IRQ_N_HANDLERS];

// Per-idle-tick fast-path gates (see mcu_master_clock):
//  - timer_enabled_mask: bit i set while timer[i] is enabled. Lets an idle tick
//    skip the whole timer loop with a single word test. Maintained via
//    mcu_timer_enable(); on enable the mask bit is set BEFORE timer[].enable and
//    on disable it is cleared AFTER, so the mask is always a superset of the
//    enabled timers as observed from the sim thread (x86-TSO store ordering) -
//    the inner if(timer[i].enable) check is preserved, so a stale extra bit only
//    costs a harmless loop pass and can never cause a missed or extra IRQ.
//  - gpio_irq_pending: set whenever mcu_gpio_in raises an irq_state bit; lets an
//    idle tick skip the 10-port GPIO scan. Cleared before servicing. Each GPIO
//    handler clears its own trigger (irq_state), so a change fires exactly once,
//    identical to the previous scan-every-tick behaviour.
static volatile uint32_t timer_enabled_mask = 0;
static volatile bool gpio_irq_pending = false;

mcu_uart_t uart;
mcu_timer_t timer[MCU_N_TIMERS];
mcu_timer_t systick_timer;
gpio_port_t gpio[MCU_N_GPIO];

static void default_handler (void)
{
    // NOOP - TODO: change to blocking loop?
}

void mcu_reset (void)
{
    uint_fast8_t i;

    irq_enable = false;

    for(i = 0; i < IRQ_N_HANDLERS; i++)
        isr[i] = default_handler;

    memset(&uart, 0, sizeof(mcu_uart_t));
    memset(&timer, 0, sizeof(mcu_timer_t) * MCU_N_TIMERS);
    memset(&systick_timer, 0, sizeof(mcu_timer_t));
    memset(&gpio, 0, sizeof(gpio_port_t) * MCU_N_GPIO);

    // all timers/gpio just zeroed - reset the fast-path caches to match
    timer_enabled_mask = 0;
    gpio_irq_pending = false;

    irq_enable = true;
    booted = true;
}

// Enable/disable one of the timer[] timers, keeping timer_enabled_mask in sync.
// Ordering matters for the sim-thread fast path: set the mask bit before enabling
// and clear it after disabling so the mask is never missing a bit for a live timer.
void mcu_timer_enable (uint_fast8_t timer_id, bool on)
{
    if(on) {
        timer_enabled_mask |= (1u << timer_id);
        timer[timer_id].enable = true;
    } else {
        timer[timer_id].enable = false;
        timer_enabled_mask &= ~(1u << timer_id);
    }
}

void mcu_register_irq_handler (interrupt_handler handler, irq_num_t irq_num)
{
    isr[irq_num] = handler == NULL ? default_handler : handler;
}

void mcu_enable_interrupts (void)
{
    irq_enable = true;
}

void mcu_disable_interrupts (void)
{
    irq_enable = false;
}

void mcu_master_clock (void)
{
    uint_fast8_t i;

    if(!booted)
        return;

    // Timer loop - skipped entirely when no timer[] timer is enabled. The inner
    // per-timer logic is unchanged; only the switch() dispatch is replaced with a
    // direct isr[Timer0_IRQ + i]() call (Timer0/1/2_IRQ are consecutive enums).
    if(timer_enabled_mask) {
        for(i = 0; i < MCU_N_TIMERS; i++) {

            if(timer[i].enable) {

                if(timer[i].prescaler) {
                    if(timer[i].prescale == 0)
                        timer[i].prescale = timer[i].prescaler;
                    else {
                        if(--timer[i].prescale != 0)
                            continue;
                        else
                            timer[i].prescale = timer[i].prescaler;
                    }
                }

                if(timer[i].value == 0)
                    timer[i].value = timer[i].load;
                else if(--timer[i].value == 0) {
                    if(timer[i].irq_enable && irq_enable)
                        isr[Timer0_IRQ + i]();
                    timer[i].value = timer[i].load;
                }
            }
        }
    }

    // GPIO pin-change IRQs - skipped unless mcu_gpio_in flagged a pending change.
    // Cleared before servicing so a change raised while servicing is kept for the
    // next tick. Handlers clear their own irq_state, so each change fires once.
    if(gpio_irq_pending) {
        gpio_irq_pending = false;
        for(i = 0; i < MCU_N_GPIO; i++) {
            if(gpio[i].irq_state.value & gpio[i].irq_mask.value)
                isr[GPIO0_IRQ + i]();
        }
    }

    if(systick_timer.enable) {
        if(systick_timer.value == 0)
            systick_timer.value = systick_timer.load;
        else if(--systick_timer.value == 0) {
            if(systick_timer.irq_enable && irq_enable)
                isr[Systick_IRQ]();
            systick_timer.value = systick_timer.load;
        }
    }
}

// Returns how many of the next `max` ticks are guaranteed free of timer/GPIO
// activity: no IRQ, no reload, no prescaler wrap. Skipping that many ticks with
// mcu_skip_ticks() leaves every peripheral in the exact state `n` calls to
// mcu_master_clock() would have produced, so the following tick-by-tick step
// fires ISRs at exactly the same masterclock value as an unskipped simulation.
uint32_t mcu_ticks_to_event (uint32_t max)
{
    uint_fast8_t i;
    uint32_t t, skip = max;

    if(!booted || max == 0)
        return max;

    for(i = 0; i < MCU_N_GPIO; i++) {
        if(gpio[i].irq_state.value & gpio[i].irq_mask.value)
            return 0; // pending pin change interrupt fires every tick until serviced
    }

    for(i = 0; i < MCU_N_TIMERS; i++) {
        if(timer[i].enable) {
            if(timer[i].prescaler)
                t = timer[i].prescale ? timer[i].prescale - 1 : 0; // counter logic runs when the prescaler wraps
            else if(timer[i].value)
                t = timer[i].value - 1;         // IRQ/reload on the tick the counter reaches zero
            else
                t = timer[i].load ? 0 : max;    // reload due next tick; a 0/0 timer never does anything
            if(t < skip)
                skip = t;
        }
    }

    if(systick_timer.enable) {
        t = systick_timer.value ? systick_timer.value - 1 : (systick_timer.load ? 0 : max);
        if(t < skip)
            skip = t;
    }

    return skip;
}

// Advance all counters by `ticks` known to be event-free (see mcu_ticks_to_event).
// The > comparisons guard against a counter being reprogrammed from the grbl
// thread between the two calls; leaving it untouched then equals the write
// landing just after the jump.
void mcu_skip_ticks (uint32_t ticks)
{
    uint_fast8_t i;

    if(!booted || ticks == 0)
        return;

    for(i = 0; i < MCU_N_TIMERS; i++) {
        if(timer[i].enable) {
            if(timer[i].prescaler) {
                if(timer[i].prescale > ticks)
                    timer[i].prescale -= ticks;
            } else if(timer[i].value > ticks)
                timer[i].value -= ticks;
        }
    }

    if(systick_timer.enable && systick_timer.value > ticks)
        systick_timer.value -= ticks;
}

void mcu_gpio_set (gpio_port_t *port, uint16_t pins, uint16_t mask)
{
    port->state.value = (port->state.value & ~mask) | (pins & mask);
}

uint8_t mcu_gpio_get (gpio_port_t *port, uint16_t mask)
{
    return port->state.value & mask;
}

void mcu_gpio_toggle_in (gpio_port_t *port, uint16_t pins)
{
    mcu_gpio_in(port, (port->state.value & pins) ^ pins, pins);
}

// Set input pin, trigger interrupt if enabled
void mcu_gpio_in (gpio_port_t *port, uint16_t pins, uint16_t mask)
{
    pins &= mask;

    uint8_t changed = (port->state.value & mask) ^ pins, bitflag = 1;

    do {
        if(changed & bitflag) {
            if(port->state.value & bitflag) {
                if(port->falling.value & bitflag) {
                    port->irq_state.value |= bitflag;
                    gpio_irq_pending = true;
                }
            } else {
                if(port->rising.value & bitflag) {
                    port->irq_state.value |= bitflag;
                    gpio_irq_pending = true;
                }
            }
            changed &= ~bitflag;
        }
        bitflag <<= 1;
    } while(changed);

    port->state.value = (port->state.value & ~mask) | pins;
}

// TODO: move to mcu_master_clock() above
void simulate_serial (void)
{
    if(!booted)
        return;

    if(uart.tx_flag) {
        sim.putchar(uart.tx_data);
        uart.tx_flag = 0;
    }

    if((uart.tx_irq = uart.tx_irq_enable))
        isr[UART_IRQ]();

    // Deliver at most one byte per grbl main-loop pass (see sim_process_realtime):
    // a pulse of 0 means the grbl thread is still booting and would flush the input,
    // an unchanged pulse means it has not run since the previous byte - skip this
    // byte slot and try again at the next one, no input is lost.
    static uint32_t pulse_seen = 0;

    if(sim.grbl_pulse != pulse_seen && uart.rx_irq_enable && !uart.rx_irq && hal.stream.get_rx_buffer_free() > 100) {
        uint8_t char_in = sim.getchar();
        if (char_in) {
            pulse_seen = sim.grbl_pulse;
            uart.rx_data = char_in;
            uart.rx_irq = 1;
            isr[UART_IRQ]();
        }
    }
}
