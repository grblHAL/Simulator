/*
  driver.c - driver code for simulator MCU

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>

#include "mcu.h"
#include "driver.h"
#include "serial.h"
#include "eeprom.h"
#include "grbl_eeprom_extensions.h"
#include "platform.h"

#include "grbl/hal.h"

#ifndef SQUARING_ENABLED
#define SQUARING_ENABLED 0
#endif

static spindle_id_t spindle_id;
static bool probe_invert;
static uint32_t ticks = 0;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static on_execute_realtime_ptr on_execute_realtime;

void SysTick_Handler (void);
void Stepper_IRQHandler (void);
void Limits0_IRQHandler (void);
void Control_IRQHandler (void);

#if SQUARING_ENABLED
static axes_signals_t motors_0 = {AXES_BITMASK}, motors_1 = {AXES_BITMASK};
void Limits1_IRQHandler (void);
#endif

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        systick_timer.enable = 1;
        if(!(delay.callback = callback))
            while(delay.ms);
    } else if(callback)
        callback();
}

#if SQUARING_ENABLED

inline static void set_step_outputs (axes_signals_t step_out_0)
{
    axes_signals_t step_out_1;

    step_out_1.bits = (step_out_0.bits & motors_1.bits) ^ settings.steppers.step_invert.bits;
    step_out_0.bits = (step_out_0.bits & motors_0.bits) ^ settings.steppers.step_invert.bits;

    mcu_gpio_set(&gpio[STEP_PORT0], step_out_0.bits, AXES_BITMASK);
    mcu_gpio_set(&gpio[STEP_PORT1], step_out_1.bits, AXES_BITMASK);
}

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
        ganged.x = On;
    } else {
        ganged.x = On;
    }

    return ganged;
}

#else

inline static void set_step_outputs (axes_signals_t step_out)
{
    step_out.bits = (step_out.bits) ^ settings.steppers.step_invert.bits;

    mcu_gpio_set(&gpio[STEP_PORT0], step_out.bits, AXES_BITMASK);
}

#endif

inline static void set_dir_outputs (axes_signals_t dir_out)
{
    mcu_gpio_set(&gpio[DIR_PORT], dir_out.value ^ settings.steppers.dir_invert.mask, AXES_BITMASK);
}

static void stepperEnable (axes_signals_t enable, bool hold)
{
    mcu_gpio_set(&gpio[STEPPER_ENABLE_PORT], enable.value ^ settings.steppers.enable_invert.mask, AXES_BITMASK);
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    timer[STEPPER_TIMER].load = 5000;
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].enable = 1;

//    hal.stepper_interrupt_callback();   // start the show
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].load = 0;
    timer[STEPPER_TIMER].enable = 0;

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout, limiting the slowest speed
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    timer[STEPPER_TIMER].load = cycles_per_tick;
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].enable = 1;
}

// "Normal" version: Sets stepper direction and pulse pins and starts a step pulse a few nanoseconds later.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }

    if(stepper->step_out.bits) {
        set_step_outputs(stepper->step_out);
    }
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
// TODO: only delay after setting dir outputs?
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }

    if(stepper->step_out.bits) {
//        next_step_out = stepper->step_out; // Store out_bits
//        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }
}

static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.value = gpio[LIMITS_PORT0].state.value;

    if (settings.limits.invert.mask)
        signals.min.mask ^= settings.limits.invert.mask;

    return signals;
}

#if SQUARING_ENABLED

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_0.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0);
    motors_1.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0);
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
static limit_signals_t limitsGetHomeState()
{
    limit_signals_t signals = {0};

    if(motors_0.mask) {

        signals.min.mask = gpio[LIMITS_PORT0].state.value;

        if (settings.limits.invert.mask)
            signals.min.mask ^= settings.limits.invert.mask;
    }

    if(motors_1.mask) {

       signals.max.mask = gpio[LIMITS_PORT1].state.value;

        if (settings.limits.invert.mask)
            signals.max.mask ^= settings.limits.invert.mask;
    }

    return signals;
}

#endif

static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    gpio[LIMITS_PORT0].irq_mask.mask = on ? AXES_BITMASK : 0;
    gpio[LIMITS_PORT0].irq_state.mask = 0;

  #if SQUARING_ENABLED
    gpio[LIMITS_PORT1].irq_mask.mask = on ? AXES_BITMASK : 0;
    gpio[LIMITS_PORT1].irq_state.mask = 0;

    hal.limits.get_state = homing_cycle.mask != 0 ? limitsGetHomeState : limitsGetState;
  #endif
}

static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.mask = gpio[CONTROL_PORT].state.value;
	signals.limits_override = settings.control_invert.limits_override;

    if(settings.control_invert.mask)
        signals.mask ^= settings.control_invert.mask;

    return signals;
}

static void probeConfigureInvertMask (bool is_probe_away, bool probing)
{
  probe_invert = settings.probe.invert_probe_pin;

  if (is_probe_away)
      probe_invert ^= is_probe_away;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.value = mcu_gpio_get(&gpio[PROBE_PORT], PROBE_MASK);

    state.triggered ^= probe_invert;

    return state;
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    mcu_gpio_set(&gpio[SPINDLE_PORT], state.value ^ settings.pwm_spindle.invert.mask, SPINDLE_MASK);
}

// Variable spindle control functions

// Sets spindle speed
static void spindle_set_speed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return 0; //spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    mcu_gpio_set(&gpio[SPINDLE_PORT], state.value ^ settings.pwm_spindle.invert.mask, SPINDLE_MASK);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};

    state.value = gpio[SPINDLE_PORT].state.value ^ settings.pwm_spindle.invert.mask;

    return state;
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

//    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

static void coolantSetState (coolant_state_t mode)
{
    mcu_gpio_set(&gpio[COOLANT_PORT], mode.value ^ settings.coolant.invert.mask, COOLANT_MASK);
}

static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.value = gpio[COOLANT_PORT].state.value ^ settings.coolant.invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
//    __disable_interrupts();
    *ptr |= bits;
//    __enable_interrupts();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
//    __disable_interrupts();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
//    __enable_interrupts();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
//    __disable_interrupts();
    uint_fast16_t prev = *ptr;
    *ptr = value;
//    __enable_interrupts();
    return prev;
}

void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    if(changed.spindle) {
        spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
        if(spindle_id == spindle_get_default())
            spindle_select(spindle_id);
    }

#if SQUARING_ENABLED
    hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif
}

bool driver_setup (settings_t *settings)
{
    timer[STEPPER_TIMER].prescaler = 0;
    timer[STEPPER_TIMER].irq_enable = 1;
    mcu_register_irq_handler(Stepper_IRQHandler, Timer0_IRQ);

    gpio[STEPPER_ENABLE_PORT].dir.mask = AXES_BITMASK;
    gpio[STEP_PORT0].dir.mask = AXES_BITMASK;
    gpio[DIR_PORT].dir.mask = AXES_BITMASK;

    gpio[COOLANT_PORT].dir.mask = COOLANT_MASK;
    gpio[SPINDLE_PORT].dir.mask = SPINDLE_MASK;

    gpio[LIMITS_PORT0].dir.mask = AXES_BITMASK;
    gpio[LIMITS_PORT0].rising.mask = AXES_BITMASK;
    mcu_register_irq_handler(Limits0_IRQHandler, LIMITS_IRQ0);

#if SQUARING_ENABLED
    gpio[STEP_PORT1].dir.mask = AXES_BITMASK;

    gpio[LIMITS_PORT1].dir.mask = AXES_BITMASK;
    gpio[LIMITS_PORT1].rising.mask = AXES_BITMASK;
    mcu_register_irq_handler(Limits1_IRQHandler, LIMITS_IRQ1);
#endif

    gpio[CONTROL_PORT].dir.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].rising.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].irq_mask.mask = CONTROL_MASK;
    mcu_register_irq_handler(Control_IRQHandler, CONTROL_IRQ);

    mcu_gpio_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT, PROBE_CONNECTED_BIT); // default to connected

    settings_changed_flags_t changed_flags = {0};
    hal.settings_changed(settings, changed_flags);
    hal.stepper.go_idle(true);
    spindle_ptrs_t* spindle;

    if((spindle = spindle_get(0))) {
        spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
    }

    hal.coolant.set_state((coolant_state_t){0});

    return settings->version.id == 23;
}

// used to inject a sleep in grbl main loop,
// ensures hardware simulator gets some cycles in "parallel"
void sim_process_realtime (uint_fast16_t state)
{
    //platform_sleep(0); // yield needed? or simply trust the OS's thread scheduler...
    on_execute_realtime(state);
}

uint32_t millis (void)
{
    return ticks;
}

bool driver_init ()
{
    mcu_reset();

    mcu_register_irq_handler(SysTick_Handler, Systick_IRQ);

    systick_timer.load = F_CPU / 1000 - 1;
    systick_timer.irq_enable = 1;
    systick_timer.enable = 1;

    hal.info = "Simulator";
    hal.driver_version = "251002";
    hal.driver_setup = driver_setup;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.f_step_timer = F_CPU;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = sim_process_realtime;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#if SQUARING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigureInvertMask;

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.laser = On,
        .cap.direction = On,
        .config = spindleConfig,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindle_set_speed,
        .set_state = spindleSetState,
        .get_state = spindleGetState
    };

    spindle_register(&spindle, "simulated PWM spindle");

    hal.control.get_state = systemGetState;
/*
    hal.show_message = showMessage;
*/

    memcpy(&hal.stream, serialInit(), sizeof(io_stream_t));
    hal.nvs.type = NVS_EEPROM;
    hal.nvs.get_byte = eeprom_get_char;
    hal.nvs.put_byte = eeprom_put_char;
    hal.nvs.memcpy_to_nvs = memcpy_to_eeprom;
    hal.nvs.memcpy_from_nvs = memcpy_from_eeprom;

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = millis;

    hal.driver_cap.amass_level = 3;
    hal.coolant_cap.flood = On;
    hal.coolant_cap.mist = On;
    // hal.driver_cap.software_debounce = On;
    // This is required for the hal to initialize properly!
    hal.driver_cap.step_pulse_delay = On;

    hal.signals_cap.safety_door_ajar = On;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 10;
}

// Main stepper driver
void Stepper_IRQHandler (void)
{
    hal.stepper.interrupt_callback();
}

void Control_IRQHandler (void)
{
    gpio[CONTROL_PORT].irq_state.value = ~CONTROL_MASK;
    hal.control.interrupt_callback(hal.control.get_state());
}

void Limits0_IRQHandler (void)
{
    gpio[LIMITS_PORT0].irq_state.value = (uint8_t)~AXES_BITMASK;
    hal.limits.interrupt_callback(hal.limits.get_state());
}

#if SQUARING_ENABLED

void Limits1_IRQHandler (void)
{
    gpio[LIMITS_PORT1].irq_state.value = (uint8_t)~AXES_BITMASK;
    hal.limits.interrupt_callback(hal.limits.get_state());
}

#endif

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
    ticks++;

    if(delay.ms && --delay.ms == 0) {
//        systick_timer.enable = 0;
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
