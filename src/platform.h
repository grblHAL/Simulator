#ifndef platform_h

#ifdef  PLATFORM_LINUX
#include "platform_linux.h"
#elif  PLATFORM_OSX
#include "platform_osx.h"
#else
#include "platform_windows.h"
#endif

#define platform_h
#include <inttypes.h>

// On the host, the grbl thread and the simulator thread share one (or few) real
// CPU cores. grbl's main loop and driver_delay_ms's spin loop run at full host
// speed, so without cooperation they starve the simulator thread and burn a
// whole core (process CPU ~= 2x wall). Yielding the host CPU here does NOT change
// simulation timing: the simulated tick at which every event fires is driven by
// the simulator thread's clock/ISRs, not by how often the grbl thread polls.
// It only lets the scheduler hand cycles to the simulator thread when the grbl
// thread would otherwise busy-spin.
#ifndef WIN32
#include <sched.h>
#define sim_yield() sched_yield()
#else
// SwitchToThread() hands the rest of the timeslice to another ready thread
// (the simulator thread) and returns immediately when none is waiting - the
// Windows equivalent of sched_yield(). windows.h comes in via platform.h.
#define sim_yield() SwitchToThread()
#endif

void platform_init();
void platform_terminate();

plat_thread_t* platform_start_thread(plat_threadfunc_t func);
void platform_stop_thread(plat_thread_t* thread);
void platform_kill_thread(plat_thread_t* thread);

uint32_t  platform_ns();  //monotonically increasing nanoseconds since program start.
void platform_sleep(long microsec); //sleep for suggested time in microsec.

uint8_t platform_poll_stdin(); //non-blocking stdin read - returns 0 if no char present, 0xFF for EOF

#endif
