/*
  platform_linux.c - linux specific functions with generic cross-platform interface

  Part of Grbl Simulator

  Copyright (c) 2014 Adam Shelly

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
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include "platform.h"

#define MS_PER_SEC 1000000

#define SIM_ECHO_TERMINAL 0 //use this to make grbl_sim act like a serial terminal with local echo on.

// Saved STDIN state so it can be restored on exit. The terminal is switched to
// raw mode (and STDIN made non-blocking) once at startup instead of on every
// poll, so platform_poll_stdin() can be a single non-blocking read().
static struct termios orig_termios;
static int termios_saved = 0;
static int orig_stdin_flags = 0;
static int stdin_flags_saved = 0;

//restore STDIN to the state it had before platform_init()
static void platform_restore_terminal(void)
{
    if (termios_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
        termios_saved = 0;
    }
    if (stdin_flags_saved) {
        fcntl(STDIN_FILENO, F_SETFL, orig_stdin_flags);
        stdin_flags_saved = 0;
    }
}

//any platform-specific setup that must be done before sim starts here
void platform_init()
{
    // Make STDIN non-blocking once so polling is a single read() with no
    // per-poll select()/tcsetattr() dance. Save the original flags so they can
    // be restored on exit (important when STDIN is a shared tty).
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags != -1) {
        orig_stdin_flags = flags;
        stdin_flags_saved = 1;
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    // If STDIN is a terminal, switch it to raw (non-canonical, no echo) mode
    // once. For a pipe/redirected file tcgetattr() fails and we leave it alone,
    // matching the old per-poll code (which no-op'd on non-ttys).
    if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &orig_termios) == 0) {
        struct termios raw = orig_termios;
        raw.c_lflag &= ~(ICANON);
        if (!SIM_ECHO_TERMINAL)
            raw.c_lflag &= ~(ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        termios_saved = 1;
    }

    // Restore the terminal even on abnormal exit.
    atexit(platform_restore_terminal);
}

//cleanup int here;
void platform_terminate()
{
    platform_restore_terminal();
}

//returns a free-running 32 bit nanosecond counter which rolls over
uint32_t platform_ns() 
{
    static uint32_t gTimeBase = 0;
    static uint32_t timestamp;
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    timestamp = ts.tv_sec * (uint32_t)1e9 + ts.tv_nsec;
    if (gTimeBase == 0)
        gTimeBase = timestamp;

    return timestamp - gTimeBase;
}

//sleep in microseconds
void platform_sleep(long  microsec)
{
    struct timespec ts={0};

    while (microsec >= MS_PER_SEC){
        ts.tv_sec++;
        microsec -= MS_PER_SEC;
    }
    ts.tv_nsec = microsec * 1000;
    nanosleep(&ts, NULL);
}

plat_thread_t* platform_start_thread(plat_threadfunc_t threadfunc)
{
    plat_thread_t* th = malloc(sizeof(plat_thread_t));
    if (pthread_create(&th->tid, NULL, threadfunc, &th->exit)){
        free(th);
        return NULL;
    }
    return th;
}

//ask thread to exit nicely, wait
void platform_stop_thread(plat_thread_t* th)
{
    th->exit = 1;
    pthread_join(th->tid, NULL);  
}

//force-kill thread
void platform_kill_thread(plat_thread_t* th)
{
    th->exit = 1;
    pthread_cancel(th->tid); 
}

//return char if one available.
uint8_t platform_poll_stdin()
{
    uint8_t char_in = 0;

    // STDIN was put in non-blocking mode once in platform_init(), so a single
    // read() replaces the old per-poll tcgetattr/tcsetattr/select/getchar.
    ssize_t n = read(STDIN_FILENO, &char_in, 1);

    if (n == 1)
        return char_in;   // byte available
    if (n == 0)
        return 0xFF;      // EOF: matches old getchar()==EOF cast to uint8_t

    return 0;             // no data (EAGAIN/EWOULDBLOCK) or error -> nothing
}
