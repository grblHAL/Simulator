/*
  main.c - main grbl simulator program

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler
  Copyright (c) 2014-2015 Adam Shelly

  2020 - modified for grblHAL by Terje Io

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

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#endif

#include "simulator.h"
#include "eeprom.h"
#include "grbl_interface.h"

#include "grbl/grbllib.h"

arg_vars_t args;
const char* progname;

#ifdef WIN32
static SOCKET socket_fd;
#else
static int socket_fd = 0;
#endif
static fd_set rfds;

void print_usage(const char* badarg)
{
    if (badarg)
        printf("Unrecognized option %s\n",badarg);

    printf("Usage: \n"
      "%s [options] [time_step] [block_file]\n"
      "  Options:\n"
      "    -r <report time>   : minimum time step for printing stepper values. Default=0=no print.\n"
      "    -t <time factor>   : multiplier to realtime clock. Default=1.0; 0=\"as fast as possible\"\n"
      "    -g <response file> : file to report responses from grbl.  default = stdout\n"
      "    -b <block file>    : file to report each block executed.  default = stdout\n"
      "    -s <step file>     : file to report each step executed.  default = stderr\n"
      "    -e <EEPROM file>   : file containing grblHAL settings.  default = EEPROM.DAT\n"
      "    -p <port>          : port to open raw telnet communication.\n"
      "    -c<comment_char>   : character to print before each line from grbl.  default = '#'\n"
      "    -n                 : no comments before grbl response lines.\n"
      "    -h                 : this help.\n"
      "\n"
      "  <time_step> and <block_file> can be specifed with option flags or positional parameters\n"
      "\n"
      "  ^-F to shutdown cleanly\n"
      "\n",
      progname);
}

//wrapper for thread interface
PLAT_THREAD_FUNC(grbl_main_thread, exit)
{
    grbl_enter();

    return 0; //NULL;
}

#ifdef WIN32

//return char if one available.
uint8_t sim_socket_in()
{
    char c = 0;
    int retval;
    DWORD t = 0;

    if(sim.socket_fd == INVALID_SOCKET) {
        sim.socket_fd = accept(socket_fd, NULL, NULL);
        setsockopt(sim.socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&t, sizeof(t));
        u_long mode = 1;
        ioctlsocket(sim.socket_fd, FIONBIO, &mode); // set non-blocking
    } else if((retval = recv(sim.socket_fd, &c, 1, 0)) < 1) {
        if(retval == 0)
            sim.socket_fd = INVALID_SOCKET;
        else if((retval = WSAGetLastError()) != WSAEWOULDBLOCK) {
            printf("Fatal: socket error %d\n", retval);
            exit(-5);
        }
    }

    return c;
}

#else

//return char if one available.
uint8_t sim_socket_in()
{
    char c = 0;
    int retval;

    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = 0
    }; 

    fd_set cfds = rfds;

    if((retval = select((socket_fd > sim.socket_fd ? socket_fd : sim.socket_fd) + 1, &cfds, NULL, NULL, &tv)) > 0) {

        if(FD_ISSET(socket_fd, &cfds)) {

            socklen_t addrlen = sizeof(struct sockaddr_in);
            struct sockaddr_in client_addr;

            sim.socket_fd = accept(socket_fd, (struct sockaddr *) &client_addr, &addrlen);
            if (sim.socket_fd < 0) {
                printf("Fatal: Error on socket accept.\n");
                exit(-5);
            }

            FD_SET(sim.socket_fd, &rfds);

            struct timeval t = {
                .tv_sec = 0,
                .tv_usec = 0
            }; 

            setsockopt(sim.socket_fd, SOL_SOCKET, SO_RCVTIMEO, &t, sizeof(t));
        }

        if(FD_ISSET(sim.socket_fd, &cfds)) {

            retval = read(sim.socket_fd, &c, 1);

            if(retval == 0) {
                close(sim.socket_fd);
                FD_CLR(sim.socket_fd, &rfds);
                sim.socket_fd = 0;
            }
//            if(c && c != '?' && c >= ' ')
//                sim_serial_out(c == '\r' ? '\n' : c);
        }
    }

    return c;
}

#endif

static void exithandler (int signum)
{
    eeprom_close();
}

int main(int argc, char *argv[])
{
    int positional_args = 0;

    //defaults
    args.step_out_file = stderr;
    args.block_out_file = stdout;
    args.serial_out_file = stdout;
    args.comment_char = '#';
    args.speedup = 1.0f;

    args.step_time = 0.0f;
    // Get the minimum time step for printing stepper values.
    // If not given or the command line cannot be parsed to a float than
    // step_time= 0.0; This means to not print stepper values at all

    set_eeprom_name("EEPROM.DAT");

    progname = argv[0];

    while (argc > 1) {
        argv++; argc--;
        if (argv[0][0] == '-') {

            switch(argv[0][1]){

                case 'c':  //set Comment char 
                    args.comment_char = argv[0][2];
                    break;

                case 'n': //No comment char on grbl responses
                    args.comment_char = 0;
                    break;

                case 't': //Tick rate
                    argv++; argc--;
                    args.speedup = atof(*argv);
                    break;

                case 'b': //Block file
                    argv++; argc--;
                    args.block_out_file = fopen(*argv,"w");
                    if (!args.block_out_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return EXIT_FAILURE;
                    }
                    break;

                case 'e': //EEPROM file
                    argv++; argc--;
                    set_eeprom_name(*argv);
                    break;

                case 's': //Step out file.
                    argv++; argc--;
                    args.step_out_file = fopen(*argv,"w");
                    if (!args.step_out_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return EXIT_FAILURE;
                    }
                    break;

                case 'g': //Grbl output
                    argv++; argc--;
                    args.serial_out_file = fopen(*argv,"w");
                    if (!args.serial_out_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return EXIT_FAILURE;
                    }
                    break;

                case 'r':  //step_time for Reporting
                    argv++; argc--;
                    args.step_time = atof(*argv);
                    break;

                case 'p':  // Raw telnet port
                    argv++; argc--;
                    args.port = atoi(*argv);
                    break;

                case 'h':
                    print_usage(NULL);
                    return EXIT_SUCCESS;

                default:
                    print_usage(*argv);
                    return EXIT_FAILURE;
            }
        }
        else { //handle old positional argument interface

            switch(++positional_args) {

                case 1:
                    args.step_time= atof(*argv);
                    break;

                case 2:  //block out and grbl out to same file, like before.
                    args.block_out_file = fopen(*argv,"w");
                    if (!args.block_out_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return EXIT_FAILURE;
                    }
                    args.serial_out_file = args.block_out_file;
                    break;

                default:
                    print_usage(*argv);
                    return EXIT_FAILURE;
            }
        }
    }

    // Make sure the output streams are flushed immediately.
    // This is important when using the simulator inside another application in parallel
    // to the real grbl.
    // Theoretically flushing could be limited to complete lines. Unfortunately Windows
    // does not know line buffered streams. So for now we stick to flushing every character.
    //setvbuf(stdout, NULL, _IONBF, 1);
    //setvbuf(stderr, NULL, _IONBF, 1);
    //( Files are now closed cleanly when sim gets EOF or CTRL-F.)
    platform_init(); 

    sim.on_init = grbl_app_init;
    sim.on_shutdown = grbl_app_exit;
    sim.on_tick = grbl_per_tick;
    sim.on_byte = grbl_per_byte;

    init_simulator();

    if(args.port) {

#ifdef WIN32
        char port[10];
        WSADATA wsaData;
        struct addrinfo *result = NULL, *ptr = NULL, hints;

        ZeroMemory(&hints, sizeof (hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;
        hints.ai_flags = AI_PASSIVE;

        if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
            printf("Fatal: Unable to create socket.\n");
            exit(-5);
        }

        if (getaddrinfo(NULL, itoa(args.port, port, 10), &hints, &result) != 0) {
            WSACleanup();
            printf("Fatal: Unable to create socket.\n");
            exit(-5);
        }

        if ((socket_fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol)) == INVALID_SOCKET) {
            WSACleanup();
            freeaddrinfo(result);
            printf("Fatal: Unable to create socket.\n");
            exit(-5);
        }

        if (bind(socket_fd, result->ai_addr, (int)result->ai_addrlen) == SOCKET_ERROR) {
            freeaddrinfo(result);
            closesocket(socket_fd);
            WSACleanup();
            printf("Fatal: Unable to bind socket.\n");
            exit(-5);
        }

        freeaddrinfo(result);

        if (listen(socket_fd, SOMAXCONN ) == SOCKET_ERROR ) {
            closesocket(socket_fd);
            WSACleanup();
            exit(-5);
        }
        
        sim.socket_fd = INVALID_SOCKET;
#else

        socklen_t addrlen = sizeof(struct sockaddr_in);
        struct sockaddr_in server_addr = {0};

        if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("Fatal: Unable to create socket.\n");
            exit(-5);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(args.port);

        if (bind(socket_fd, (struct sockaddr *) &server_addr, addrlen) < 0) {
            printf("Fatal: Unable to bind socket.\n");
            exit(-5);
        }

        FD_ZERO(&rfds);
        FD_SET(socket_fd, &rfds);

        listen(socket_fd, 1);
/*
        struct sockaddr_in client_addr;

        if ((sim.socket_fd = accept(socket_fd, (struct sockaddr *) &client_addr, &addrlen)) < 0) {
            printf("Fatal: Error on socket accept.\n");
            exit(-5);
        }

        FD_SET(sim.socket_fd, &rfds);

        struct timeval t = {
            .tv_sec = 0,
            .tv_usec = 0
        }; 

        setsockopt(sim.socket_fd, SOL_SOCKET, SO_RCVTIMEO, &t, sizeof(t));
*/
#endif
        sim.getchar = sim_socket_in;
        sim.putchar = sim_socket_out;

    } else {
        sim.getchar = platform_poll_stdin;
        sim.putchar = sim_serial_out;
    }

    //launch a thread with the original grbl code.
    plat_thread_t *th = platform_start_thread(grbl_main_thread); 
    if (!th){
        printf("Fatal: Unable to start hardware thread.\n");
        exit(-5);
    }

    // Do not leave EEPROM file in an inconsistent state on ^C.
    atexit(eeprom_close);
    signal(SIGTERM, exithandler);

    // All the stream io and interrupt happen in this thread.
    sim_loop();

    // Graceful exit
    shutdown_simulator();

    platform_kill_thread(th); //need force kill since original main has no return.

    // close the files we opened
	if (args.block_out_file != stdout && args.block_out_file != args.serial_out_file)
		fclose(args.block_out_file);
	if (args.step_out_file != stderr)
		fclose(args.step_out_file);
	if (args.serial_out_file != stdout && args.serial_out_file != args.block_out_file)
		fclose(args.serial_out_file);

    if(args.port) {
#ifdef WIN32
        closesocket(socket_fd);
        WSACleanup();
#else
        if(sim.socket_fd)
            close(sim.socket_fd);
        close(socket_fd);
#endif
    }

    platform_terminate();

    exit(EXIT_SUCCESS);
}
