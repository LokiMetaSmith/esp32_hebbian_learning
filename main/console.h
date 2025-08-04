/**
 * @file console.h
 * @brief Header file for the console command interface.
 *
 * This file declares the functions for initializing the console
 * and the main task that runs the command processing loop.
 */

#ifndef CONSOLE_H
#define CONSOLE_H

#include "argtable3/argtable3.h"

struct export_states_args_t {
    struct arg_int *num_samples;
    struct arg_end *end;
};
extern struct export_states_args_t export_states_args;

struct import_states_args_t {
    struct arg_str *json;
    struct arg_end *end;
};
extern struct import_states_args_t import_states_args;

struct set_mode_args_t {
    struct arg_int *mode;
    struct arg_end *end;
};
extern struct set_mode_args_t set_mode_args;

struct rw_set_params_args_t {
    struct arg_int *delta_pos;
    struct arg_int *interval_ms;
    struct arg_end *end;
};
extern struct rw_set_params_args_t rw_set_params_args;

void initialize_console(void);
void console_task(void *pvParameters);

#endif // CONSOLE_H
