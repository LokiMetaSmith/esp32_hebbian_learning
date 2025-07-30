/**
 * @file console.h
 * @brief Header file for the console command interface.
 *
 * This file declares the functions for initializing the console
 * and the main task that runs the command processing loop.
 */

#ifndef CONSOLE_H
#define CONSOLE_H

void initialize_console(void);
void console_task(void *pvParameters);

#endif // CONSOLE_H
