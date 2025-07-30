#ifndef MCP_SERVER_COMMANDS_H
#define MCP_SERVER_COMMANDS_H

#include "main.h"

int cmd_set_pos(int argc, char **argv);
int cmd_get_pos(int argc, char **argv);
int cmd_babble_start(int argc, char **argv);
int cmd_babble_stop(int argc, char **argv);
int cmd_set_servo_acceleration(int argc, char **argv);
int cmd_get_current(int argc, char **argv);
int cmd_export_network(int argc, char **argv);
int cmd_set_mode(int argc, char **argv);
int cmd_export_states(int argc, char **argv);
int cmd_import_states(int argc, char **argv);
int cmd_get_stats(int argc, char **argv);
int cmd_rw_set_params(int argc, char **argv);

#endif // MCP_SERVER_COMMANDS_H
