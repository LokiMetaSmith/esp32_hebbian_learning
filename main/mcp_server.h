/**
 * @file mcp_server.h
 * @brief Header file for the MCP (Motion Control Program) Server.
 *
 * This file declares the functions for initializing and running the Wi-Fi
 * access point and the TCP server that listens for commands from a client.
 * It provides the main interface for external control of the robot.
 */

#ifndef MCP_SERVER_H
#define MCP_SERVER_H

#include "main.h"

void mcp_server_init(void);

#endif // MCP_SERVER_H
