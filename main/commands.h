#ifndef COMMANDS_H
#define COMMANDS_H

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
int cmd_save_network(int argc, char **argv);
int cmd_reset_network(int argc, char **argv);
int cmd_rw_start(int argc, char **argv);
int cmd_rw_stop(int argc, char **argv);
int cmd_set_accel(int argc, char **argv);
int cmd_get_accel_raw(int argc, char **argv);
int cmd_set_max_torque(int argc, char **argv);
int cmd_set_ema_alpha(int argc, char **argv);
int cmd_set_traj_step(int argc, char **argv);
int cmd_set_max_accel(int argc, char **argv);
int cmd_get_servo_acceleration(int argc, char **argv);
int cmd_set_torque_limit(int argc, char **argv);
int cmd_start_map_cal(int argc, char **argv);
int cmd_add_obstacle(int argc, char **argv);
int cmd_clear_obstacles(int argc, char **argv);
int cmd_ik_move(int argc, char **argv);
int cmd_get_snn_stats(int argc, char **argv);
int cmd_get_bt_status(int argc, char **argv);
int cmd_set_vision_class(int argc, char **argv);
int cmd_get_peer_stats(int argc, char **argv);
int cmd_get_workspace_map(int argc, char **argv);
int cmd_teach_gesture(int argc, char **argv);
int cmd_start_record(int argc, char **argv);
int cmd_stop_record(int argc, char **argv);
int cmd_get_full_status(int argc, char **argv);

// Bicameral Weight-Space Programming RPC commands
int cmd_dump_weights(int argc, char **argv);
int cmd_patch_ab(int argc, char **argv);
int cmd_commit_ab(int argc, char **argv);
int cmd_rollback_ab(int argc, char **argv);
int cmd_modulate(int argc, char **argv);
int cmd_get_surface_profile(int argc, char **argv);

#endif // COMMANDS_H
