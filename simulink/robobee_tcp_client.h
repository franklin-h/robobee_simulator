#ifndef ROBOBEE_TCP_CLIENT_H_
#define ROBOBEE_TCP_CLIENT_H_

#ifdef __cplusplus
extern "C" {
#endif

enum {
  ROBOBEE_TCP_OK = 0,
  ROBOBEE_TCP_ERR_SOCKET = -1,
  ROBOBEE_TCP_ERR_CONNECT = -2,
  ROBOBEE_TCP_ERR_SEND = -3,
  ROBOBEE_TCP_ERR_RECV = -4,
  ROBOBEE_TCP_ERR_BAD_ARG = -5
};

void robobee_tcp_set_endpoint_c(const char* host, int port);
void robobee_tcp_reset_c(void);

/* pose is [time_s, x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad, thrust_z_N,
 * roll_torque_Nm, pitch_torque_Nm, yaw_torque_Nm] -- 11 doubles. */
int robobee_tcp_step_c(double dt_s,
                       double left_voltage_v,
                       double right_voltage_v,
                       double bias_voltage_v,
                       double pose[11]);

#ifdef __cplusplus
}
#endif

#endif  /* ROBOBEE_TCP_CLIENT_H_ */
