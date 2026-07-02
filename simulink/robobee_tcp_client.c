#include "robobee_tcp_client.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

enum { kDefaultPort = 4242 };
enum { kConnectRetryAttempts = 200 };
enum { kConnectRetrySleepUs = 50000 };

static int g_socket_fd = -1;
static char g_host[64] = "127.0.0.1";
static int g_port = kDefaultPort;

static void robobee_tcp_close(void) {
  if (g_socket_fd >= 0) {
    shutdown(g_socket_fd, SHUT_RDWR);
    close(g_socket_fd);
    g_socket_fd = -1;
  }
}

static int robobee_tcp_send_exact(int fd, const void* data, size_t byte_count) {
  const char* cursor = (const char*)data;
  size_t remaining = byte_count;
  while (remaining > 0) {
    const ssize_t sent = send(fd, cursor, remaining, 0);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      return ROBOBEE_TCP_ERR_SEND;
    }
    if (sent == 0) {
      return ROBOBEE_TCP_ERR_SEND;
    }
    cursor += sent;
    remaining -= (size_t)sent;
  }
  return ROBOBEE_TCP_OK;
}

static int robobee_tcp_recv_exact(int fd, void* data, size_t byte_count) {
  char* cursor = (char*)data;
  size_t remaining = byte_count;
  while (remaining > 0) {
    const ssize_t received = recv(fd, cursor, remaining, 0);
    if (received < 0) {
      if (errno == EINTR) {
        continue;
      }
      return ROBOBEE_TCP_ERR_RECV;
    }
    if (received == 0) {
      return ROBOBEE_TCP_ERR_RECV;
    }
    cursor += received;
    remaining -= (size_t)received;
  }
  return ROBOBEE_TCP_OK;
}

static int robobee_tcp_connect(void) {
  if (g_socket_fd >= 0) {
    return ROBOBEE_TCP_OK;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons((uint16_t)g_port);
  if (inet_pton(AF_INET, g_host, &addr.sin_addr) != 1) {
    return ROBOBEE_TCP_ERR_BAD_ARG;
  }

  for (int attempt = 0; attempt < kConnectRetryAttempts; ++attempt) {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      return ROBOBEE_TCP_ERR_SOCKET;
    }

    int enabled = 1;
    (void)setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
#ifdef SO_NOSIGPIPE
    (void)setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled));
#endif

    if (connect(fd, (const struct sockaddr*)&addr, sizeof(addr)) == 0) {
      g_socket_fd = fd;
      return ROBOBEE_TCP_OK;
    }

    close(fd);
    if (attempt + 1 < kConnectRetryAttempts) {
      usleep(kConnectRetrySleepUs);
    }
  }

  return ROBOBEE_TCP_ERR_CONNECT;
}

void robobee_tcp_set_endpoint_c(const char* host, int port) {
  if (host != NULL && host[0] != '\0') {
    strncpy(g_host, host, sizeof(g_host) - 1);
    g_host[sizeof(g_host) - 1] = '\0';
  }
  if (port > 0 && port <= 65535) {
    g_port = port;
  }
  robobee_tcp_close();
}

void robobee_tcp_reset_c(void) {
  robobee_tcp_close();
}

int robobee_tcp_step_c(double dt_s,
                       double left_voltage_v,
                       double right_voltage_v,
                       double bias_voltage_v,
                       double pose[7]) {
  if (pose == NULL || !isfinite(dt_s) || dt_s <= 0.0 ||
      !isfinite(left_voltage_v) || !isfinite(right_voltage_v) ||
      !isfinite(bias_voltage_v)) {
    return ROBOBEE_TCP_ERR_BAD_ARG;
  }

  int status = robobee_tcp_connect();
  if (status != ROBOBEE_TCP_OK) {
    return status;
  }

  double request[4];
  request[0] = dt_s;
  request[1] = left_voltage_v;
  request[2] = right_voltage_v;
  request[3] = bias_voltage_v;

  for (int attempt = 0; attempt < 2; ++attempt) {
    status = robobee_tcp_send_exact(g_socket_fd, request, sizeof(request));
    if (status == ROBOBEE_TCP_OK) {
      status = robobee_tcp_recv_exact(g_socket_fd, pose, 7 * sizeof(double));
    }
    if (status == ROBOBEE_TCP_OK) {
      return ROBOBEE_TCP_OK;
    }
    robobee_tcp_close();
    if (attempt == 0) {
      status = robobee_tcp_connect();
      if (status != ROBOBEE_TCP_OK) {
        return status;
      }
    }
  }

  return status;
}
