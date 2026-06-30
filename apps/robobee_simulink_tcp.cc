#include "apps/robobee_simulink_tcp.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robobee::simulink {
namespace {

void ThrowSystemError(const std::string& operation) {
  throw std::runtime_error(operation + " failed: " + std::strerror(errno));
}

}  // namespace

FileDescriptor::FileDescriptor(int fd) : fd_(fd) {}

FileDescriptor::FileDescriptor(FileDescriptor&& other) noexcept
    : fd_(other.fd_) {
  other.fd_ = -1;
}

FileDescriptor& FileDescriptor::operator=(FileDescriptor&& other) noexcept {
  if (this != &other) {
    Reset();
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

FileDescriptor::~FileDescriptor() { Reset(); }

int FileDescriptor::get() const { return fd_; }

bool FileDescriptor::valid() const { return fd_ >= 0; }

int FileDescriptor::release() {
  const int fd = fd_;
  fd_ = -1;
  return fd;
}

void FileDescriptor::Reset(int fd) {
  if (fd_ >= 0) {
    close(fd_);
  }
  fd_ = fd;
}

void SetTcpNoDelay(int fd) {
  const int enabled = 1;
  if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) != 0) {
    ThrowSystemError("setsockopt(TCP_NODELAY)");
  }
}

FileDescriptor ListenOnLocalhost(int port) {
  FileDescriptor listen_fd(socket(AF_INET, SOCK_STREAM, 0));
  if (!listen_fd.valid()) {
    ThrowSystemError("socket");
  }

  const int reuse_addr = 1;
  if (setsockopt(listen_fd.get(), SOL_SOCKET, SO_REUSEADDR, &reuse_addr,
                 sizeof(reuse_addr)) != 0) {
    ThrowSystemError("setsockopt(SO_REUSEADDR)");
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<uint16_t>(port));

  if (bind(listen_fd.get(), reinterpret_cast<const sockaddr*>(&addr),
           sizeof(addr)) != 0) {
    ThrowSystemError("bind");
  }
  if (listen(listen_fd.get(), 1) != 0) {
    ThrowSystemError("listen");
  }
  return listen_fd;
}

FileDescriptor AcceptClient(int listen_fd) {
  sockaddr_in addr{};
  socklen_t addr_len = sizeof(addr);
  FileDescriptor client_fd(
      accept(listen_fd, reinterpret_cast<sockaddr*>(&addr), &addr_len));
  if (!client_fd.valid()) {
    ThrowSystemError("accept");
  }
  SetTcpNoDelay(client_fd.get());
  return client_fd;
}

bool RecvExact(int fd, void* data, int byte_count) {
  char* cursor = static_cast<char*>(data);
  int remaining = byte_count;
  while (remaining > 0) {
    const ssize_t received = recv(fd, cursor, remaining, 0);
    if (received == 0) {
      return false;
    }
    if (received < 0) {
      if (errno == EINTR) continue;
      ThrowSystemError("recv");
    }
    cursor += received;
    remaining -= static_cast<int>(received);
  }
  return true;
}

void SendExact(int fd, const void* data, int byte_count) {
  const char* cursor = static_cast<const char*>(data);
  int remaining = byte_count;
  while (remaining > 0) {
    const ssize_t sent = send(fd, cursor, remaining, 0);
    if (sent < 0) {
      if (errno == EINTR) continue;
      ThrowSystemError("send");
    }
    cursor += sent;
    remaining -= static_cast<int>(sent);
  }
}

}  // namespace robobee::simulink
