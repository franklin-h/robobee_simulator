#pragma once

namespace robobee::simulink {

struct StepRequest {
  // All fields are little-endian IEEE doubles, as sent by MATLAB on macOS.
  // left_voltage_v and right_voltage_v are actuator voltage amplitudes.
  double dt_s{};
  double left_voltage_v{};
  double right_voltage_v{};
};

struct StepResponse {
  double time_s{};
  double x_m{};
  double y_m{};
  double z_m{};
  double roll_rad{};
  double pitch_rad{};
  double yaw_rad{};
};

class FileDescriptor final {
 public:
  FileDescriptor() = default;
  explicit FileDescriptor(int fd);
  FileDescriptor(const FileDescriptor&) = delete;
  FileDescriptor& operator=(const FileDescriptor&) = delete;
  FileDescriptor(FileDescriptor&& other) noexcept;
  FileDescriptor& operator=(FileDescriptor&& other) noexcept;
  ~FileDescriptor();

  int get() const;
  bool valid() const;
  int release();
  void Reset(int fd = -1);

 private:
  int fd_{-1};
};

void SetTcpNoDelay(int fd);
FileDescriptor ListenOnLocalhost(int port);
FileDescriptor AcceptClient(int listen_fd);
bool RecvExact(int fd, void* data, int byte_count);
void SendExact(int fd, const void* data, int byte_count);

}  // namespace robobee::simulink
