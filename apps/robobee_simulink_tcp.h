#pragma once

namespace robobee::simulink {

struct StepRequest {
  // All fields are little-endian IEEE doubles, as sent by MATLAB on macOS.
  // Voltage fields are pre-amplifier commands. The Drake server applies its
  // configured voltage amplifier gain before using them as actuator voltages.
  // left_voltage_v and right_voltage_v are per-wing waveform samples. After
  // amplification, Drake maps (wing_voltage_v - 100 V) to slider stroke using
  // 200 V peak-to-peak -> 0.6 mm peak-to-peak. bias_voltage_v is logged as the
  // actuator bias/upper-rail command but is not used to reject wing voltages.
  double dt_s{};
  double left_voltage_v{};
  double right_voltage_v{};
  double bias_voltage_v{};
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
void SetReceiveTimeout(int fd, double timeout_s);
FileDescriptor ListenOnLocalhost(int port);
FileDescriptor AcceptClient(int listen_fd);
bool RecvExact(int fd, void* data, int byte_count);
void SendExact(int fd, const void* data, int byte_count);

}  // namespace robobee::simulink
