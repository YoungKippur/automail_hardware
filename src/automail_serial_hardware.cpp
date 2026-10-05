#include "automail_hardware/automail_serial_hardware.hpp"

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace automail_hardware
{

// ──────────────────────────────────────────────────────────────────────────────
// on_init: se llama cuando ros2_control carga el plugin.
// Lee los parámetros del xacro y valida la info del hardware.
// ──────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
AutomailSerialHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parámetros del <hardware> en el xacro
  serial_port_ = info_.hardware_parameters.count("serial_port") ?
    info_.hardware_parameters.at("serial_port") : "/dev/ttyUSB0";

  baud_rate_ = info_.hardware_parameters.count("baud_rate") ?
    std::stoi(info_.hardware_parameters.at("baud_rate")) : 115200;

  // Reservar vectores por el número de joints declarados
  const size_t n = info_.joints.size();
  hw_positions_.resize(n, 0.0);
  hw_velocities_.resize(n, 0.0);
  hw_commands_.resize(n, 0.0);
  servo_min_deg_.resize(n, 0);
  servo_max_deg_.resize(n, 180);

  for (size_t i = 0; i < n; ++i) {
    const auto & j = info_.joints[i];
    joint_names_.push_back(j.name);

    // Límites físicos del servo en grados (pueden diferir del rango URDF)
    // Se declaran como parámetros en el xacro:
    //   <param name="servo_min_deg">0</param>
    //   <param name="servo_max_deg">180</param>
    if (j.parameters.count("servo_min_deg")) {
      servo_min_deg_[i] = std::stoi(j.parameters.at("servo_min_deg"));
    }
    if (j.parameters.count("servo_max_deg")) {
      servo_max_deg_[i] = std::stoi(j.parameters.at("servo_max_deg"));
    }

    // Validar que cada joint tiene command_interface "position"
    if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(rclcpp::get_logger("AutomailSerialHardware"),
        "Joint '%s' debe tener exactamente 1 command_interface 'position'.",
        j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("AutomailSerialHardware"),
    "Inicializado con %zu joints. Puerto: %s @ %d baud.",
    n, serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────────────────────
// on_configure: abre el puerto serie.
// ──────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
AutomailSerialHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  serial_fd_ = openSerial(serial_port_, baud_rate_);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("AutomailSerialHardware"),
      "No se pudo abrir el puerto serie '%s'. Verificá que el Arduino esté conectado "
      "y que tengas permisos (sudo usermod -aG dialout $USER).",
      serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // El Arduino resetea al abrirse el puerto; esperamos que arranque
  rclcpp::sleep_for(std::chrono::milliseconds(2000));

  RCLCPP_INFO(rclcpp::get_logger("AutomailSerialHardware"),
    "Puerto serie '%s' abierto correctamente.", serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────────────────────
// on_activate: mueve todos los servos a posición inicial (0 rad → centro).
// ──────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
AutomailSerialHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hw_commands_[i] = hw_positions_[i];  // arranca desde posición actual
  }

  RCLCPP_INFO(rclcpp::get_logger("AutomailSerialHardware"),
    "Hardware activado. Servos listos.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────────────────────
// on_deactivate: cierra el puerto serie.
// ──────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
AutomailSerialHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("AutomailSerialHardware"),
    "Puerto serie cerrado.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────────────────────
// export_state_interfaces / export_command_interfaces
// ──────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
AutomailSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    ifaces.emplace_back(joint_names_[i],
      hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    ifaces.emplace_back(joint_names_[i],
      hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
AutomailSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    ifaces.emplace_back(joint_names_[i],
      hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return ifaces;
}

// ──────────────────────────────────────────────────────────────────────────────
// read: los servos PWM no tienen feedback de posición.
// Reportamos el último comando enviado como estado (posición asumida).
// ──────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type
AutomailSerialHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = hw_commands_[i];
    hw_velocities_[i] = 0.0;
  }
  return hardware_interface::return_type::OK;
}

// ──────────────────────────────────────────────────────────────────────────────
// write: envía cada comando de posición al Arduino por serie.
// Solo envía si el comando cambió más de 0.01 rad para no saturar el puerto.
// ──────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type
AutomailSerialHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // Límites del joint del URDF
    // En ROS Humble, min/max son std::string, no std::optional<double>
    const auto & j = info_.joints[i];
    double joint_lo = -M_PI, joint_hi = M_PI;
    if (!j.command_interfaces[0].min.empty()) {
      joint_lo = std::stod(j.command_interfaces[0].min);
    }
    if (!j.command_interfaces[0].max.empty()) {
      joint_hi = std::stod(j.command_interfaces[0].max);
    }

    if (!sendCommand(static_cast<int>(i), hw_commands_[i])) {
      RCLCPP_WARN(rclcpp::get_logger("AutomailSerialHardware"),
        "Error enviando comando al servo %zu ('%s')",
        i, joint_names_[i].c_str());
    }
  }

  return hardware_interface::return_type::OK;
}

// ──────────────────────────────────────────────────────────────────────────────
// Helpers
// ──────────────────────────────────────────────────────────────────────────────

int AutomailSerialHardware::openSerial(const std::string & port, int baud_rate)
{
  int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    return -1;
  }

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    return -1;
  }

  // Velocidad
  speed_t speed = B115200;
  if (baud_rate == 9600)   speed = B9600;
  if (baud_rate == 57600)  speed = B57600;
  if (baud_rate == 115200) speed = B115200;

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1, sin control de flujo
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
  tty.c_oflag &= ~OPOST;

  // Lectura bloqueante con timeout 1 s
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 10;   // 1 segundo

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    return -1;
  }

  return fd;
}

bool AutomailSerialHardware::sendCommand(int servo_id, double angle_rad)
{
  // Mapeo: los límites del joint en URDF vienen de joint_limits.yaml
  // Aquí usamos los límites físicos del servo definidos en el xacro
  const double joint_lo = -M_PI;   // fallback genérico
  const double joint_hi =  M_PI;

  int deg = radToDeg(angle_rad, joint_lo, joint_hi,
    servo_min_deg_[servo_id], servo_max_deg_[servo_id]);

  // Protocolo: "S<id>:<grados>\n"
  // Ejemplo:   "S0:90\n"
  std::ostringstream cmd;
  cmd << "S" << servo_id << ":" << deg << "\n";
  const std::string s = cmd.str();

  ssize_t written = ::write(serial_fd_, s.c_str(), s.size());
  if (written < 0 || static_cast<size_t>(written) != s.size()) {
    return false;
  }

  // Espera "OK\n" del Arduino (timeout 200 ms)
  char buf[16];
  ssize_t n = ::read(serial_fd_, buf, sizeof(buf) - 1);
  if (n > 0) {
    buf[n] = '\0';
    // Aceptamos cualquier respuesta con "OK"
    return (std::strstr(buf, "OK") != nullptr);
  }

  return false;
}

int AutomailSerialHardware::radToDeg(
  double rad, double joint_lo, double joint_hi,
  int servo_lo, int servo_hi) const
{
  // Clampear al rango del joint
  rad = std::max(joint_lo, std::min(joint_hi, rad));

  // Mapeo lineal del rango URDF al rango físico del servo
  double t = (rad - joint_lo) / (joint_hi - joint_lo);
  int deg = static_cast<int>(std::round(servo_lo + t * (servo_hi - servo_lo)));

  // Clampear al rango físico del servo
  deg = std::max(servo_lo, std::min(servo_hi, deg));
  return deg;
}

}  // namespace automail_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  automail_hardware::AutomailSerialHardware,
  hardware_interface::SystemInterface)