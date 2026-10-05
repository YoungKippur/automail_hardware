#ifndef AUTOMAIL_HARDWARE__AUTOMAIL_SERIAL_HARDWARE_HPP_
#define AUTOMAIL_HARDWARE__AUTOMAIL_SERIAL_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace automail_hardware
{

class AutomailSerialHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AutomailSerialHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Abre el puerto serie. Devuelve el fd o -1 en error.
  int openSerial(const std::string & port, int baud_rate);

  // Envía un comando de posición al Arduino:
  //   "S<servo_id>:<degrees>\n"
  // y espera "OK\n".
  bool sendCommand(int servo_id, double angle_rad);

  // Convierte radianes a grados con los límites mapeados al rango del servo.
  int radToDeg(double rad, double joint_lo, double joint_hi,
               int servo_lo, int servo_hi) const;

  // Puerto serie
  std::string serial_port_;   // parámetro: serial_port  (ej: /dev/ttyUSB0)
  int baud_rate_;             // parámetro: baud_rate    (default: 115200)
  int serial_fd_{-1};

  // Nombre y estado de cada joint (mismo orden que el SRDF)
  std::vector<std::string> joint_names_;
  std::vector<double>      hw_positions_;     // estado leído
  std::vector<double>      hw_velocities_;    // siempre 0 (servo PWM)
  std::vector<double>      hw_commands_;      // último comando enviado

  // Límites en grados para cada servo (ángulo mínimo y máximo físico del servo).
  // Se leen de los parámetros en el xacro: servo_min_deg_<joint> / servo_max_deg_<joint>
  std::vector<int> servo_min_deg_;
  std::vector<int> servo_max_deg_;
};

}  // namespace automail_hardware

#endif  // AUTOMAIL_HARDWARE__AUTOMAIL_SERIAL_HARDWARE_HPP_
