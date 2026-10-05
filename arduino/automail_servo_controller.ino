/*
 * automail_servo_controller.ino
 * ─────────────────────────────
 * Controlador de servos PWM para el brazo AutoMail.
 * Se comunica con el hardware interface de ROS2 por puerto serie USB.
 *
 * Protocolo serie (115200 baud, 8N1):
 *   ROS2 → Arduino:  "S<id>:<grados>\n"    ej: "S0:90\n"
 *   Arduino → ROS2:  "OK\n"                 (confirmación)
 *   Arduino → ROS2:  "ERR\n"               (comando inválido)
 *
 * Mapeo de servo ID → pin PWM:
 *   ID 0  joint1_base_rotation   → pin 3
 *   ID 1  joint2_shoulder_pitch  → pin 5
 *   ID 2  joint3_elbow_pitch     → pin 6
 *   ID 3  joint4_wrist_roll      → pin 9
 *   ID 4  joint5_wrist_pitch     → pin 10
 *   ID 5  joint_finger_left      → pin 11
 *   ID 6  joint_finger_right     → pin A0 (pin 14, controlado por mimic en ROS2)
 *
 * Librería requerida: Servo (incluida en Arduino IDE)
 *
 * Cableado típico servo:
 *   Marrón/Negro → GND
 *   Rojo         → 5V (fuente externa, NO de los 5V del Arduino si son >3 servos)
 *   Naranja/Amarillo → Pin PWM del Arduino
 */

#include <Servo.h>

// ── Número de servos ──────────────────────────────────────────────────────────
static const int NUM_SERVOS = 7;

// ── Pins PWM para cada servo (cambiar según tu cableado) ─────────────────────
static const int SERVO_PINS[NUM_SERVOS] = {3, 5, 6, 9, 10, 11, A0};

// ── Límites físicos de cada servo en grados ──────────────────────────────────
// Ajustar según el rango real de cada servo para no forzar los topes.
static const int SERVO_MIN[NUM_SERVOS] = {  0,   0,   0,   0,   0,   0,   0};
static const int SERVO_MAX[NUM_SERVOS] = {180, 180, 180, 180, 180, 180, 180};

// ── Posición inicial de cada servo al encender ───────────────────────────────
static const int SERVO_HOME[NUM_SERVOS] = {90, 90, 135, 90, 90, 90, 90};

Servo servos[NUM_SERVOS];
String inputBuffer = "";

// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  while (!Serial) { ; }   // espera conexión USB en Arduino Leonardo/Micro

  // Adjuntar servos y mover a posición inicial
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(SERVO_HOME[i]);
    delay(50);             // pequeña pausa entre servos para no pico de corriente
  }

  Serial.println("AUTOMAIL_READY");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
  // Leer bytes disponibles y acumular hasta '\n'
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// processCommand: parsea "S<id>:<grados>" y mueve el servo correspondiente.
// ─────────────────────────────────────────────────────────────────────────────
void processCommand(const String & cmd)
{
  // Formato esperado: "S<id>:<grados>"
  if (cmd.length() < 3 || cmd.charAt(0) != 'S') {
    Serial.println("ERR");
    return;
  }

  int colon = cmd.indexOf(':');
  if (colon < 0) {
    Serial.println("ERR");
    return;
  }

  int servo_id = cmd.substring(1, colon).toInt();
  int degrees  = cmd.substring(colon + 1).toInt();

  // Validar ID
  if (servo_id < 0 || servo_id >= NUM_SERVOS) {
    Serial.println("ERR");
    return;
  }

  // Clampear al rango físico del servo
  degrees = constrain(degrees, SERVO_MIN[servo_id], SERVO_MAX[servo_id]);

  servos[servo_id].write(degrees);

  Serial.println("OK");
}
