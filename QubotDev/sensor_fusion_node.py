#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import serial.tools.list_ports
import time
import threading
from QubotDev import tfmplus as tfmP


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__("sensor_fusion_node")

        # Configuración de sensores conocidos
        self.sensor_configs = {
            "urm04_0x13": {"baudrate": 19200, "address": 0x13, "name": "der_front"},
            "urm04_0x11_bus1": {
                "baudrate": 19200,
                "address": 0x11,
                "name": "izq_front",
            },
            "urm04_0x11_bus2": {"baudrate": 19200, "address": 0x11, "name": "izq_back"},
            "urm37": {"baudrate": 9600, "name": "der_back"},
            "lidar": {"baudrate": 115200, "name": "lidar"},
        }

        # Crear publicadores para cada sensor
        self.sensor_publishers = {
            "izq_back": self.create_publisher(Range, "/sensor/izq_back", 10),
            "izq_front": self.create_publisher(Range, "/sensor/izq_front", 10),
            "der_front": self.create_publisher(Range, "/sensor/der_front", 10),
            "der_back": self.create_publisher(Range, "/sensor/der_back", 10),
            "lidar": self.create_publisher(Range, "/sensor/lidar", 10),
        }

        # Estado de sensores
        self.sensor_status = {}
        self.error_counts = {}
        self.max_errors = 5

        # Mapeo de puertos detectados
        self.detected_ports = {}
        self.serial_ports = {}

        # Threading para lectura independiente de sensores
        self.sensor_threads = {}
        self.thread_locks = {}
        self.running = True

        # Detectar y configurar sensores automáticamente
        self.auto_detect_sensors()

        # Configurar temporizadores
        if self.detected_ports:
            # Timer para reconexión automática cada 30 segundos
            self.reconnect_timer = self.create_timer(30.0, self.attempt_reconnections)

            # Iniciar threads independientes para cada tipo de sensor
            self.start_sensor_threads()
        else:
            self.get_logger().error(
                "No se detectaron sensores. Reintentando en 10 segundos..."
            )
            self.retry_timer = self.create_timer(10.0, self.retry_detection)

    def get_available_ports(self):
        """Obtiene lista de puertos serie disponibles con información adicional"""
        ports = []
        for port in serial.tools.list_ports.comports():
            if "ttyUSB" in port.device or "ttyACM" in port.device:
                # Obtener información del hub USB si está disponible
                port_info = {
                    "device": port.device,
                    "description": port.description,
                    "hwid": port.hwid,
                    "vid": port.vid,
                    "pid": port.pid,
                }
                ports.append(port_info)
                self.get_logger().info(
                    f"Puerto encontrado: {port.device} - {port.description}"
                )

        return sorted(ports, key=lambda x: x["device"])

    def test_lidar_sensor_robust(self, port_info, baudrate):
        """Prueba LiDAR con configuración robusta para hubs USB"""
        port = port_info["device"]
        try:
            # Configuración específica para LiDAR en hub
            self.get_logger().info(
                f"Probando LiDAR en {port} con configuración robusta..."
            )

            # Intentar múltiples veces con diferentes timeouts
            for attempt in range(5):
                try:
                    # Configurar puerto serie con parámetros específicos para hub
                    ser = serial.Serial(
                        port=port,
                        baudrate=baudrate,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=1.0,  # Timeout más largo para hubs
                        write_timeout=1.0,
                        inter_byte_timeout=0.1,
                    )

                    # Esperar estabilización del puerto
                    time.sleep(0.5)

                    # Limpiar buffers
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()

                    ser.close()

                    # Ahora probar con tfmP
                    if tfmP.begin(port, baudrate):
                        # Probar múltiples lecturas
                        success_count = 0
                        for test in range(5):
                            if tfmP.getData():
                                success_count += 1
                            time.sleep(0.1)

                        if success_count >= 2:  # Al menos 2 lecturas exitosas
                            self.get_logger().info(
                                f"LiDAR detectado exitosamente en {port}"
                            )
                            return True

                    time.sleep(0.5)  # Esperar entre intentos

                except Exception as e:
                    self.get_logger().debug(
                        f"Intento {attempt + 1} fallido para LiDAR: {str(e)}"
                    )
                    time.sleep(0.5)

            return False

        except Exception as e:
            self.get_logger().debug(f"Error general probando LiDAR en {port}: {str(e)}")
            return False

    def test_urm37_sensor_robust(self, port_info, baudrate):
        """Prueba URM37 con configuración robusta para hubs USB"""
        port = port_info["device"]
        try:
            # Configuración específica para URM37 en hub
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.8,  # Timeout ajustado para hub
                write_timeout=0.5,
                inter_byte_timeout=0.05,
            )

            time.sleep(0.3)  # Tiempo adicional para hub

            # Intentar comunicación múltiples veces
            for attempt in range(5):
                try:
                    # Limpiar buffers antes de cada intento
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()

                    cmd = self.build_command_urm37()
                    ser.write(bytearray(cmd))
                    time.sleep(0.2)  # Tiempo extendido para hub

                    if ser.in_waiting >= 4:
                        response = ser.read(4)
                        if len(response) == 4 and response[0] == 0x22:
                            # Verificar checksum
                            expected_checksum = (
                                response[0] + response[1] + response[2]
                            ) & 0xFF
                            if response[3] == expected_checksum:
                                ser.close()
                                return True

                    time.sleep(0.1)

                except Exception:
                    continue

            ser.close()
            return False

        except Exception:
            return False

    def auto_detect_sensors(self):
        """Detecta automáticamente todos los sensores con manejo mejorado de hubs"""
        self.get_logger().info("Iniciando detección automática de sensores...")

        available_ports = self.get_available_ports()
        if not available_ports:
            self.get_logger().error("No se encontraron puertos USB disponibles")
            return

        detected_sensors = []

        # Estrategia: Detectar primero sensores de menor baudrate (menos interferencia)

        # 1. Buscar URM37 primero (baudrate más bajo)
        self.get_logger().info("Buscando URM37...")
        for port_info in available_ports:
            if self.test_urm37_sensor_robust(port_info, 9600):
                self.detected_ports["der_back"] = {
                    "port": port_info["device"],
                    "type": "urm37",
                    "baudrate": 9600,
                    "port_info": port_info,
                }
                detected_sensors.append(f"URM37 (der_back) en {port_info['device']}")
                break

        # 2. Buscar URM04 sensores
        self.get_logger().info("Buscando URM04 con dirección 0x13...")
        for port_info in available_ports:
            # Saltar puertos ya usados
            if any(
                config["port"] == port_info["device"]
                for config in self.detected_ports.values()
            ):
                continue

            if self.test_urm04_sensor(port_info["device"], 19200, 0x13):
                self.detected_ports["der_front"] = {
                    "port": port_info["device"],
                    "type": "urm04",
                    "address": 0x13,
                    "baudrate": 19200,
                    "port_info": port_info,
                }
                detected_sensors.append(
                    f"URM04 0x13 (der_front) en {port_info['device']}"
                )

                # Buscar en el mismo bus el 0x11
                if self.test_urm04_sensor(port_info["device"], 19200, 0x11):
                    self.detected_ports["izq_front"] = {
                        "port": port_info["device"],
                        "type": "urm04",
                        "address": 0x11,
                        "baudrate": 19200,
                        "port_info": port_info,
                    }
                    detected_sensors.append(
                        f"URM04 0x11 (izq_front) en {port_info['device']}"
                    )
                break

        # 3. Buscar segundo URM04 0x11
        self.get_logger().info("Buscando segundo URM04 con dirección 0x11...")
        for port_info in available_ports:
            if any(
                config["port"] == port_info["device"]
                for config in self.detected_ports.values()
            ):
                continue

            if self.test_urm04_sensor(port_info["device"], 19200, 0x11):
                self.detected_ports["izq_back"] = {
                    "port": port_info["device"],
                    "type": "urm04",
                    "address": 0x11,
                    "baudrate": 19200,
                    "port_info": port_info,
                }
                detected_sensors.append(
                    f"URM04 0x11 (izq_back) en {port_info['device']}"
                )
                break

        # 4. Buscar LiDAR al final (baudrate más alto, más interferencia)
        self.get_logger().info("Buscando LiDAR con configuración robusta...")
        for port_info in available_ports:
            if any(
                config["port"] == port_info["device"]
                for config in self.detected_ports.values()
            ):
                continue

            if self.test_lidar_sensor_robust(port_info, 115200):
                self.detected_ports["lidar"] = {
                    "port": port_info["device"],
                    "type": "lidar",
                    "baudrate": 115200,
                    "port_info": port_info,
                }
                detected_sensors.append(f"LiDAR en {port_info['device']}")
                break

        # Reportar resultados
        if detected_sensors:
            self.get_logger().info("Sensores detectados:")
            for sensor in detected_sensors:
                self.get_logger().info(f"  - {sensor}")

            # Inicializar sensores detectados
            self.initialize_detected_sensors()
        else:
            self.get_logger().error("No se detectaron sensores")

    def initialize_lidar_robust(self, port, baudrate):
        """Inicializa LiDAR con configuración robusta para hubs"""
        try:
            # Configuración específica para LiDAR en hub USB
            self.get_logger().info(
                f"Inicializando LiDAR en {port} con configuración robusta..."
            )

            # Reintentos con delays progresivos
            for attempt in range(3):
                try:
                    if tfmP.begin(port, baudrate):
                        # Probar estabilidad con múltiples lecturas
                        success_count = 0
                        for test in range(10):
                            if tfmP.getData():
                                success_count += 1
                            time.sleep(0.05)

                        if success_count >= 5:  # Al menos 50% de éxito
                            self.get_logger().info(
                                f"LiDAR inicializado exitosamente en {port}"
                            )
                            return True
                        else:
                            self.get_logger().warn(
                                f"LiDAR inestable en {port}, solo {success_count}/10 lecturas exitosas"
                            )

                    time.sleep(1.0 * (attempt + 1))  # Delay progresivo

                except Exception as e:
                    self.get_logger().debug(
                        f"Intento {attempt + 1} de inicialización LiDAR falló: {str(e)}"
                    )
                    time.sleep(0.5)

            return False

        except Exception as e:
            self.get_logger().error(f"Error general inicializando LiDAR: {str(e)}")
            return False

    def start_sensor_threads(self):
        """Inicia threads independientes para cada tipo de sensor"""
        self.get_logger().info("Iniciando threads independientes para sensores...")

        # Thread para LiDAR (si existe)
        if "lidar" in self.detected_ports:
            self.thread_locks["lidar"] = threading.Lock()
            self.sensor_threads["lidar"] = threading.Thread(
                target=self.lidar_thread, daemon=True
            )
            self.sensor_threads["lidar"].start()

        # Thread para sensores serie (URM04 y URM37)
        serial_sensors = [
            name
            for name, config in self.detected_ports.items()
            if config["type"] in ["urm04", "urm37"]
        ]

        if serial_sensors:
            self.thread_locks["serial"] = threading.Lock()
            self.sensor_threads["serial"] = threading.Thread(
                target=self.serial_sensors_thread, daemon=True
            )
            self.sensor_threads["serial"].start()

    def lidar_thread(self):
        """Thread independiente para LiDAR"""
        config = self.detected_ports["lidar"]

        # Inicializar LiDAR con configuración robusta
        if not self.initialize_lidar_robust(config["port"], config["baudrate"]):
            self.sensor_status["lidar"] = False
            self.error_counts["lidar"] = self.max_errors
            return

        self.sensor_status["lidar"] = True
        self.error_counts["lidar"] = 0

        while self.running:
            try:
                distance = self.get_distance_lidar()

                if distance is not None:
                    self.handle_sensor_success("lidar")
                    frame_id = "lidar_frame"
                    self.publish_range("lidar", distance, frame_id)
                else:
                    self.handle_sensor_error("lidar")

                time.sleep(0.25)  # 10 Hz para LiDAR

            except Exception as e:
                self.get_logger().debug(f"Error en thread LiDAR: {str(e)}")
                time.sleep(0.5)

    def serial_sensors_thread(self):
        """Thread independiente para sensores serie"""
        # Inicializar sensores serie
        self.initialize_serial_sensors()

        while self.running:
            try:
                for sensor_name, config in self.detected_ports.items():
                    if config["type"] in ["urm04", "urm37"] and self.sensor_status.get(
                        sensor_name, False
                    ):
                        distance = None

                        ser = self.serial_ports.get(config["port"])
                        if ser:
                            if config["type"] == "urm37":
                                distance = self.get_distance_urm37(ser)
                            elif config["type"] == "urm04":
                                distance = self.get_distance_urm04(
                                    ser, config["address"]
                                )

                        # Procesar resultado
                        if distance is not None:
                            self.handle_sensor_success(sensor_name)
                            frame_id = f"{sensor_name}_frame"
                            self.publish_range(sensor_name, distance, frame_id)
                        else:
                            self.handle_sensor_error(sensor_name)

                time.sleep(0.0)  # 2 Hz para sensores serie

            except Exception as e:
                self.get_logger().debug(f"Error en thread sensores serie: {str(e)}")
                time.sleep(1.0)

    def initialize_detected_sensors(self):
        """Inicializa los sensores detectados"""
        for sensor_name, config in self.detected_ports.items():
            if config["type"] == "lidar":
                # El LiDAR se inicializará en su propio thread
                self.sensor_status["lidar"] = False  # Se activará en el thread
                self.error_counts["lidar"] = 0
            else:
                # Los sensores serie se inicializarán en su thread
                self.sensor_status[sensor_name] = False  # Se activará en el thread
                self.error_counts[sensor_name] = 0

    def initialize_serial_sensors(self):
        """Inicializa sensores serie con configuración robusta"""
        for sensor_name, config in self.detected_ports.items():
            if config["type"] in ["urm04", "urm37"]:
                try:
                    port = config["port"]
                    if port not in self.serial_ports:
                        # Configuración robusta para hub USB
                        self.serial_ports[port] = serial.Serial(
                            port=port,
                            baudrate=config["baudrate"],
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=0.8,
                            write_timeout=0.5,
                            inter_byte_timeout=0.05,
                        )
                        time.sleep(0.2)  # Tiempo para estabilización

                    self.sensor_status[sensor_name] = True
                    self.error_counts[sensor_name] = 0
                    self.get_logger().info(
                        f"Sensor {sensor_name} inicializado en {port}"
                    )

                except serial.SerialException as e:
                    self.get_logger().error(
                        f"Error inicializando {sensor_name}: {str(e)}"
                    )
                    self.sensor_status[sensor_name] = False
                    self.error_counts[sensor_name] = self.max_errors

    # [Resto de métodos permanecen iguales]
    def test_urm04_sensor(self, port, baudrate, address):
        """Prueba si hay un sensor URM04 en el puerto con la dirección especificada"""
        try:
            ser = serial.Serial(port, baudrate, timeout=0.5)
            time.sleep(0.1)

            for attempt in range(3):
                try:
                    cmd1 = self.build_command_urm04(address, 0x01)
                    ser.write(bytearray(cmd1))
                    time.sleep(0.05)

                    cmd2 = self.build_command_urm04(address, 0x02)
                    ser.write(bytearray(cmd2))
                    time.sleep(0.1)

                    if ser.in_waiting >= 8:
                        response = ser.read(8)
                        if (
                            len(response) == 8
                            and response[0] == 0x55
                            and response[1] == 0xAA
                        ):
                            ser.close()
                            return True

                    ser.reset_input_buffer()
                    time.sleep(0.1)

                except Exception:
                    continue

            ser.close()
            return False

        except Exception:
            return False

    def calculate_checksum(self, command):
        return sum(command[:5]) & 0xFF

    def build_command_urm04(self, sensor_address, command_type):
        command = [0x55, 0xAA, sensor_address, 0x00, command_type, 0x00]
        command[5] = self.calculate_checksum(command)
        return command

    def build_command_urm37(self):
        command = [0x22, 0x00, 0x00]
        checksum = sum(command) & 0xFF
        command.append(checksum)
        return command

    def send_command(self, ser, command):
        """Envía comando con manejo de errores"""
        try:
            ser.write(bytearray(command))
            time.sleep(0.05)
            return True
        except Exception as e:
            self.get_logger().debug(f"Error enviando comando: {str(e)}")
            return False

    def read_response(self, ser, expected_length):
        try:
            start_time = time.time()
            while ser.in_waiting < expected_length:
                if time.time() - start_time > 0.2:  # Timeout extendido
                    return None
            response = ser.read(expected_length)
            return response if len(response) == expected_length else None
        except Exception:
            return None

    def get_distance_urm04(self, ser, sensor_address):
        """Lee distancia URM04 con manejo robusto de errores"""
        try:
            if not self.send_command(
                ser, self.build_command_urm04(sensor_address, 0x01)
            ):
                return None
            time.sleep(0.05)
            if not self.send_command(
                ser, self.build_command_urm04(sensor_address, 0x02)
            ):
                return None

            response = self.read_response(ser, 8)
            if response and len(response) == 8:
                distance = (response[5] << 8) | response[6]
                return distance / 100.0 if distance != 0xFFFF else None
        except Exception as e:
            self.get_logger().debug(f"Error leyendo URM04: {str(e)}")
        return None

    def get_distance_urm37(self, ser):
        """Lee distancia URM37 con manejo robusto de errores"""
        try:
            if not self.send_command(ser, self.build_command_urm37()):
                return None
            time.sleep(0.15)  # Tiempo extendido

            if ser.in_waiting >= 4:
                response = ser.read(4)
                header, highbyte, lowbyte, checksum = response
                if header == 0x22 and checksum == (
                    (header + highbyte + lowbyte) & 0xFF
                ):
                    return ((highbyte << 8) | lowbyte) / 100.0
        except Exception as e:
            self.get_logger().debug(f"Error leyendo URM37: {str(e)}")
        return None

    def get_distance_lidar(self):
        """Lee distancia LiDAR con manejo robusto de errores"""
        try:
            if tfmP.getData():
                return tfmP.dist / 100.0
        except Exception as e:
            self.get_logger().debug(f"Error leyendo LiDAR: {str(e)}")
        return None

    def handle_sensor_error(self, sensor_name):
        """Maneja errores de sensores de forma consistente"""
        self.error_counts[sensor_name] = self.error_counts.get(sensor_name, 0) + 1

        if self.error_counts[sensor_name] >= self.max_errors:
            if self.sensor_status.get(sensor_name, True):
                self.get_logger().warn(
                    f"Sensor {sensor_name} marcado como fallido tras {self.max_errors} errores"
                )
                self.sensor_status[sensor_name] = False

    def handle_sensor_success(self, sensor_name):
        """Resetea contador de errores en lectura exitosa"""
        if self.error_counts.get(sensor_name, 0) > 0:
            self.error_counts[sensor_name] = 0
            if not self.sensor_status.get(sensor_name, True):
                self.sensor_status[sensor_name] = True
                self.get_logger().info(f"Sensor {sensor_name} recuperado exitosamente")

    def publish_range(self, sensor_name, distance, frame_id):
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = frame_id

        if sensor_name == "lidar":
            range_msg.radiation_type = Range.INFRARED
        else:
            range_msg.radiation_type = Range.ULTRASOUND

        range_msg.field_of_view = 0.26
        range_msg.min_range = 0.2
        range_msg.max_range = 5.0
        range_msg.range = distance if distance is not None else float("inf")

        self.sensor_publishers[sensor_name].publish(range_msg)
        self.get_logger().info(f"{sensor_name}: {distance:.2f} m")

    def retry_detection(self):
        """Reintenta la detección de sensores"""
        self.get_logger().info("Reintentando detección de sensores...")
        self.auto_detect_sensors()

        if self.detected_ports:
            self.retry_timer.cancel()
            self.reconnect_timer = self.create_timer(30.0, self.attempt_reconnections)
            self.start_sensor_threads()

    def attempt_reconnections(self):
        """Intenta reconectar sensores fallidos"""
        self.get_logger().info("Verificando sensores desconectados...")

        for sensor_name, config in self.detected_ports.items():
            if not self.sensor_status.get(sensor_name, False):
                if config["type"] == "lidar":
                    if self.initialize_lidar_robust(config["port"], config["baudrate"]):
                        self.get_logger().info(
                            f"LiDAR reconectado exitosamente en {config['port']}"
                        )
                else:
                    try:
                        port = config["port"]
                        if port in self.serial_ports:
                            self.serial_ports[port].close()

                        self.serial_ports[port] = serial.Serial(
                            port=port,
                            baudrate=config["baudrate"],
                            timeout=0.8,
                            write_timeout=0.5,
                        )
                        self.sensor_status[sensor_name] = True
                        self.error_counts[sensor_name] = 0
                        self.get_logger().info(
                            f"Sensor {sensor_name} reconectado exitosamente en {port}"
                        )

                    except serial.SerialException:
                        self.get_logger().debug(
                            f"Sensor {sensor_name} aún no disponible"
                        )

    def cleanup(self):
        """Limpieza segura"""
        self.running = False

        # Esperar que terminen los threads
        for thread in self.sensor_threads.values():
            if thread.is_alive():
                thread.join(timeout=1.0)

        # Cerrar puertos serie
        for port, ser in self.serial_ports.items():
            try:
                if ser.is_open:
                    ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SensorFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Cerrando sensores...")
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
