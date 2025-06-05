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
        self.sensor_status = {
            config["name"]: False for config in self.sensor_configs.values()
        }
        self.error_counts = {
            config["name"]: 0 for config in self.sensor_configs.values()
        }
        self.last_reading_status = {
            config["name"]: "Not initialized" for config in self.sensor_configs.values()
        }
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
            self.reconnect_timer = self.create_timer(30.0, self.attempt_reconnections)
            self.status_report_timer = self.create_timer(5.0, self.report_sensor_status)
            self.start_sensor_threads()
        else:
            self.get_logger().error(
                "No se detectaron sensores. Reintentando en 10 segundos..."
            )
            self.retry_timer = self.create_timer(10.0, self.retry_detection)

    def attempt_reconnections(self):
        """Intenta reconectar sensores fallidos y re-detectar sensores faltantes"""
        self.get_logger().info("Verificando sensores desconectados...")

        # Intentar reconectar sensores detectados pero inactivos
        for sensor_name, config in self.detected_ports.items():
            if not self.sensor_status.get(sensor_name, False):
                if config["type"] == "lidar":
                    if self.initialize_lidar_robust(config["port"], config["baudrate"]):
                        self.get_logger().info(
                            f"LiDAR reconectado exitosamente en {config['port']}"
                        )
                        self.last_reading_status["lidar"] = "Reconnected"
                else:
                    try:
                        port = config["port"]
                        if (
                            port in self.serial_ports
                            and self.serial_ports[port].is_open
                        ):
                            self.serial_ports[port].close()

                        self.serial_ports[port] = serial.Serial(
                            port=port,
                            baudrate=config["baudrate"],
                            timeout=0.8,
                            write_timeout=0.5,
                        )
                        self.sensor_status[sensor_name] = True
                        self.error_counts[sensor_name] = 0
                        self.last_reading_status[sensor_name] = "Reconnected"
                        self.get_logger().info(
                            f"Sensor {sensor_name} reconectado exitosamente en {port}"
                        )

                    except serial.SerialException as e:
                        self.get_logger().debug(
                            f"Sensor {sensor_name} aún no disponible: {str(e)}"
                        )
                        self.last_reading_status[sensor_name] = (
                            f"Reconnection failed: {str(e)}"
                        )

        # Intentar re-detectar sensores faltantes
        for config_key, config in self.sensor_configs.items():
            sensor_name = config["name"]
            if sensor_name not in self.detected_ports:
                self.try_detect_sensor(sensor_name, config)

    def redetect_missing_sensors(self):
        """Re-detecta sensores que no fueron encontrados inicialmente"""
        missing_sensors = []

        # Verificar qué sensores faltan
        for config_key, config in self.sensor_configs.items():
            sensor_name = config["name"]
            if sensor_name not in self.detected_ports:
                missing_sensors.append(sensor_name)

        if missing_sensors:
            self.get_logger().info(
                f"Re-detectando sensores faltantes: {missing_sensors}"
            )

            available_ports = self.get_available_ports()

            # Buscar específicamente izq_back si falta
            if "izq_back" in missing_sensors:
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
                        self.get_logger().info(
                            f"URM04 0x11 (izq_back) re-detectado en {port_info['device']}"
                        )

                        # Inicializar el sensor
                        try:
                            if port_info["device"] not in self.serial_ports:
                                self.serial_ports[port_info["device"]] = serial.Serial(
                                    port=port_info["device"],
                                    baudrate=19200,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    timeout=0.8,
                                    write_timeout=0.5,
                                    inter_byte_timeout=0.05,
                                )
                                time.sleep(0.2)

                            self.sensor_status["izq_back"] = True
                            self.error_counts["izq_back"] = 0
                            self.last_reading_status["izq_back"] = (
                                "Re-detected and initialized"
                            )
                            self.get_logger().info(
                                "Sensor izq_back inicializado exitosamente"
                            )

                        except Exception as e:
                            self.get_logger().error(
                                f"Error inicializando izq_back re-detectado: {str(e)}"
                            )

                        break

    def get_available_ports(self):
        """Obtiene lista de puertos serie disponibles con información adicional"""
        ports = []
        for port in serial.tools.list_ports.comports():
            if "ttyUSB" in port.device or "ttyACM" in port.device:
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
            self.get_logger().info(
                f"Probando LiDAR en {port} con configuración robusta..."
            )

            for attempt in range(5):
                try:
                    ser = serial.Serial(
                        port=port,
                        baudrate=baudrate,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=1.0,
                        write_timeout=1.0,
                        inter_byte_timeout=0.1,
                    )

                    time.sleep(0.5)
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    ser.close()

                    if tfmP.begin(port, baudrate):
                        success_count = 0
                        for test in range(5):
                            if tfmP.getData():
                                success_count += 1
                            time.sleep(0.1)

                        if success_count >= 2:
                            self.get_logger().info(
                                f"LiDAR detectado exitosamente en {port}"
                            )
                            return True

                    time.sleep(0.5)

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
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.8,
                write_timeout=0.5,
                inter_byte_timeout=0.05,
            )

            time.sleep(0.3)

            for attempt in range(5):
                try:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()

                    cmd = self.build_command_urm37()
                    ser.write(bytearray(cmd))
                    time.sleep(0.2)

                    if ser.in_waiting >= 4:
                        response = ser.read(4)
                        if len(response) == 4 and response[0] == 0x22:
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
        """Detecta automáticamente todos los sensores con lógica corregida"""
        self.get_logger().info("Iniciando detección automática de sensores...")

        available_ports = self.get_available_ports()
        if not available_ports:
            self.get_logger().error("No se encontraron puertos USB disponibles")
            return

        detected_sensors = []

        # Inicializar estado y contadores para todos los sensores
        for sensor_name in [config["name"] for config in self.sensor_configs.values()]:
            self.sensor_status[sensor_name] = False
            self.error_counts[sensor_name] = 0
            self.last_reading_status[sensor_name] = "Not initialized"

        # 1. Buscar URM37 (der_back)
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

        # 2. Identificar puertos con ambos sensores URM04 (hub derecho)
        hub_derecho_port = None
        self.get_logger().info(
            "Buscando hub con URM04 0x13 y 0x11 (der_front + izq_front)..."
        )
        for port_info in available_ports:
            if any(
                config["port"] == port_info["device"]
                for config in self.detected_ports.values()
            ):
                continue

            # Verificar si este puerto tiene AMBOS sensores (0x13 Y 0x11)
            has_0x13 = self.test_urm04_sensor(port_info["device"], 19200, 0x13)
            has_0x11 = self.test_urm04_sensor(port_info["device"], 19200, 0x11)

            if has_0x13 and has_0x11:
                hub_derecho_port = port_info["device"]

                # Asignar der_front (0x13)
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

                # Asignar izq_front (0x11) - SOLO en el mismo puerto que tiene 0x13
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

                self.get_logger().info(
                    f"Hub derecho detectado en {port_info['device']} con ambos sensores"
                )
                break
            elif has_0x13:
                # Solo 0x13, asumir que es der_front standalone
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
                self.get_logger().info(
                    f"Solo der_front detectado en {port_info['device']}"
                )

        # 3. Buscar izq_back (0x11) en un puerto DIFERENTE al hub derecho
        self.get_logger().info(
            "Buscando URM04 izq_back (0x11) en puerto independiente..."
        )
        for port_info in available_ports:
            if any(
                config["port"] == port_info["device"]
                for config in self.detected_ports.values()
            ):
                continue
            if port_info["device"] == hub_derecho_port:  # Saltar el hub derecho
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
                self.get_logger().info(
                    f"izq_back detectado en puerto independiente {port_info['device']}"
                )
                break

        # 4. Buscar LiDAR
        self.get_logger().info("Buscando LiDAR...")
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
            self.initialize_detected_sensors()
        else:
            self.get_logger().error("No se detectaron sensores")

    def initialize_lidar_robust(self, port, baudrate):
        """Inicializa LiDAR con configuración robusta para hubs"""
        try:
            self.get_logger().info(
                f"Inicializando LiDAR en {port} con configuración robusta..."
            )

            for attempt in range(3):
                try:
                    if tfmP.begin(port, baudrate):
                        success_count = 0
                        for test in range(10):
                            if tfmP.getData():
                                success_count += 1
                            time.sleep(0.05)

                        if success_count >= 5:
                            self.get_logger().info(
                                f"LiDAR inicializado exitosamente en {port}"
                            )
                            return True
                        else:
                            self.get_logger().warn(
                                f"LiDAR inestable en {port}, solo {success_count}/10 lecturas exitosas"
                            )

                    time.sleep(1.0 * (attempt + 1))

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

        if "lidar" in self.detected_ports:
            self.thread_locks["lidar"] = threading.Lock()
            self.sensor_threads["lidar"] = threading.Thread(
                target=self.lidar_thread, daemon=True
            )
            self.sensor_threads["lidar"].start()

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

    def report_sensor_status(self):
        """Reporta el estado actual de todos los sensores"""
        self.get_logger().info("=== Reporte de estado de sensores ===")
        for config_key, config in self.sensor_configs.items():
            sensor_name = config["name"]
            status = self.sensor_status.get(sensor_name, False)
            last_status = self.last_reading_status.get(sensor_name, "Not initialized")
            error_count = self.error_counts.get(sensor_name, 0)
            port = self.detected_ports.get(sensor_name, {}).get("port", "Not assigned")

            status_msg = f"Sensor {sensor_name} (Puerto: {port}): "
            status_msg += "Activo" if status else "Inactivo"
            status_msg += f" | Errores: {error_count}/{self.max_errors}"
            status_msg += f" | Último estado: {last_status}"
            self.get_logger().info(status_msg)
        self.get_logger().info("=================================")

    def lidar_thread(self):
        """Thread independiente para LiDAR"""
        config = self.detected_ports["lidar"]

        if not self.initialize_lidar_robust(config["port"], config["baudrate"]):
            self.sensor_status["lidar"] = False
            self.error_counts["lidar"] = self.max_errors
            self.last_reading_status["lidar"] = "Initialization failed"
            return

        self.sensor_status["lidar"] = True
        self.error_counts["lidar"] = 0
        self.last_reading_status["lidar"] = "Initialized"

        while self.running:
            try:
                result = self.get_distance_lidar()

                if isinstance(result, dict):
                    distance = result.get("distance")
                    status = result.get("status")
                    self.last_reading_status["lidar"] = status

                    if status == "Valid reading":
                        self.handle_sensor_success("lidar")
                        frame_id = "lidar_frame"
                        self.publish_range("lidar", distance, frame_id, status)
                    elif status == "No object detected":
                        self.handle_sensor_success("lidar")
                        frame_id = "lidar_frame"
                        self.publish_range("lidar", float("inf"), frame_id, status)
                    else:
                        self.handle_sensor_error("lidar")

                time.sleep(0.25)

            except Exception as e:
                self.get_logger().debug(f"Error en thread LiDAR: {str(e)}")
                self.last_reading_status["lidar"] = f"Error: {str(e)}"
                self.handle_sensor_error("lidar")
                time.sleep(0.5)

    def serial_sensors_thread(self):
        """Thread independiente para sensores serie"""
        self.initialize_serial_sensors()

        while self.running:
            try:
                # Iterar sobre todos los sensores seriales definidos en sensor_configs
                for config_key, config in self.sensor_configs.items():
                    sensor_name = config["name"]
                    if (
                        config["name"] == "lidar"
                    ):  # Saltar LiDAR, manejado en otro thread
                        continue

                    # Verificar si el sensor está detectado
                    if sensor_name not in self.detected_ports:
                        # Intentar detectar el sensor si no está en detected_ports
                        self.try_detect_sensor(sensor_name, config)

                    # Intentar leer el sensor si está activo o intentar inicializarlo
                    if self.sensor_status.get(sensor_name, False):
                        result = None
                        ser = self.serial_ports.get(
                            self.detected_ports[sensor_name]["port"]
                        )
                        if ser and ser.is_open:
                            if config["name"] == "der_back":  # URM37
                                result = self.get_distance_urm37(ser)
                            else:  # URM04
                                result = self.get_distance_urm04(ser, config["address"])
                        else:
                            # Puerto no disponible, intentar reinicializar
                            self.initialize_single_serial_sensor(sensor_name, config)
                            ser = self.serial_ports.get(
                                self.detected_ports.get(sensor_name, {}).get("port")
                            )
                            if ser and ser.is_open:
                                if config["name"] == "der_back":
                                    result = self.get_distance_urm37(ser)
                                else:
                                    result = self.get_distance_urm04(
                                        ser, config["address"]
                                    )
                            else:
                                result = {
                                    "distance": None,
                                    "status": "Port not available",
                                }

                        if isinstance(result, dict):
                            distance = result.get("distance")
                            status = result.get("status")
                            self.last_reading_status[sensor_name] = status

                            if status == "Valid reading":
                                self.handle_sensor_success(sensor_name)
                                frame_id = f"{sensor_name}_frame"
                                self.publish_range(
                                    sensor_name, distance, frame_id, status
                                )
                            elif status == "No object detected":
                                self.handle_sensor_success(sensor_name)
                                frame_id = f"{sensor_name}_frame"
                                self.publish_range(
                                    sensor_name, float(4.0), frame_id, status
                                )
                            else:
                                self.handle_sensor_error(sensor_name)
                                frame_id = f"{sensor_name}_frame"
                                self.publish_range(
                                    sensor_name, float(4.0), frame_id, status
                                )  # Publicar incluso en error
                    else:
                        self.last_reading_status[sensor_name] = "Sensor not initialized"
                        frame_id = f"{sensor_name}_frame"
                        self.publish_range(
                            sensor_name, float(4.0), frame_id, "Sensor not initialized"
                        )

                # time.sleep(0.5)

            except Exception as e:
                self.get_logger().debug(f"Error en thread sensores serie: {str(e)}")
                for sensor_name in [
                    config["name"]
                    for config in self.sensor_configs.values()
                    if config["name"] != "lidar"
                ]:
                    self.last_reading_status[sensor_name] = f"Error: {str(e)}"
                time.sleep(1.0)

    def try_detect_sensor(self, sensor_name, config):
        """Intenta detectar un sensor que no está en detected_ports"""
        if sensor_name in self.detected_ports:
            return

        available_ports = self.get_available_ports()
        for port_info in available_ports:
            if any(
                config["port"] == port_info["device"]
                for config in self.detected_ports.values()
            ):
                continue

            if config["name"] == "der_back" and self.test_urm37_sensor_robust(
                port_info, config["baudrate"]
            ):
                self.detected_ports[sensor_name] = {
                    "port": port_info["device"],
                    "type": "urm37",
                    "baudrate": config["baudrate"],
                    "port_info": port_info,
                }
                self.get_logger().info(
                    f"Sensor {sensor_name} (URM37) detectado en {port_info['device']}"
                )
                self.initialize_single_serial_sensor(sensor_name, config)
                break
            elif config["name"] in [
                "der_front",
                "izq_front",
                "izq_back",
            ] and self.test_urm04_sensor(
                port_info["device"], config["baudrate"], config["address"]
            ):
                self.detected_ports[sensor_name] = {
                    "port": port_info["device"],
                    "type": "urm04",
                    "address": config["address"],
                    "baudrate": config["baudrate"],
                    "port_info": port_info,
                }
                self.get_logger().info(
                    f"Sensor {sensor_name} (URM04 0x{config['address']:02X}) detectado en {port_info['device']}"
                )
                self.initialize_single_serial_sensor(sensor_name, config)
                break

    def initialize_single_serial_sensor(self, sensor_name, config):
        """Inicializa un sensor serial específico"""
        try:
            port = self.detected_ports[sensor_name]["port"]
            if port not in self.serial_ports:
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
                time.sleep(0.2)

            self.sensor_status[sensor_name] = True
            self.error_counts[sensor_name] = 0
            self.last_reading_status[sensor_name] = "Initialized"
            self.get_logger().info(f"Sensor {sensor_name} inicializado en {port}")

        except serial.SerialException as e:
            self.get_logger().error(f"Error inicializando {sensor_name}: {str(e)}")
            self.sensor_status[sensor_name] = False
            self.error_counts[sensor_name] = self.max_errors
            self.last_reading_status[sensor_name] = f"Initialization failed: {str(e)}"

    def initialize_detected_sensors(self):
        """Inicializa los sensores detectados"""
        for sensor_name, config in self.detected_ports.items():
            if config["type"] == "lidar":
                self.sensor_status["lidar"] = False
                self.error_counts["lidar"] = 0
                self.last_reading_status["lidar"] = "Not initialized"
            else:
                self.sensor_status[sensor_name] = False
                self.error_counts[sensor_name] = 0
                self.last_reading_status[sensor_name] = "Not initialized"

    def initialize_serial_sensors(self):
        """Inicializa sensores serie con configuración robusta"""
        for sensor_name, config in self.detected_ports.items():
            if config["type"] in ["urm04", "urm37"]:
                try:
                    port = config["port"]
                    if port not in self.serial_ports:
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
                        time.sleep(0.2)

                    self.sensor_status[sensor_name] = True
                    self.error_counts[sensor_name] = 0
                    self.last_reading_status[sensor_name] = "Initialized"
                    self.get_logger().info(
                        f"Sensor {sensor_name} inicializado en {port}"
                    )

                except serial.SerialException as e:
                    self.get_logger().error(
                        f"Error inicializando {sensor_name}: {str(e)}"
                    )
                    self.sensor_status[sensor_name] = False
                    self.error_counts[sensor_name] = self.max_errors
                    self.last_reading_status[sensor_name] = (
                        f"Initialization failed: {str(e)}"
                    )

    def test_urm04_sensor(self, port, baudrate, address):
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.8,
                write_timeout=0.5,
                inter_byte_timeout=0.05,
            )
            time.sleep(0.2)  # Estabilizar puerto

            success_count = 0
            for attempt in range(5):
                try:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()

                    # Enviar comando de medición (0x01)
                    cmd1 = self.build_command_urm04(address, 0x01)
                    ser.write(bytearray(cmd1))
                    time.sleep(0.1)  # Aumentar tiempo para respuesta

                    # Enviar comando de lectura (0x02)
                    cmd2 = self.build_command_urm04(address, 0x02)
                    ser.write(bytearray(cmd2))
                    time.sleep(0.15)  # Tiempo suficiente para respuesta completa

                    if ser.in_waiting >= 8:
                        response = ser.read(8)
                        # Verificar cabecera y dirección
                        if (
                            len(response) == 8
                            and response[0] == 0x55
                            and response[1] == 0xAA
                            and response[2] == address
                        ):
                            success_count += 1
                            if success_count >= 2:  # Requiere 2 éxitos
                                ser.close()
                                self.get_logger().info(
                                    f"URM04 0x{address:02X} detectado en {port}"
                                )
                                return True
                        else:
                            self.get_logger().debug(
                                f"Respuesta inválida para URM04 0x{address:02X} en {port}: {response.hex()}"
                            )
                    else:
                        self.get_logger().debug(
                            f"Respuesta incompleta para URM04 0x{address:02X} en {port}: {ser.in_waiting} bytes recibidos"
                        )
                    time.sleep(0.1)
                except Exception as e:
                    self.get_logger().debug(
                        f"Intento {attempt + 1} falló para URM04 0x{address:02X} en {port}: {str(e)}"
                    )
                    time.sleep(0.1)

            ser.close()
            self.get_logger().debug(
                f"No se detectó URM04 0x{address:02X} en {port} tras 5 intentos"
            )
            return False
        except Exception as e:
            self.get_logger().debug(f"Error probando URM04 en {port}: {str(e)}")
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
                if time.time() - start_time > 0.2:
                    return None
            response = ser.read(expected_length)
            return response if len(response) == expected_length else None
        except Exception:
            return None

    def get_distance_urm04(self, ser, sensor_address):
        """Lee distancia URM04 con manejo robusto de errores"""
        try:
            ser.reset_input_buffer()  # Limpiar buffer antes de enviar comando

            if not self.send_command(
                ser, self.build_command_urm04(sensor_address, 0x01)
            ):
                return {"distance": None, "status": "Failed to send command"}
            time.sleep(0.05)
            if not self.send_command(
                ser, self.build_command_urm04(sensor_address, 0x02)
            ):
                return {"distance": None, "status": "Failed to send second command"}

            response = self.read_response(ser, 8)
            if response and len(response) == 8:
                if response[0] == 0x55 and response[1] == 0xAA:
                    distance = (response[5] << 8) | response[6]
                    if distance == 0xFFFF or distance == 0:  # Sin objeto detectado
                        return {"distance": None, "status": "No object detected"}
                    elif distance > 0 and distance < 65535:  # Lectura válida
                        return {"distance": distance / 100.0, "status": "Valid reading"}
                    else:
                        return {"distance": None, "status": "Invalid distance value"}
                else:
                    return {"distance": None, "status": "Invalid response header"}
            else:
                return {"distance": None, "status": "No response received"}
        except Exception as e:
            self.get_logger().debug(f"Error leyendo URM04: {str(e)}")
            return {"distance": None, "status": f"Exception: {str(e)}"}

    def get_distance_urm37(self, ser):
        """Lee distancia URM37 con manejo robusto de errores"""
        try:
            ser.reset_input_buffer()  # Limpiar buffer antes de enviar comando

            if not self.send_command(ser, self.build_command_urm37()):
                return {"distance": None, "status": "Failed to send command"}
            time.sleep(0.15)

            if ser.in_waiting >= 4:
                response = ser.read(4)
                if len(response) == 4:
                    header, highbyte, lowbyte, checksum = response
                    if header == 0x22 and checksum == (
                        (header + highbyte + lowbyte) & 0xFF
                    ):
                        distance = (highbyte << 8) | lowbyte
                        if distance == 0xFFFF or distance == 0:  # Sin objeto detectado
                            return {"distance": None, "status": "No object detected"}
                        elif distance > 0 and distance < 65535:  # Lectura válida
                            return {
                                "distance": distance / 100.0,
                                "status": "Valid reading",
                            }
                        else:
                            return {
                                "distance": None,
                                "status": "Invalid distance value",
                            }
                    else:
                        return {
                            "distance": None,
                            "status": "Invalid checksum or header",
                        }
                else:
                    return {"distance": None, "status": "Incomplete response"}
            else:
                return {"distance": None, "status": "No response received"}
        except Exception as e:
            self.get_logger().debug(f"Error leyendo URM37: {str(e)}")
            return {"distance": None, "status": f"Exception: {str(e)}"}

    def get_distance_lidar(self):
        """Lee distancia LiDAR con manejo robusto de errores"""
        try:
            if tfmP.getData():
                distance = tfmP.dist
                if distance == 0 or distance == 65535:  # Common no-detection values
                    return {"distance": None, "status": "No object detected"}
                return {"distance": distance / 100.0, "status": "Valid reading"}
            return {"distance": None, "status": "No data received"}
        except Exception as e:
            self.get_logger().debug(f"Error leyendo LiDAR: {str(e)}")
            return {"distance": None, "status": f"Exception: {str(e)}"}

    def handle_sensor_error(self, sensor_name):
        """Maneja errores de sensores de forma consistente. No incrementa errores para 'No object detected'."""
        current_status = self.last_reading_status.get(sensor_name, "")
        if "No object detected" not in current_status:
            self.error_counts[sensor_name] = self.error_counts.get(sensor_name, 0) + 1
            self.get_logger().debug(
                f"Error en {sensor_name}: {current_status}. Errores totales: {self.error_counts[sensor_name]}/{self.max_errors}"
            )

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

    def publish_range(self, sensor_name, distance, frame_id, status):
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

        # Usar 4.0 metros cuando no se detecta un objeto
        if status == "No object detected":
            range_msg.range = 4.0
        else:
            range_msg.range = distance if distance is not None else float("inf")

        self.sensor_publishers[sensor_name].publish(range_msg)

        log_msg = f"{sensor_name}: "
        if status == "Valid reading":
            log_msg += f"{distance:.2f} m"
        elif status == "No object detected":
            log_msg += "4.00 m (no object detected)"
        else:
            log_msg += f"Error - {status}"
        self.get_logger().info(log_msg)

    def retry_detection(self):
        """Reintenta la detección de sensores"""
        self.get_logger().info("Reintentando detección de sensores...")
        self.auto_detect_sensors()

        if self.detected_ports:
            self.retry_timer.cancel()
            self.reconnect_timer = self.create_timer(30.0, self.attempt_reconnections)
            self.status_report_timer = self.create_timer(5.0, self.report_sensor_status)
            self.start_sensor_threads()

    def cleanup(self):
        """Limpieza segura"""
        self.running = False

        for thread in self.sensor_threads.values():
            if thread.is_alive():
                thread.join(timeout=1.0)

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
