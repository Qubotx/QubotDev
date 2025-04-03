### QubotDevDev en ROS2 Jazzy

Historial de cambios:

03/04/2025
- Se crea  zlac8015d_bridge.py, nodo para manejar tópicos de ros2 y enviarlos al driver de los motores.

Para usar:
- Requiere instalar pymodbus
pip install pymodbus

- Compilar, source...
- Ajustar en el código puerto serial /dev/ttyUSBXX
Como conocer ña dirección del puerto:
pyserial-miniterm

- Dar permisos al puerto
sudo chmod a+rw /dev/ttyUSBXX

- ejecutar el nodo
ros2 run QubotDev zlac8015d_bridge

- controlar con algo
ros2 run teleop_twist_keyboard teleop_twist_keyboard
