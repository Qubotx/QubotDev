import pymodbus.client.sync as _sync

# patch write_register
_orig_wr = _sync.ModbusSerialClient.write_register


def _write_register(self, address, value, device_id=None, **kw):
    return _orig_wr(self, address, value, unit=device_id, **kw)


_sync.ModbusSerialClient.write_register = _write_register

# patch write_registers
_orig_wrs = _sync.ModbusSerialClient.write_registers


def _write_registers(self, address, values, device_id=None, **kw):
    return _orig_wrs(self, address, values, unit=device_id, **kw)


_sync.ModbusSerialClient.write_registers = _write_registers


from pymodbus.client.sync import ModbusSerialClient as ModbusClient

# from pymodbus.client import ModbusSerialClient as ModbusClient
import logging

# Habilitar logs de depuración (opcional)
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# Crear cliente Modbus RTU
client = ModbusClient(
    method="rtu",
    port="/dev/ttyUSB0",
    baudrate=115200,
    timeout=1,
    stopbits=1,
    bytesize=8,
    parity="N",
)

# Dirección del dispositivo Modbus (unit ID)
DEVICE_ID = 1

# Dirección de registro de control
CONTROL_REG = 0x2000  # ejemplo, verificar con datasheet
STOP_COMMAND = 0x02  # valor típico para detener motores

# Conectar
if not client.connect():
    print("Error: no se pudo conectar con el dispositivo ZLAC8015D")
    exit(1)

# Enviar comando para deshabilitar motores
result = client.write_register(CONTROL_REG, STOP_COMMAND, DEVICE_ID)

client.write_registers(0x2080, [0, 0], 1)

# Verificar respuesta
if result.isError():
    print("Error al escribir registro:", result)
else:
    print("Comando enviado correctamente")

# Cerrar conexión
client.close()
