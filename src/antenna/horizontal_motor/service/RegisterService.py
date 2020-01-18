
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.client.sync import ModbusRtuFramer
import pymodbus.exceptions
from RegisterMapping import RegisterMapping


class RegisterService:

    def __init__(self, port, slave):
        self.host = '192.168.1.101'
        self.port = port
        self.slave = slave
        self.database = RegisterMapping(port, slave)
        self.client = ModbusClient(host=self.host, port=port, timeout=1, stopbits=1, bytesize=8, parity='N', baudrate=9600, framer=ModbusRtuFramer)

    def save_new_configuration(self, port, slave):
        if port.__len__() != 0:
            self.port = port
        if slave.__len__() != 0:
            self.slave = int(slave)

        self.database = RegisterMapping(self.port, self.slave)
        self.client = ModbusClient(host=self.host, port=self.port, timeout=1, stopbits=1, bytesize=8, parity='N', baudrate=9600, framer=ModbusRtuFramer)

        return self.get_configuration()

    def get_configuration(self):
        return {"port": self.port, "slave": self.slave}

    def get_all_coils(self):
        return self.database.get_coils_as_dict()

    def get_coil_by_name_as_dict(self, name):
        return self.database.get_coil_as_dict(name)

    def get_all_registers(self):
        return self.database.get_registers_as_dict()

    def get_register_by_name_as_dict(self, name):
        return self.database.get_register_as_dict(name)

    def write_to_coil(self, address, value):
        register = self.database.get_coil_by_name(address)
        decimal_address = register.get_integer_address()
        integer_value = int(value)

        try:
            self.client.connect()
            self.client.write_coil(decimal_address, integer_value, unit=self.slave)
            self.client.close()
        except pymodbus.exceptions.ConnectionException:
            return "No device connected"

        return {"address": address, "decimal_address": decimal_address, "value": value, "status": "OK"}

    def write_to_register(self, address, value):
        register = self.database.get_register_by_name(address)
        decimal_address = register.get_integer_address()
        integer_value = int(value)
        try:
            self.client.connect()
            self.client.write_register(decimal_address, integer_value, unit=self.slave)
            self.client.close()
        except pymodbus.exceptions.ConnectionException:
            return "No device connected"

        return {"address": address, "decimal_address": decimal_address, "value": value, "status": "OK"}

    def get_coil_value_from_inverter(self, address):
        register = self.database.get_register_by_name(address)
        decimal_address = register.get_integer_address()
        try:
            response = self.client.read_coils(decimal_address, 1, unit=self.slave)
        except pymodbus.exceptions.ConnectionException:
            return "No device connected"
        return response.bits

    def get_register_value_from_inverter(self, address):
        register = self.database.get_register_by_name(address)
        decimal_address = register.get_integer_address()

        try:
            self.client.connect()
            response = self.client.read_holding_registers(decimal_address, 1, unit=self.slave)
            self.client.close()
        except pymodbus.exceptions.ConnectionException:
            return "No device connected"

        return response.registers[0]