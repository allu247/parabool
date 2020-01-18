#! /usr/bin/env python

from pymodbus.client.sync import ModbusSerialClient as ModbusClient


class RegisterElement:
    COIL_OFFSET = -1

    def __init__(self, name, description, example, hex_address, function_code, write_access, is_coil, port, slave):
        self.name = name
        self.description = description
        self.hex_address = hex_address
        self.write_access = write_access
        self.is_coil = is_coil
        self.port = port
        self.slave = slave
        self.function_code = function_code
        self.example = example

    def get_client(self):
        client = ModbusClient(method='rtu', port=self.port, timeout=1, stopbits=1, bytesize=8, parity='N',
                              baudrate=9600)
        client.connect()
        return client

    def get_integer_address(self):
        return int(str(self.hex_address), 16) + self.COIL_OFFSET

    def get_register_name(self):
        return self.name

    def get_value(self):
        final_address = self.get_integer_address()
        client = self.get_client()

        if self.is_coil is True:
            result = client.read_coils(final_address, 1, unit=self.slave)
            client.close()
            return result.bits
        else:
            result = client.read_holding_registers(final_address, 1, unit=self.slave)
            client.close()
            return result.registers

    def get_value_by_existing_client(self, client):
        final_address = self.get_integer_address()

        if self.is_coil is True:
            result = client.read_coils(final_address, 1, unit=self.slave)
            return result.bits
        else:
            result = client.read_holding_registers(final_address, 1, unit=self.slave)
            return result.registers

    def write_value(self, value):
        final_address = self.get_integer_address()
        client = self.get_client()

        if self.is_coil is True:
            result = client.write_coil(final_address, value, unit=self.slave)
            client.close()
            return result.bits
        else:
            result = client.write_register(final_address, value, unit=self.slave)
            client.close()
            return result.registers
