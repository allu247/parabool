#! /usr/bin/env python3

import csv
import pathlib

from RegisterElement import RegisterElement


class RegisterMapping:

    def __init__(self, port, slave):
        self.port = port
        self.slave = slave
        self.coils = {}
        self.registers = {}

        self.fill_mapping_with_coil_data()
        self.fill_mapping_with_register_data()

    def get_coils(self):
        return self.coils

    def get_registers(self):
        return self.registers

    def get_coil_by_name(self, name):
        return self.coils[name]

    def get_register_by_name(self, name):
        return self.registers.get(name)

    def get_coils_as_dict(self):
        dict_list = []
        for register in self.coils.values():
            dict_list.append(register.__dict__)
        return dict_list

    def get_coil_as_dict(self, name):
        return self.coils.get(name).__dict__

    def get_registers_as_dict(self):
        dict_list = []
        for register in self.registers.values():
            dict_list.append(register.__dict__)
        return dict_list

    def get_register_as_dict(self, name):
        return self.registers.get(name).__dict__

    def fill_mapping_with_coil_data(self):
        file = pathlib.Path(__file__).parent.parent / 'files/coils.csv'
        with open(str(file), 'r') as csv_file:
            reader = csv.reader(csv_file, delimiter=';', quotechar='"')
            for row in reader:
                address = str(row[0]).replace("h", "")
                write_access = str(row[2]) == "R/W"
                self.coils[address] = RegisterElement(address, row[1], row[3], address, "", write_access, True,
                                                        self.port, self.slave)

    def fill_mapping_with_register_data(self):
        file = pathlib.Path(__file__).parent.parent / 'files/registers.csv'
        with open(str(file), 'r') as csv_file:
            reader = csv.reader(csv_file, delimiter=';', quotechar='"')
            for row in reader:
                address = str(row[0]).replace("h", "")
                write_access = str(row[3]) == "R/W"
                self.registers[address] = RegisterElement(address, row[1], row[4], address, row[2], write_access, False,
                                                        self.port, self.slave)