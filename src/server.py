from flask import Flask, request
from flask_cors import CORS, cross_origin
import json




from src.antenna.horizontal_motor.horizontal_motor_node import RotationService
from src.antenna.horizontal_motor.service.RegisterService import RegisterService




app = Flask(__name__)
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'

# USED ON HORIZONTAL MOTOR DUE TO TEST SET UP

port = '/dev/tty.usbserial-A906O081'
slave = 1

horizontal_register_service = RegisterService(port, slave)
horizontal_rotate_service = RotationService(port, slave)



@app.route("/coil/<name>", methods=['GET'])
def get_coil_by_name(name):
    register_element = horizontal_register_service.get_coil_by_name_as_dict(name)

    return json.dumps(register_element)


@app.route("/coil", methods=['GET'])
def get_all_coils():
    register_elements = horizontal_register_service.get_all_coils()

    return json.dumps(register_elements)


@app.route("/register/<name>", methods=['GET'])
def get_register_by_name_(name):
    register_element = horizontal_register_service.get_register_by_name_as_dict(name)

    return json.dumps(register_element)


@app.route("/register", methods=['GET'])
def get_all_registers():
    register_elements = horizontal_register_service.get_all_registers()

    return json.dumps(register_elements)


@app.route("/coil/<address>", methods=['POST'])
def write_to_coil(address):
    value = str(request.json.get('value', ''))
    if value.__len__() == 0:
        return "Value can not be null"

    response = horizontal_register_service.write_to_coil(address, value)
    return json.dumps(response)


@app.route("/register/<address>", methods = ['POST'])
def write_to_register(address):
    value = str(request.json.get('value', ''))

    if value.__len__() == 0:
        return "Value can not be null"

    response = horizontal_register_service.write_to_register(address, value)
    return json.dumps(response)


@app.route("/setting", methods=['POST'])
def write_to_configuration():
    new_port = str(request.json.get('port', ''))
    new_slave = str(request.json.get('slave', ''))

    response = horizontal_register_service.save_new_configuration(new_port, new_slave)
    return json.dumps(response)


@app.route("/setting", methods = ['GET'])
def get_configuration():
    response = horizontal_register_service.get_configuration()
    return json.dumps(response)


@app.route("/coil/<address>/value", methods=['GET'])
def get_coil_value_from_source(address):
    response = horizontal_register_service.get_coil_value_from_inverter(address)
    return json.dumps(response[0])


@app.route("/register/<address>/value", methods=['GET'])
def get_register_value_from_source(address):
    response = horizontal_register_service.get_register_value_from_inverter(address)
    return json.dumps(response)


@app.route("/rotate", methods=['POST'])
def manual_rotate():
    commands = request.json

    response = horizontal_rotate_service.append_to_command_queue(commands, True)
    return json.dumps(response)

@app.rout("/buttonrotate", methods=['POST'])
def button_rotate():
    if(response[0]):
        if(response[1] == 0):
            RotationService.rotate_horizontal_left()
        if(response[1] == 1):
            RotationService.rotate_horizontal_right()
        if(response[1] == 2):
            RotationService.rotate_vertical_up()
        if(response[1] == 3):
            RotationService.rotate_vertical_down()
    else:
        if(response[1] == 0 or response[1] == 1):
            RotationService.rotate_horizontal_stop()
        if(response[1] == 2 or response[1] == 3):
            RotationService.rotate_vertical_stop()
    return json.dumps(response)
