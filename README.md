<b>NB !!! This application can not be runned without Omron Inverter connected to the computer"</b>

<b>Setup for Ubuntu</b>

1. Install ROS Melodic (http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Setup ROS by using same link provided before
3. Install node and npm (latest)
4. Install Flask
5. pip install pathlib
6. pip install  -U pymodbus
7. pip install flask_cors

To get the USB port name run next command on terminal: <br>

$ ls /dev/tty.*

Launch node by making first next files executable:

$ chmod +x <filename>

1. horizontal_motor_node.py
2. horizontal_sensor.py
3. vertical_motor_node.py
4. vertical_sensor.py
5. main_controller.py

Run frontend:

1. npm install
2. npm start

Run backend:

1. FLASK_APP=server.py flask run