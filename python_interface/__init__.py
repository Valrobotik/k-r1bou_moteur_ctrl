import serial
import time
import serial.tools.list_ports

import numpy as np
from math import pi

try :
    np.load("params.npy")
except FileNotFoundError:
    params = {
        "motor1_kp" : .0,
        "motor1_ki" : .0,
        "motor1_kd" : .0,

        "motor2_kp" : .0,
        "motor2_ki" : .0,
        "motor2_kd" : .0,

        "motor1_wheel_perimeter" : pi*0.05,
        "motor2_wheel_perimeter" : pi*0.05,

        "wheel_distance" : 0.1976,
    }
    np.save("params.npy", params)

baudrate = 115200

def get_ports():
    list_com_ports = {
        "motor" : None,
        "ihm" : None,
        "lidar1" : None,
        "lidar2" : None,
        "actioneur" : None,
    }

    list_available_port = serial.tools.list_ports.comports()
    for port in list_available_port:
        test_port = serial.Serial(port.device, baudrate, timeout=1)
        test_port.write(b"QX\n")
        received = test_port.readline().decode().strip()
        if "Motor" in received:
            list_com_ports["motor"] = port.device
        elif "IHM" in received:
            list_com_ports["ihm"] = port.device
        elif "Actioneur" in received:
            list_com_ports["actioneur"] = port.device

    return list_com_ports

class RobotInterface:
    def __init__(self, port):
        self.ser = serial.Serial(port, 9600, timeout=1)
        self.stop = False
        self.continuous_speed_return = False
        self.open_loop = False

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.time_uctrl = 0.0

        self.cmd_list = {
            "Linear Speed" : "LX",
            "Angular Speed" : "AX",

            "Motor1 Kp" : "PL",
            "Motor1 Ki" : "IL",
            "Motor1 Kd" : "DL",

            "Motor2 Kp" : "PR",
            "Motor2 Ki" : "IR",
            "Motor2 Kd" : "DR",

            "Motor1 Wheel Perimeter" : "CL",
            "Motor2 Wheel Perimeter" : "CR",

            "Wheel Distance" : "EX",
            "Stop Robot" : "SX",
            "Toggle Continuous Speed Return" : "VX",
            "Print Odometry" : "WX",
            "Enable Open Loop" : "BO",
            "Disable Open Loop" : "BF",
            "Request Angle" : "GX",
            "Print Time" : "TX",
        }

        self.list_cmd_to_execute = {
            "cmd_name" : [],
            "cmd_values" : [],
        }


    def send_command(self, cmd_type : list, values : list =None):
        command = f""
        for i in range(0, len(cmd_type)) :
            command += f"{self.cmd_list[cmd_type[i]]}{values[i]}"
        command += "\n"
        self.ser.write(command.encode())

    def add_cmd(self, cmd_type : list, value : list =None):
        self.list_cmd_to_execute["cmd_name"].append(cmd_type)
        self.list_cmd_to_execute["cmd_values"].append(value)

    def execute_cmd(self):
        self.send_command(self.list_cmd_to_execute["cmd_name"], self.list_cmd_to_execute["cmd_values"])

    def set_linear_speed(self, value: float, execute: bool = False):
        self.add_cmd("Linear Speed", value)
        if execute: self.execute_cmd()

    def set_angular_speed(self, value: float, execute: bool = False):
        self.add_cmd("Angular Speed", value)
        if execute: self.execute_cmd()

    def set_motor1_kp(self, value: float, execute: bool = False, save : bool = True):
        self.add_cmd("Motor1 Kp", value)
        
        if save :
            params["motor1_kp"] = value
            np.save("params.npy", params)
        if execute: self.execute_cmd()

    def set_motor1_ki(self, value: float, execute: bool = False, save : bool = True):
        self.add_cmd("Motor1 Ki", value)
        if save :
            params["motor1_ki"] = value
            np.save("params.npy", params)
        if execute: self.execute_cmd()

    def set_motor1_kd(self, value: float, execute: bool = False, save : bool = True):
        self.add_cmd("Motor1 Kd", value)
        if save :
            params["motor1_kd"] = value
            np.save("params.npy", params)
        if execute: self.execute_cmd()

    def set_motor2_kp(self, value: float, execute: bool = False, save : bool = True):
        self.add_cmd("Motor2 Kp", value)
        if save :
            params["motor2_kp"] = value
            np.save("params.npy", params)
        if execute: self.execute_cmd()

    def set_motor2_ki(self, value: float, execute: bool = False, save : bool = True):
        self.add_cmd("Motor2 Ki", value)
        if save :
            params["motor2_ki"] = value
            np.save("params.npy", params)
        if execute: self.execute_cmd()

    def set_motor2_kd(self, value: float, execute: bool = False, save : bool = True):
        self.add_cmd("Motor2 Kd", value)
        if save :
            params["motor2_kd"] = value
            np.save("params.npy", params)
        if execute: self.execute_cmd()

    def update_motor1_wheel_perimeter(self, value: float, execute: bool = False, save : bool = True):
        self.add_cmd("Motor1 Wheel Perimeter", value)
        if save :
            params["motor1_wheel_perimeter"] = value
            np.save("params.npy", params)
        if execute: self.execute_cmd()

    def update_motor2_wheel_perimeter(self, value: float, execute: bool = False, save : bool = True):
        self.add_cmd("Motor2 Wheel Perimeter", value)
        if save :
            params["motor2_wheel_perimeter"] = value
            np.save("params.npy", params)
        if execute: self.execute_cmd()

    def set_wheel_distance(self, value: float, execute: bool = False, save : bool = True):
        self.add_cmd("Wheel Distance", value)
        if save :
            params["wheel_distance"] = value
            np.save("params.npy", params)
        if execute: self.execute_cmd()

    def stop_robot(self, execute: bool = False):
        self.add_cmd("Stop Robot")
        if execute: self.execute_cmd()

    def toggle_continuous_speed_return(self, execute: bool = False):
        self.add_cmd("Toggle Continuous Speed Return")
        if execute: self.execute_cmd()

    def print_odometry(self, execute: bool = False):
        self.add_cmd("Print Odometry")
        if execute: self.execute_cmd()

    def enable_open_loop(self, execute: bool = False):
        self.add_cmd("Enable Open Loop")
        if execute: self.execute_cmd()

    def disable_open_loop(self, execute: bool = False):
        self.add_cmd("Disable Open Loop")
        if execute: self.execute_cmd()

    def request_angle(self, execute: bool = False):
        self.add_cmd("Request Angle")
        if execute: self.execute_cmd()

    def print_time(self, execute: bool = False):
        self.add_cmd("Print Time")
        if execute: self.execute_cmd()
    
    def set_speed(self, linear_speed: float, angular_speed: float, execute: bool = False):
        self.set_linear_speed(linear_speed)
        self.set_angular_speed(angular_speed)
        if execute: self.execute_cmd()

    def receive_speed(self):
        received = self.ser.readline().decode().strip()
        if received[0] == "L":
            L = 0
            A = received.index("A")
            T = received.index("T")
            self.linear_speed = float(received[L+1:A])
            self.angular_speed = float(received[A+1:T])
            self.time_uctrl = float(received[T+1:])
            return self.linear_speed, self.angular_speed, self.time_uctrl
        else:
            print(f"Error in received data : {received}")

    def loop(self):
        while not self.stop:
            if self.ser.in_waiting:
                self.receive_speed()

robot : RobotInterface = None

def cmd_vell_callback(data):
    #data type : Twist
    robot.set_speed(data.linear.x, data.angular.z, execute=True)

def cmd_stop_callback(data):
    robot.stop_robot(execute=True)

def initialise_robot_cst():
    """Initialise the robot with the cst values"""
    robot.set_motor1_kp(params["motor1_kp"], save=False)
    robot.set_motor1_ki(params["motor1_ki"], save=False)
    robot.set_motor1_kd(params["motor1_kd"], save=False)

    robot.set_motor2_kp(params["motor2_kp"], save=False)
    robot.set_motor2_ki(params["motor2_ki"], save=False)
    robot.set_motor2_kd(params["motor2_kd"], save=False)

    robot.update_motor1_wheel_perimeter(params["motor1_wheel_perimeter"], save=False)
    robot.update_motor2_wheel_perimeter(params["motor2_wheel_perimeter"], save=False)

    robot.set_wheel_distance(params["wheel_distance"], save=False)

    robot.execute_cmd()



if __name__ == "__main__":
    list_com_ports = get_ports()
    print(list_com_ports)
    robot = RobotInterface(list_com_ports["motor"])
