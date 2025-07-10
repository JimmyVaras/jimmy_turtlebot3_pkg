# --------------------
# This is the Python file that contains the commands to be executed when clicking on laucnh buttons in the GUI
# Author: Jaime Varas Cáceres
# --------------------

import subprocess


def start_simulation(gui=False):
    try:
        subprocess.Popen(
            ["gnome-terminal", "--", "roslaunch", "jimmy_simulations_pkg", "small_house.launch", f"gui:={gui}"])
    except Exception as e:
        print(f"Error al iniciar la simulación: {e}")


def start_navigator():
    try:
        subprocess.Popen(["gnome-terminal", "--", "roslaunch", "jimmy_tools_pkg", "nav_db.launch"])
    except Exception as e:
        print(f"Error al arrancar nav_db.launch: {e}")


def start_detector():
    try:
        subprocess.Popen(["gnome-terminal", "--", "roslaunch", "jimmy_tools_pkg", "detection.launch"])
    except Exception as e:
        print(f"Error al arrancar detection.launch: {e}")


def start_lmg():
    try:
        subprocess.Popen(["gnome-terminal", "--", "roslaunch", "jimmy_tools_pkg", "patrol.launch"])
    except Exception as e:
        print(f"Error al arrancar patrol.launch: {e}")


def start_rosb():
    try:
        subprocess.Popen(["gnome-terminal", "--", "roslaunch", "rosbridge_server", "rosbridge_websocket.launch"])
    except Exception as e:
        print(f"Error al arrancar ROSBridge: {e}")


def start_tunnel():
    try:
        subprocess.Popen(["gnome-terminal", "--", "devtunnel", "host", "ros"])
    except Exception as e:
        print(f"Error al arrancar el tunnel <ROS>: {e}")
