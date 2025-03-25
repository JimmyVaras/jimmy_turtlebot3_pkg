import subprocess

def start_simulation(gui=False):
    try:
        subprocess.Popen(["gnome-terminal", "--", "roslaunch", "jimmy_simulations_pkg", "small_house.launch", f"gui:={gui}"])
    except Exception as e:
        print(f"Error al iniciar la simulación: {e}")

def start_navigator():
    try:
        subprocess.Popen(["gnome-terminal", "--", "roslaunch", "jimmy_tools_pkg", "navigation.launch"])
    except Exception as e:
        print(f"Error al arrancar navigation.launch: {e}")

def start_detector():
    try:
        subprocess.Popen(["gnome-terminal", "--", "roslaunch", "jimmy_tools_pkg", "detection.launch"])
    except Exception as e:
        print(f"Error al arrancar detection.launch: {e}")

def start_lmg():
    try:
        subprocess.Popen(["gnome-terminal", "--", "roslaunch", "jimmy_tools_pkg", "location_memory_goto.launch"])
    except Exception as e:
        print(f"Error al arrancar location_memory_goto.launch: {e}")