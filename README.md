# Reconocimiento de objetos simulados en el entorno Gazebo/ROS

**Realizado por**: Jaime Varas Cáceres

**Dirigido por**: Fernando Díaz del Río

**Versión ROS usada**: Noetic Ninjemys

**Modelo TurtleBot3** usado: TB3 Waffle Pi con cámara de profundidad añadida (en [/jimmy_simulations_pkg/docs/](https://github.com/JimmyVaras/jimmy_turtlebot3_pkg/blob/master/jimmy_simulations_pkg/docs/turtlebot3_waffle_pi.gazebo.xacro))

## Descripción:

Este paquete contiene el código desarrollado (del sistema ROS) para mi Trabajo de Fin de Grado, de Ingeniería de Software en la Universidad de Sevilla, en 2025. En este proyecto, se desarrolló un sistema para ejecutarse en un robot móvil simulado basado en TurtleBot3. 
El objetivo es recorrer una vivienda, detectar objetos y ejecutar órdenes que provienen de la aplicación web (en este otro repositorio: https://github.com/JimmyVaras/ros-web-app). 
Se utilizan técnicas de visión por computador, navegación autónoma y percepción 3D para permitir la detección contextual y la localización de objetos en el entorno.

## 3 Sub-paquetes:

- **jimmy_gui_pkg**: Interfaz de usuario que sirve para arrancar los sistemas del robot desde su propio entorno local.
- **jimmy_simulations_pkg**: Simulaciones, mapas y configuraciones para ejecutar la simulación en la que se ha probado el robot.
- **jimmy_tools_pkg**: Scripts, nodos, lanzadores de la lógica que se encarga de la detección, localización, navegación y otras tareas controlables desde la app web.

---

# Object Recognition and Navigation System for TurtleBot3 (ROS)

This repository contains the **ROS (Robot Operating System)** packages for a **TurtleBot3** mobile robot. The system enables autonomous navigation, object detection using computer vision, and mapping of object positions in a simulated home environment.

This software is part of a larger project and is designed to be controlled by a [remote control web application](#) (link to your other repository).

## 🤖 Main Features

* **Realistic Simulation**: Simulated home environment in **Gazebo**, using the `aws-robomaker-small-house-world` map.
* **Object Detection**: Uses a **YOLOv8** model to identify objects in real time through the robot’s camera.
* **3D Localization**: Calculates the spatial position of detected objects using depth camera data.
* **Autonomous Navigation**: Implements the ROS navigation stack (**AMCL**, **move_base**) for mapping, localization, and autonomous movement.
* **Spatial Memory**: Stores object detections in an external database to build a “map” of household items.
* **Smart Patrolling**: The robot can autonomously follow a series of predefined waypoints, stopping to perform a 360° scan to enhance detection.
* **Object-Based Navigation**: Can autonomously move to the location of a previously detected object.

## 🛠️ Technologies Used

* **ROS 1 Noetic Ninjemys**
* **Gazebo**: for 3D simulation of the robot and environment.
* **RViz**: for visualizing data such as maps, robot position, and detections.
* **Python**: main language used for ROS nodes.
* **TurtleBot3**: robotic platform (Waffle Pi model).
* **YOLOv8**: for object detection.
* **OpenCV**: for image processing.

## 🚀 Getting Started

### Prerequisites

* Ubuntu 20.04 Focal Fossa
* ROS 1 Noetic installed
* TurtleBot3 simulation packages for ROS Noetic
* Python libraries: `ultralytics`, `opencv-python`, `rospy`, etc.

### Installation

1. Clone this repository inside the `src` folder of your `catkin_workspace`:
    ```bash
    cd ~/catkin_workspace/src
    git clone https://github.com/YOUR_USERNAME/YOUR_REPOSITORY.git
    ```

2. Install Python dependencies:
    ```bash
    pip install -r requirements.txt
    ```

3. Go back to the workspace root and build the project:
    ```bash
    cd ~/catkin_workspace
    catkin_make
    ```

### Usage

The system is launched through a series of `.launch` files. Run them in separate terminals in the following order:

1. **Start the Gazebo simulation**:  
    This command loads the world and the TurtleBot3 model.
    ```bash
    roslaunch turtlebot3_pkg simulation.launch
    ```

2. **Activate the navigation stack**:  
    This command starts the nodes for localization (AMCL), map server, and RViz.
    ```bash
    roslaunch turtlebot3_pkg navigation.launch
    ```

3. **Start the perception nodes**:  
    This command launches the object detection nodes using YOLO and the locator node that calculates object positions.
    ```bash
    roslaunch turtlebot3_pkg detection.launch
    ```

Once all components are running, the system is ready to receive commands from the external web application.
