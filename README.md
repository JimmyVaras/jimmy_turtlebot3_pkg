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
