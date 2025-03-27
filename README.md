## Introducción

Este repositorio fue creado a partir de https://github.com/cmoralesd/tortubot

fcoera-main y todas las ramas comenzando con fcoera son ramas de desarrollo y pruebas de Fcoera

## QubotDev en ROS2 Jazzy y Gazebo Harmonic

Paquete de pruebas para el control del robot educativo QubotDev en entorno de simulación.<br>
El paquete ha sido creado siguiendo el procedimiento descrito por @ArticulatedRobotics (https://www.youtube.com/@ArticulatedRobotics). Gracias por la valiosa información!<br>

El paso a paso del procedimiento de este tutorial, se encuentra en los siguientes enlaces:<br>
Puesta en marcha: <br>
Mapeo y localización SLAM: <br>
Navegación autónoma: <br>

### 1. Descargar y verificar el proyecto base
El proyecto base para iniciar la integración con los módulos de SLAM y NAV2 se encuentra en: https://github.com/cmoralesd/QubotDev/tree/main

Para ejecutar la simulación:
--------------------------------
Se debe indicar la ruta absoluta al archivo de mundo de GazeboSim (archivo .sdf).
Si desea descargar modelos y mundos desarrollados por la comunidad, visitar este sitio: https://app.gazebosim.org/dashboard

`ros2 launch QubotDev sim.launch.py world:=<ruta_absoluta_al_archivo_sdf_sin_commillas>`<br>

Para enviar órdenes de movimiento por medio de teclado:
---------------------------------
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`<br>

### 2. Crea un mapa con slam-toolbox
Verifique si tiene instalado el paquete de slam_toolbox: <br>
`sudo apt install ros-jazzy-slam-toolbox`<br>
Utilizaremos el modo online-asyncronous (para una breve descripción de la librería slam-toolbox y sus componentes, consultar aquí: https://roscon.ros.org/2019/talks/roscon2019_slamtoolbox.pdf). <br>
Copiar el archivo de configuración desde la siguiente ruta `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml` y alojarla en la carpeta de configuración del proyecto:<br>

En este archivo, ajustar las siguientes opciones:<br>
```
    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: true
    mode: mapping
```
Para ejecutar slam-toolbox, se debe iniciar, indicando la ruta al archivo de configuración e indicando que se trabaja con tiempo de simulación.<br>
`ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<ruta_al_archivo_de configuración> use_sim_time:=true`<br>
Grabar el mapa generado en la carpeta de configuración del proyecto.
Detener la ejecución de slam-toolbox en modo `mapping`.

### 3. Utilizar el mapa para localización
Para localizar el robot en un mapa, utilizaremos las capacidades de <b>slam-toolbox</b>, en conjunto con la librería <b>nav2</b>. Un detalle de conceptos y procedimientos los encuentran en el sitio oficial: https://docs.nav2.org/getting_started/index.html. <br>
Verifique que la librería nav se encuentra instalada en su sistema:<br>
`sudo apt install ros-jazzy-navigation2`<br>
`sudo apt install ros-jazzy-nav2-bringup`<br>

En esta ocasión, utilizaremos el mapa previamente generado, para determinar la localización del robot.<br>
Siga el procedimiento <b>estrictamente en este orden</b>:
1. Reinicie la simulación en GazeboSim.
2. Reinicie la ventana de rviz2. En las opciones de visualización de rviz, agregue map y configure la opción <b>Topic</b> para recibir datos desde <b>/map</b>. En <b>/Global options</b>, defina el marco de referencia fijo escribiendo <b>map</b> en el campo <b>Fixed frame</b>. Rviz mostrará errores porque el servicio de map aun no está creado.
2. Inicie el servicio de mapa de nav2 en un terminal nuevo:<br>
`ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=<ruta_al_archivo_de_mapa_.yaml> -p use_sim_time:=true`
3. Una vez creado el servicio, requiere ser activado. En otro terminal:<br>
`ros2 run nav2_util lifecycle_bringup map_server`
En este paso ya es posible visualizar el mapa en rviz. Si no se visualiza, en Map/Topic, configurar Durability Policy como 'Transient local'<br>
4. Inicializar un servicio de localizador con el algoritmo de Montecarlo:
`ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true`
5. Este servicio de localizador también requiere ser activado:
`ros2 run nav2_util lifecycle_bringup amcl`.
6. Para que los nuevos servicios creados permitan localizar al robot dentro del mapa y se pueda visualizar la posición actual, una estimación inicial de la posición actual del robot debe ser ingresada. Para ello, puede utilizar el boton <b>2D Pose Estimate</b> de Rviz.
7. Mueva el robot sobre el mapa para mejorar la precisión de localización.

### 4. Navegación autónoma con nav2
Con slam-toolbox y nav2 es posible también cargar un mapa pregrabado y actualizarlo en tiempo real, a la vez que navegar sobre el mismo mapa.
Para esto, es necesario modificar el archivo de configuración de slam-toolbox:
mapper_params_online_async.yaml` <br>

En este archivo, ajustar las siguientes opciones:<br>
```
    # ROS Parameters
    ...
    mode: localization
    ...
    map_file_name: <ruta_del_archivo_de_mapa_sin_extension>
    map_start_at_dock: true
```
ahora seguir este procedimiento:
1. Ejecutar el lanzador de slam-toolbox:<br>
`ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<ruta_al_archivo_de configuración> use_sim_time:=true`<br>

2. Ejecutar el lanzador de nav2:
`ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true`

Para visualizar el mapa de costo que utiliza nav2 para definir la evasión de obstáculos en tiempo real, agregar un segundo visualizador <b>map</b> en rviz y seleccionar el tópico <b>/global_costmap/costmap</b>