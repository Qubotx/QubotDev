## tortubot en ROS2 Jazzy y Gazebo Harmonic

Paquete de pruebas para el control del robot educativo TortuBot en entorno de simulación.<br>
El paquete ha sido creado siguiendo el procedimiento descrito por @ArticulatedRobotics (https://www.youtube.com/@ArticulatedRobotics). Gracias por la valiosa información!<br>

Para una explicación detallada del código y el paso a paso para ponerlo en operación: link <br>

Para ejecutar la simulación:
--------------------------------
Se debe indicar la ruta absoluta al archivo de mundo de GazeboSim (archivo .sdf).
Para descargar modelos y mundos personalizados desarrollados por la comunidad, visitar este sitio: https://app.gazebosim.org/dashboard

En un mundo customizado:<br>
`ros2 launch tortubot sim.launch.py world:=<ruta_absoluta_al_archivo_sdf_sin_commillas>`<br>

Para enviar órdenes de movimiento por medio de teclado:
---------------------------------
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`<br>



