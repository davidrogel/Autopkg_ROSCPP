
# Autogenerador tonto de paquetes para ROSCPP

Autogenerador de paquetes que usan C++ en ROS. Por ahora tiene las siguientes capacidades:

- Permite crear paquetes desde cualquier lugar
- Crea un fichero .cpp con el código básico
- Descomenta las 3 secciones principales para poder compilar

## Instalación

- TODO

## Forma de Uso

Se puede ejecutar tanto dandole permisos de ejecución como con python2.

Uso:
`./autopkg_roscpp.py [pkg_name] [ros_arguments]`

Ejemplo: 
`./autopkg_roscpp.py pacote roscpp`

Después de ejecutar el script bastaría con compilar `cd ~/catkin_ws && catkin_make --only-pkg-with-deps pacote`

