#!/usr/bin/env python2

"""
Script que crea un paquete de ros de c++
Descomenta las lineas necesarias para compilar
    - add_executable
    - add_dependencies
    - target_link_libraries
Crea la carpeta "src" si es necesario y crea el fichero .cpp correspondiente en ella
"""

from __future__ import print_function

import sys
import os

# TODO: 
CATKIN_CREATE_PKG_CMD = 'catkin_create_pkg'

def create_cpp_file(src_path, pkg_name):
    cpp_file = """
#include <ros/ros.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "<x>");
    return 0;
}
"""
    file_name = pkg_name + '_node.cpp'
    with open(os.path.join(src_path, file_name), 'w') as f:
        f.writelines(cpp_file.replace("<x>", pkg_name + '_nodo'))

def get_path_catkin_ws_src():
    try:
        catkin_ws = os.environ['ROS_WORKSPACE']
        return os.path.join(os.path.expanduser(catkin_ws), 'src')
    except:
        src_path = os.path.join(os.environ['HOME'], 'catkin_ws/src')
        if os.path.exists(src_path):
            return src_path
        else:
            raise Exception('Neno que te falta el catkin_ws')

def uncomment_CMakeList(pkg_path):
    # section_name : comment_lines
    uncomment_sections = [
        # TODO: Buscar una mejor manera de saber que hay que descomentar, hay algunas que se llaman igual la unica diferencia que veo es la de _node
        { 'add_executable(${PROJECT_NAME}_node' : 1 },
        { 'add_dependencies(${PROJECT_NAME}_node' : 1 },
        { 'target_link_libraries(${PROJECT_NAME}_node' : 3 },
    ]

    cmake_list_file = os.path.join(pkg_path, 'CMakeLists.txt')
    aux = cmake_list_file + '.bak'

    with open(aux, 'w') as tmp:
        with open(cmake_list_file, 'r') as f:
            # for line in f.readlines():
            line = f.readline()
            while line != '':
                flag = True
                for sect in uncomment_sections:
                    for k, v in sect.items():
                        if k in line:
                            flag = False
                            if v == 1:
                                tmp.writelines(line[1:])
                            else:
                                for _ in range(v-1):
                                    tmp.writelines(line[1:])
                                    line = f.readline() # avanzamos una
                                tmp.writelines(line[1:])
                if flag:
                    tmp.writelines(line)

                line = f.readline()

    os.remove(cmake_list_file)
    os.rename(aux, cmake_list_file)

def create_pkg(pkg_name, args):
    cmd = '%s %s %s' % (CATKIN_CREATE_PKG_CMD, str(pkg_name), ' '.join(args))
    print('Ejecutando...', cmd)
    os.system(cmd)

def main():
    if len(sys.argv) == 1:
        print('No se han proporcionado argumentos suficientes')
        exit(1)
    # TODO: Se debe comprobar si los argumentos contienen roscpp, o si al menos hay argumentos a parte del nombre del paquete, si no solo crearlo

    # change dir to $HOME/catkin_ws/src
    try:
        src_path = get_path_catkin_ws_src()
        os.chdir(src_path)
    except Exception as e:
        print(e)
    # create package
    pkg_name = sys.argv[1]
    create_pkg(pkg_name, sys.argv[2:])

    # create cpp file
    # TODO: Se debe mirar si existe la carpeta "src" en el paquete
    create_cpp_file(os.path.join(src_path, os.path.join(pkg_name, 'src')), pkg_name)

    # uncomment cmakelist.txt
    try:
        uncomment_CMakeList(os.path.join(src_path, pkg_name))
    except:
        print('Error al modificar el CMakeList.txt')

if __name__ == '__main__':
    main()
