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

PROG_NAME = 'autopkg_roscpp'
PROG_DESC = 'Crea paquetes para ros en su version c++, genera un fichero c++ basico y descomenta las lineas principales de CMakeList.txt para poder compilar'

CATKIN_CREATE_PKG_CMD = 'catkin_create_pkg'
ROS_WORKSPACE_ENV = 'ROS_WORKSPACE'
HOME_ENV = 'HOME'

# TERM COLORS
RED   = "\033[1;31m"
BLUE  = "\033[1;34m"
CYAN  = "\033[1;36m"
GREEN = "\033[0;32m"
RESET = "\033[0;0m"
BOLD    = "\033[;1m"
REVERSE = "\033[;7m"

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
        catkin_ws = os.environ[ROS_WORKSPACE_ENV]
        return os.path.join(os.path.expanduser(catkin_ws), 'src')
    except:
        src_path = os.path.join(os.environ[HOME_ENV], 'catkin_ws/src')
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

def arguments():
    import argparse

    parser = argparse.ArgumentParser(PROG_NAME, description=PROG_DESC)

    parser.add_argument('pkg_name', type=str, help='Nombre del paquete')
    parser.add_argument('arguments', type=str, nargs='+', help='Dependencias del paquete. Ej: roscpp')

    return parser.parse_args()

def main():
    argx = arguments()

    # change dir to $HOME/catkin_ws/src
    try:
        src_path = get_path_catkin_ws_src()
        os.chdir(src_path)
    except Exception as e:
        print(e)
    # create package
    pkg_name = argx.pkg_name
    create_pkg(pkg_name, argx.arguments)

    # create cpp file
    if 'roscpp' in argx.arguments:
        path_to_pkg_src = os.path.join(src_path, os.path.join(pkg_name, 'src'))
        if not os.path.exists(path_to_pkg_src):
            os.mkdir(path_to_pkg_src) # no deberia llegar aqui nunca
        create_cpp_file(path_to_pkg_src, pkg_name)

        # uncomment cmakelist.txt
        try:
            uncomment_CMakeList(os.path.join(src_path, pkg_name))
        except:
            print('Error al modificar el CMakeList.txt')
    else:
        sys.stdout.write(RED)
        print('No se ha especificado roscpp como dependencia no se va a crear el archivo .cpp ni descomentar el CMakeList.txt')
        sys.stdout.write(RESET)
if __name__ == '__main__':
    main()
