# The following lines of boilerplate have to be in your project's CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Include the ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project Name
project(RoboPatrol)

# Optional: Add any extra component requirements here
# Generally, 'nimble' and 'mqtt' are included by default in modern IDF