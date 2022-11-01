# Следующие 3 строки шаблонного шаблона должны быть в вашем проекте. # CMakeLists именно в таком порядке, чтобы cmake работал правильно
# 1 строка сообщает CMake минимальную версию, которая требуется для сборки проекта. ESP-IDF разработан для работы с CMake 3.5 или новее. 
# 2 строка задействует остальные функции CMake для настройки проекта, обнаружения всех компонентов и т. д.
# 3 строка создает сам проект и указывает имя проекта. Название проекта используется для конечных двоичных выходных файлов приложения.

cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS c:/users/vozov/desktop/esp32_lib/esp-idf-lib-master/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(controller_RS485+BLE+DHT+AHG)
