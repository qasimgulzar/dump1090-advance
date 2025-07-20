cmake_minimum_required(VERSION 3.31)
project(custom_sdr C)

set(CMAKE_C_STANDARD 11)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
include_directories(/opt/homebrew/Cellar/libusb/1.0.29/include)
include_directories(/opt/homebrew/Cellar/librtlsdr/2.0.2/include)
include_directories(/opt/homebrew/Cellar/libpaho-mqtt/1.3.14/include/)
include_directories(/opt/homebrew/Cellar/cjson/1.7.18/include/)
#include_directories(/opt/homebrew/include)
link_directories(/opt/homebrew/Cellar/libusb/1.0.29/lib)
link_directories(/opt/homebrew/Cellar/cjson/1.7.18/lib)
link_directories(/opt/homebrew/Cellar/librtlsdr/2.0.2/lib)
link_directories(/opt/homebrew/Cellar/libpaho-mqtt/1.3.14/lib/)

add_executable(custom_sdr main.c
        serializers.h
        lib/types.h)
target_link_libraries(custom_sdr ${LIBUSB_LIBRARIES} rtlsdr paho-mqtt3c cjson)
