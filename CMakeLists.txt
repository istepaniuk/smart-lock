cmake_minimum_required(VERSION 3.16)
project(lock)

set(CMAKE_CXX_STANDARD 14)


include_directories(.)
include_directories(~/.arduino15/packages/esp8266/hardware/esp8266/2.7.4/libraries/ESP8266WiFi/src)
include_directories(~/Arduino/libraries/PubSubClient/src)


add_executable(lock
        lock.ino
        RotaryEncoder.cpp
        RotaryEncoder.h)
