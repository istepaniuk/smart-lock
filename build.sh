#!/bin/bash


arduino-cli compile -b esp8266:esp8266:nodemcuv2 lock.ino
arduino-cli upload -p /dev/hl340 -b esp8266:esp8266:nodemcuv2 lock.ino
