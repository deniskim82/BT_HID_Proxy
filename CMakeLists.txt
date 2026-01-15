# BT_HID_Proxy - Bluetooth Keyboard to USB HID Proxy
# Target: M5Stamp Pico (ESP32-PICO-D4)

cmake_minimum_required(VERSION 3.16)

# No custom components needed - all code is in main/
# set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bt_hid_proxy)
