cmake_minimum_required(VERSION 3.10)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Only build what the app requires. The receiver has no use for WiFi or Bluetooth, and without this
# every managed component is linked in: ESP-Hosted, NimBLE and lwIP are about 290 KB of the binary
# and take internal RAM at start.
idf_build_set_property(MINIMAL_BUILD ON)
project(application)
