set(CMAKE_TOOLCHAIN_FILE ./cmake/arduino-cmake/cmake/ArduinoToolchain.cmake)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3") 

project(yakruna C CXX)

#print_board_list()
#print_programmer_list()

set(ARDUINO_DEFAULT_BOARD mega2560) 
set(ARDUINO_DEFAULT_PORT /dev/ttyUSB0) # may be different

set(LIBRARY_SRCS
    libraries/Wire/Wire.cpp
    libraries/Timer4/TimerFour.cpp
    libraries/TimerOne/TimerOne.cpp
    libraries/LiquidCrystal/I2CIO.cpp
    libraries/LiquidCrystal/LCD.cpp
    libraries/LiquidCrystal/LiquidCrystal_I2C.cpp
    libraries/MIDI/MIDI.cpp    
)

set(LIBRARY_HDRS)

include_directories(
    libraries/Wire/
    libraries/Timer4/
    libraries/TimerOne/
    libraries/LiquidCrystal/
    libraries/MIDI/     
)

set(CODE_SRCS
    src/yakruna.cpp
    ${LIBRARY_SRCS}
)

generate_arduino_firmware(yakruna
    SRCS  ${CODE_SRCS}
    HDRS  ${LIBRARY_HDRS}
)
