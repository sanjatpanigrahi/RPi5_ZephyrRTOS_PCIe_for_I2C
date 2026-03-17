# app/rp1_pcie_i2c/CMakeLists.txt

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rp1_pcie_i2c)

target_sources(app PRIVATE
    src/main.c
    src/rp1_pcie_i2c.c
    src/uart_cmd.c
)

target_include_directories(app PRIVATE include)