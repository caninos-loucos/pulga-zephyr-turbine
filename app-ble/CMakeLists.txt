#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# Uncomment to activate GPS shield, when Pulga-GPS module is attached to the stack
# list(APPEND SHIELD pulga_gps)

# Uncomment to activate LoRa shield, when Pulga-LoRa module is attached to the stack
# list(APPEND SHIELD pulga-lora)

# Uncomment to activate SCD30 shield, a CO2 concentration sensor
# list(APPEND SHIELD scd30)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_sources(app PRIVATE 
                    src/main.c)