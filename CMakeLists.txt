# Specify the minimum CMake version
cmake_minimum_required(VERSION 3.16)

idf_component_register(
                    SRC_DIRS "src"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_lcd)
