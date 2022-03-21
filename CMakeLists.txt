#
# Copyright (c) 2022 Federico Zuccardi Merli.
# SPDX-License-Identifier: BSD-3-Clause
#

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(${PROJECT_NAME} PRIVATE
    memI2C.c
)
