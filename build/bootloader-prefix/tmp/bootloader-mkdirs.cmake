# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/svolkov/esp/esp-idf/components/bootloader/subproject"
  "D:/ESP_Projects/uart_async_rxtxtasks/build/bootloader"
  "D:/ESP_Projects/uart_async_rxtxtasks/build/bootloader-prefix"
  "D:/ESP_Projects/uart_async_rxtxtasks/build/bootloader-prefix/tmp"
  "D:/ESP_Projects/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp"
  "D:/ESP_Projects/uart_async_rxtxtasks/build/bootloader-prefix/src"
  "D:/ESP_Projects/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/ESP_Projects/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/ESP_Projects/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
