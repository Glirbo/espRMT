# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/mikkel/esp/esp-idf/components/bootloader/subproject"
  "/home/mikkel/ai-projects/RMTArm/esp32-idf/build/bootloader"
  "/home/mikkel/ai-projects/RMTArm/esp32-idf/build/bootloader-prefix"
  "/home/mikkel/ai-projects/RMTArm/esp32-idf/build/bootloader-prefix/tmp"
  "/home/mikkel/ai-projects/RMTArm/esp32-idf/build/bootloader-prefix/src/bootloader-stamp"
  "/home/mikkel/ai-projects/RMTArm/esp32-idf/build/bootloader-prefix/src"
  "/home/mikkel/ai-projects/RMTArm/esp32-idf/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mikkel/ai-projects/RMTArm/esp32-idf/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mikkel/ai-projects/RMTArm/esp32-idf/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
