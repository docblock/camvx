# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/andreas/esp-idfs/v5.5.1/esp-idf/components/bootloader/subproject"
  "/home/andreas/development/idfsource/camvx/build/bootloader"
  "/home/andreas/development/idfsource/camvx/build/bootloader-prefix"
  "/home/andreas/development/idfsource/camvx/build/bootloader-prefix/tmp"
  "/home/andreas/development/idfsource/camvx/build/bootloader-prefix/src/bootloader-stamp"
  "/home/andreas/development/idfsource/camvx/build/bootloader-prefix/src"
  "/home/andreas/development/idfsource/camvx/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/andreas/development/idfsource/camvx/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/andreas/development/idfsource/camvx/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
