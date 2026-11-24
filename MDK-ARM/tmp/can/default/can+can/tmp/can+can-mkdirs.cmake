# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/can+can")
  file(MAKE_DIRECTORY "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/can+can")
endif()
file(MAKE_DIRECTORY
  "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/1"
  "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/can+can"
  "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/can+can/tmp"
  "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/can+can/src/can+can-stamp"
  "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/can+can/src"
  "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/can+can/src/can+can-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/can+can/src/can+can-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/RoboMaster/training/week3_can/can/MDK-ARM/tmp/can/default/can+can/src/can+can-stamp${cfgdir}") # cfgdir has leading slash
endif()
