# Copyright (c) 2026 Dexforce Inc

#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(perception_2d)
find_package(ament_cmake REQUIRED)

set(PERCEPTION_2D_MAJOR_VERSION 1)
set(PERCEPTION_2D_MINOR_VERSION 0)
set(PERCEPTION_2D_PATCH_VERSION 0)
set(FORCE_DEBUG_BUILD True)
set(PERCEPTION_2D_VERSION ${PERCEPTION_2D_MAJOR_VERSION}.${PERCEPTION_2D_MINOR_VERSION}.${PERCEPTION_2D_PATCH_VERSION})
set(PERCEPTION_2D_SOVERSION ${PERCEPTION_2D_MAJOR_VERSION}.${PERCEPTION_2D_MINOR_VERSION})

# add_compile_options(-Werror)
include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_cartographer_project()
google_enable_testing()

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -std=c++17 -march=native")

set(BOOST_COMPONENTS iostreams)
if(WIN32)
  list(APPEND BOOST_COMPONENTS zlib)
  set(Boost_USE_STATIC_LIBS FALSE)
endif()
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
# 4.3.0 
find_package(OpenCV REQUIRED)

include(FindPkgConfig)
if (NOT WIN32)
  PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)
else()
  find_library(CAIRO_LIBRARIES cairo)
endif()

# Only build the documentation if we can find Sphinx.
# find_package(Sphinx)
# if(SPHINX_FOUND)
#   add_subdirectory("docs")
# endif()

set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/perception_2d/configuration_files
  CACHE PATH ".lua configuration files directory")

install(DIRECTORY configuration_files DESTINATION share/perception_2d/)

set(CARTOGRAPHER_MODEL_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/perception_2d/model
  CACHE PATH ".yaml model files directory")

install(DIRECTORY model DESTINATION share/perception_2d/)

install(DIRECTORY cmake DESTINATION share/perception_2d)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "${PROJECT_SOURCE_DIR}/include/perception_2d/*.hpp")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "${PROJECT_SOURCE_DIR}/src/*.cpp")
#file(GLOB_RECURSE TEST_LIBRARY_HDRS "src/fake_*.hpp" "src/*test_helpers*.hpp" "src/mock_*.hpp")
#file(GLOB_RECURSE TEST_LIBRARY_SRCS "src/fake_*.cpp" "src/*test_helpers*.cpp" "src/mock_*.cpp")
#file(GLOB_RECURSE ALL_TESTS "src/*_test.cpp")
#file(GLOB_RECURSE ALL_EXECUTABLES "src/*_main.cpp")

# Remove dotfiles/-folders that could potentially pollute the build.
# file(GLOB_RECURSE ALL_DOTFILES ".*/*")
# if (ALL_DOTFILES)
  # list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_DOTFILES})
  # list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_DOTFILES})
  # list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_DOTFILES})
  # list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_DOTFILES})
  # list(REMOVE_ITEM ALL_TESTS ${ALL_DOTFILES})
  # list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_DOTFILES})
# endif()
#list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES})
#list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})
#list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
#list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})

set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS})
include_directories(
  include
  ${PROJECT_BINARY_DIR}/include
  "."
)

add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})

configure_file(
  ${PROJECT_SOURCE_DIR}/include/perception_2d/common/config.h.cmake
  ${PROJECT_BINARY_DIR}/perception_2d/common/config.hpp)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${LUA_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBRARIES})

target_link_libraries(${PROJECT_NAME} PUBLIC glog)
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

if(WIN32)
  # Needed to fix conflict with MSVC's error macro.
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DGLOG_NO_ABBREVIATED_SEVERITIES)
endif()
if(MSVC)
  # Needed for VS 2017 5.8
  target_compile_definitions(${PROJECT_NAME} PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE -D_USE_MATH_DEFINES)
endif()

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${CAIRO_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CAIRO_LIBRARIES})

if (NOT WIN32)
  target_link_libraries(${PROJECT_NAME} PUBLIC pthread)
endif()

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT CartographerExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION ${DIR}
  )
endforeach()

foreach(HDR ${INSTALL_GENERATED_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

set(CARTOGRAPHER_CMAKE_DIR share/perception_2d/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cartographer-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/perception_2d/perception_2d-config.cmake
  PATH_VARS CARTOGRAPHER_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/perception_2d
)

install(
  EXPORT CartographerExport
  DESTINATION share/perception_2d/cmake/
  FILE CartographerTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/perception_2d/perception_2d-config.cmake
  DESTINATION share/perception_2d/
)

ament_export_targets(CartographerExport HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Eigen3 OpenCV)

ament_package()

