# CMake build (optional). On the lab PC, build.bat is simpler.
# Visual Studio 2022 can also open this folder directly: File > Open > Folder.
#
#   Real device : cmake -S . -B build && cmake --build build --config Release
#   Simulated   : cmake -S . -B build-mock -DPHANTOM_MOCK=ON && cmake --build build-mock
cmake_minimum_required(VERSION 3.16)
project(phantom_tools CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(PHANTOM_MOCK "Build against a simulated device (no SDK / hardware needed)" OFF)

if(PHANTOM_MOCK)
  find_package(Threads REQUIRED)
  add_library(phantom_hd STATIC tools/hd_mock/hd_mock.cpp)
  target_include_directories(phantom_hd PUBLIC tools/hd_mock/include)
  target_link_libraries(phantom_hd PUBLIC Threads::Threads)
else()
  if(DEFINED ENV{OH_SDK_BASE})
    file(TO_CMAKE_PATH "$ENV{OH_SDK_BASE}" OH_SDK_BASE)
  else()
    set(OH_SDK_BASE "C:/OpenHaptics/Developer/3.5.0")
  endif()
  find_path(HD_INCLUDE_DIR HD/hd.h PATHS "${OH_SDK_BASE}/include" NO_DEFAULT_PATH)
  find_library(HD_LIBRARY hd PATHS "${OH_SDK_BASE}/lib/x64/Release" "${OH_SDK_BASE}/lib/x64" "${OH_SDK_BASE}/lib/x64/Debug" NO_DEFAULT_PATH)
  if(NOT HD_INCLUDE_DIR OR NOT HD_LIBRARY)
    message(FATAL_ERROR "OpenHaptics not found under ${OH_SDK_BASE}. Set the OH_SDK_BASE environment variable, or configure with -DPHANTOM_MOCK=ON to use the simulator.")
  endif()
  add_library(phantom_hd INTERFACE)
  target_include_directories(phantom_hd INTERFACE "${HD_INCLUDE_DIR}")
  target_link_libraries(phantom_hd INTERFACE "${HD_LIBRARY}")
endif()

# Every folder in apps/ with a main.cpp becomes an executable in bin/.
file(GLOB APP_DIRS LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/apps/*")
foreach(dir ${APP_DIRS})
  if(EXISTS "${dir}/main.cpp")
    get_filename_component(name "${dir}" NAME)
    add_executable(${name} "${dir}/main.cpp")
    target_include_directories(${name} PRIVATE common)
    target_link_libraries(${name} PRIVATE phantom_hd)
    if(MSVC)
      target_compile_definitions(${name} PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS)
      target_compile_options(${name} PRIVATE /W3)
    else()
      target_compile_options(${name} PRIVATE -Wall -Wextra)
    endif()
    set_target_properties(${name} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
      RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin"
      RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin")
  endif()
endforeach()
