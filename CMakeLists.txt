cmake_minimum_required(VERSION 2.8)

set(CXX_DISABLE_WERROR 1)
include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME SakiController)
set(PROJECT_DESCRIPTION "SakiController")
set(PROJECT_URL "")

setup_project()

# Enable C++11
if(NOT WIN32)
  add_definitions(-std=c++0x)
endif()

# Use jrl-cmakemoduels to get mc_rtc depency
set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir prefix")
add_required_dependency(mc_rtc)

add_subdirectory(src)

install(FILES etc/SakiController.conf DESTINATION "${MC_RTC_LIBDIR}/mc_controller/etc")

setup_project_finalize()