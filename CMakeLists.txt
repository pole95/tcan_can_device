cmake_minimum_required(VERSION 3.8)
project(tcan_can_device)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
set(PACKAGE_DEPENDENCIES
  message_logger)

foreach(dep IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()


add_library(${PROJECT_NAME} SHARED
  src/CanDriver.cpp
  src/DeviceCanOpen.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${message_logger_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

# Add clang tooling
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
  add_clang_tooling(
    TARGET ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  PATTERN ".svn"
  EXCLUDE)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)


ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()