cmake_minimum_required(VERSION 3.0.2)
project(ret)

add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  ur_manipulation
)

catkin_package(
  LIBRARIES
    ret_logger
  CATKIN_DEPENDS
    ur_manipulation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ret_logger
   src/ret_logger.cpp
 )
add_dependencies(ret_logger ${catkin_EXPORTED_TARGETS})
target_link_libraries(ret_logger ${catkin_LIBRARIES})

add_executable(ret_application
                src/ret_application.cpp
                src/ret_logger.cpp
    )
target_link_libraries(ret_application ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ret_application DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
