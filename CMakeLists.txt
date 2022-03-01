cmake_minimum_required(VERSION 3.0.2)
project(ret)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ret_application src/ret_application.cpp)
target_link_libraries(ret_application ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ret_application DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
