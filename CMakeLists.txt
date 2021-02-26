cmake_minimum_required(VERSION 3.5)
project(3DLane)

add_compile_options(-std=c++11)

find_package(OpenCV)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}

)

add_executable(${PROJECT_NAME}_node src/Lane.cc src/Lane_node.cc)
target_link_libraries(${PROJECT_NAME}_node 
    ${OpenCV_LIBRARIES}
)