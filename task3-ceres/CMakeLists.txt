cmake_minimum_required(VERSION 3.16)
project(ceres_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)
find_package(glog REQUIRED)          
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test1 src/test1.cpp)
add_executable(test2 src/test2.cpp)
add_executable(main src/main.cpp)

target_link_libraries(test1 PRIVATE
        Ceres::ceres
        glog::glog                  

)

target_link_libraries(test2 PRIVATE
        Ceres::ceres
        glog::glog                

)

target_link_libraries(main
        Ceres::ceres
        glog::glog 
        ${OpenCV_LIBS}
)



