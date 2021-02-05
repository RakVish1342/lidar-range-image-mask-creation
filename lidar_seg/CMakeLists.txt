## Need CPP 17 due to: CPP <filesystem> and <experimental/filesystem>
## https://stackoverflow.com/questions/55474690/stdfilesystem-has-not-been-declared-after-including-experimental-filesystem
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(MY_GRAND_PROJECT)
find_package(PCL 1.8 REQUIRED COMPONENTS 
    common 
    io 
    visualization
    )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(rangeImageVisualization ../src/rangeImageVisualization.cpp)
target_link_libraries(rangeImageVisualization ${PCL_LIBRARIES})

 
