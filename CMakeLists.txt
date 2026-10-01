cmake_minimum_required(VERSION 3.16)
project(Sapphire VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable Qt's automatic build systems
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find Qt6 Widgets module
find_package(Qt6 COMPONENTS Widgets REQUIRED)

# Compile the main file
add_executable(Sapphire main.cpp)

# Link against Qt6
target_link_libraries(Sapphire PRIVATE Qt6::Widgets)
