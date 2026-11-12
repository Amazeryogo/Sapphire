cmake_minimum_required(VERSION 3.16)
project(Sapphire LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(SAPPHIRE_BUILD_APP   "Build the Qt sandbox application" ON)
option(SAPPHIRE_BUILD_TESTS "Build the physics-core tests"     ON)

# ---- Physics core: plain C++17, no Qt ----
add_library(sapphire_core STATIC
    src/core/physics_object.cpp
    src/core/collision.cpp
    src/core/constraints.cpp
    src/core/spatial_hash.cpp
    src/core/world.cpp
    src/core/thermal.cpp
    src/core/fluid.cpp
    src/core/scene.cpp
    src/core/serialization.cpp
)
target_include_directories(sapphire_core PUBLIC src/core)
if(MSVC)
    target_compile_options(sapphire_core PRIVATE /W4)
else()
    target_compile_options(sapphire_core PRIVATE -Wall -Wextra)
endif()

# ---- Qt application ----
if(SAPPHIRE_BUILD_APP)
    find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)
    add_library(sapphire_ui STATIC
        src/app/camera.h
        src/app/renderer.h
        src/app/renderer.cpp
        src/app/physics_canvas.h
        src/app/physics_canvas.cpp
        src/app/main_window.h
        src/app/main_window.cpp
    )
    target_include_directories(sapphire_ui PUBLIC src/app)
    target_link_libraries(sapphire_ui PUBLIC sapphire_core Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets)
    set_target_properties(sapphire_ui PROPERTIES AUTOMOC ON)
    if(NOT MSVC)
        target_compile_options(sapphire_ui PRIVATE -Wall -Wextra)
    endif()
    if(APPLE)
        # Apple marks OpenGL as deprecated; it is still what the 3D view uses.
        target_compile_definitions(sapphire_ui PUBLIC GL_SILENCE_DEPRECATION)
    endif()

    add_executable(sapphire WIN32 src/app/main.cpp)
    target_link_libraries(sapphire PRIVATE sapphire_ui)
endif()

# ---- Tests ----
if(SAPPHIRE_BUILD_TESTS)
    enable_testing()
    add_executable(sapphire_tests tests/core_tests.cpp)
    target_link_libraries(sapphire_tests PRIVATE sapphire_core)
    add_test(NAME core COMMAND sapphire_tests)

    # Headless end-to-end test of the window (needs Qt Test).
    if(SAPPHIRE_BUILD_APP)
        find_package(Qt6 QUIET COMPONENTS Test)
        if(Qt6Test_FOUND)
            add_executable(sapphire_ui_smoke tests/ui_smoke.cpp)
            target_link_libraries(sapphire_ui_smoke PRIVATE sapphire_ui Qt6::Test)
            add_test(NAME ui_smoke COMMAND sapphire_ui_smoke)
            set_tests_properties(ui_smoke PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
        endif()
    endif()
endif()
