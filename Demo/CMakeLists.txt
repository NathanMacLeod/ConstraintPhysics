cmake_minimum_required(VERSION 3.23)
find_package(OpenGL REQUIRED)

add_executable(demo)

target_link_libraries(demo
    PRIVATE phyz_physics
    PRIVATE mthz_library
)

target_sources(demo PRIVATE
    src/main.cpp
    src/main_old.cpp
    src/Mesh.cpp
    src/ModelReader.cpp
    src/ProgressBar.cpp
)

target_link_libraries(demo
    PRIVATE
        OpenGL::GL
        X11
        dl
        pthread
)

add_subdirectory(src/Demos)
add_subdirectory(src/renderer)
add_subdirectory(Dependencies)