cmake_minimum_required(VERSION 3.23)

add_library(mthz_library)
target_sources(mthz_library
    PRIVATE
        src/Mat3.cpp
        src/NMat.cpp
        src/Vec3.cpp
)
target_include_directories(mthz_library PUBLIC src)

if (CMAKE_CXX_FLAGS)
    target_compile_options(mthz_library PRIVATE ${CMAKE_CXX_FLAGS})
endif()