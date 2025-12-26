cmake_minimum_required(VERSION 3.23)


target_include_directories(demo PRIVATE GLFW/include)
target_include_directories(demo PRIVATE stb_image/include)

if (WIN32)
    target_link_libraries(demo
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/Release/x64/glew32s.lib
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/GLFW/lib-vc2022/glfw3.lib
    )
    target_include_directories(demo PRIVATE GLEW/include)
elseif (UNIX)
    target_link_libraries(demo
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/GLEW/glew-2.3/lib/libGLEW.a
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/GLFW/linux/libglfw3.a
    )
    target_include_directories(demo PRIVATE GLEW/glew-2.3/include)
endif()