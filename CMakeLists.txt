include(FetchContent)

cmake_minimum_required(VERSION 3.22)

project(Physics2D
    VERSION 0.1.0
    LANGUAGES C CXX
)

set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

add_library(compiler_flags INTERFACE)
target_compile_features(compiler_flags INTERFACE cxx_std_17)

# add compiler warning flags just when building this project via
# the BUILD_INTERFACE genex
set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
target_compile_options(compiler_flags INTERFACE
  "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-g;-O3;-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused>>"
  "$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# FetchContent_Declare(
#     SDL2
#     URL https://github.com/libsdl-org/SDL/archive/refs/tags/release-2.30.7.tar.gz
# )
# FetchContent_MakeAvailable(SDL2)
#find_package(SDL2 REQUIRED)
#find_package(SDL2_gfx REQUIRED)

set(SDL2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDL2)
add_subdirectory(${SDL2_DIR} EXCLUDE_FROM_ALL)

set(SDL2_GFX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDL2_gfx)
add_library(SDL2_gfx STATIC
    ${SDL2_GFX_DIR}/SDL2_gfxPrimitives.c
    ${SDL2_GFX_DIR}/SDL2_imageFilter.c
    ${SDL2_GFX_DIR}/SDL2_rotozoom.c
)
target_link_libraries(SDL2_gfx PUBLIC ${SDL2_LIBRARIES})
target_include_directories(SDL2_gfx PUBLIC ${SDL2_GFX_DIR} ${SDL2_DIR}/include)  

set(SDL_IMAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDL_image)
add_subdirectory(${SDL_IMAGE_DIR} EXCLUDE_FROM_ALL)

add_library(imgui STATIC
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_impl_sdl2.cpp
    imgui/imgui_impl_sdlrenderer2.cpp
)

target_link_libraries(imgui PUBLIC ${SDL2_LIBRARIES})
target_include_directories(imgui PUBLIC imgui ${SDL2_DIR}/include)

add_library(implot STATIC
    implot/implot.cpp
    implot/implot_demo.cpp
    implot/implot_items.cpp
)

target_include_directories(implot PUBLIC implot)
target_include_directories(implot PRIVATE imgui)
target_link_libraries(implot PRIVATE imgui)

set(CMAKE_CXX_STANDARD 17)

set(PHYSICS2D_SOURCES
    src/application.cc
    src/application.h
    src/broad_phase.cc
    src/broad_phase.h
    src/collision.cc
    src/collision.h
    src/config.h
    src/control.h
    src/editor.cc
    src/editor.h
    src/link.cc
    src/link.h
    src/main.cc
    src/narrow_phase.cc
    src/narrow_phase.h
    src/render.cc
    src/render.h
    src/rigid_body.cc
    src/rigid_body.h
    src/settings.cc
    src/settings.h
    src/shape.h
    src/shape.cc
    src/transform2.h
    src/transform2.cc
    src/utils.cc
    src/utils.h
    src/vector2.cc
    src/vector2.h
    src/world.cc
    src/world.h
)

add_executable(physics2d ${PHYSICS2D_SOURCES})

set_target_properties(physics2d PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
)

target_link_libraries(physics2d PUBLIC SDL2::SDL2 SDL2::SDL2main SDL2_image::SDL2_image SDL2_gfx imgui implot compiler_flags)

# add_subdirectory(src)
add_subdirectory(tests)
