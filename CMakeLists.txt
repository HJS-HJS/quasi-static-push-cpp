cmake_minimum_required(VERSION 3.10)
project(QuasiStaticPush)

# C++ 표준 및 컴파일 옵션 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare")

# 빌드 디렉토리 설정
set(SRC_DIR src)
set(TEST_DIR test)
set(BIN_DIR bin)
set(BINDINGS_DIR bindings)
set(OUTPUT_DIR build)

# Python 개발 환경 찾기 (가장 먼저 실행해야 함)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
include_directories(${Python3_INCLUDE_DIRS})

# SDL2 및 SDL2_gfx 라이브러리 찾기
find_package(SDL2 REQUIRED)
find_library(SDL2_GFX_LIB SDL2_gfx)

# Eigen 라이브러리 포함 경로 추가
include_directories(${CMAKE_SOURCE_DIR}/include /usr/include/eigen3)

# Pybind11 찾기
execute_process(COMMAND python3 -m pybind11 --includes
                OUTPUT_VARIABLE PYBIND11_INCLUDE
                OUTPUT_STRIP_TRAILING_WHITESPACE)
separate_arguments(PYBIND11_INCLUDE)

# 소스 파일 설정
file(GLOB DIAGRAM_SRC ${SRC_DIR}/diagram/*.cpp)
file(GLOB SIMULATION_SRC ${SRC_DIR}/simulation/*.cpp)
file(GLOB VIEWER_SRC ${SRC_DIR}/viewer/*.cpp)
file(GLOB SOLVER_SRC ${SRC_DIR}/solver/*.cpp)
file(GLOB PYBINDINGS_SRC ${BINDINGS_DIR}/*.cpp)
file(GLOB TEST_FILES ${TEST_DIR}/*.cpp)

# 실행 파일 빌드 설정
foreach(test_file ${TEST_FILES})
    get_filename_component(test_name ${test_file} NAME_WE)
    add_executable(${test_name} ${test_file} ${DIAGRAM_SRC} ${SIMULATION_SRC} ${VIEWER_SRC} ${SOLVER_SRC})
    target_link_libraries(${test_name} PRIVATE SDL2::SDL2 ${SDL2_GFX_LIB})
    set_target_properties(${test_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${BIN_DIR})
endforeach()

# Python 바인딩을 위한 공유 라이브러리 빌드
add_library(quasi_static_push MODULE ${DIAGRAM_SRC} ${SIMULATION_SRC} ${VIEWER_SRC} ${SOLVER_SRC} ${PYBINDINGS_SRC})
target_include_directories(quasi_static_push PRIVATE ${PYBIND11_INCLUDE} ${BINDINGS_DIR} ${Python3_INCLUDE_DIRS})
target_link_libraries(quasi_static_push PRIVATE SDL2::SDL2 ${SDL2_GFX_LIB} stdc++ ${Python3_LIBRARIES})
set_target_properties(quasi_static_push PROPERTIES PREFIX "")
set_target_properties(quasi_static_push PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${OUTPUT_DIR})

# 클린업 규칙 설정
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_BINARY_DIR}/${BIN_DIR};${CMAKE_BINARY_DIR}/${OUTPUT_DIR}")
