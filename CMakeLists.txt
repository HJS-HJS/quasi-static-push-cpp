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

# 필수 라이브러리 찾기
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(SDL2 REQUIRED)
find_library(SDL2_GFX_LIB SDL2_gfx)
find_package(OpenCV REQUIRED)

# NLOHMANN 경로 추가
# 기본적으로 external 폴더를 fallback 경로로 둠
set(NLOHMANN_JSON_FALLBACK_DIR "${CMAKE_SOURCE_DIR}/external")

# 시스템에 설치된 nlohmann/json.hpp 찾기
find_path(NLOHMANN_JSON_INCLUDE_DIR
    NAMES nlohmann/json.hpp
    PATH_SUFFIXES include
    DOC "Path to nlohmann/json.hpp"
)

# 못 찾았으면 fallback 경로 사용
if(NLOHMANN_JSON_INCLUDE_DIR)
    message(STATUS "Found nlohmann/json.hpp at ${NLOHMANN_JSON_INCLUDE_DIR}")
else()
    set(NLOHMANN_JSON_INCLUDE_DIR ${NLOHMANN_JSON_FALLBACK_DIR})
    message(WARNING "nlohmann/json.hpp not found in system. Using fallback at ${NLOHMANN_JSON_INCLUDE_DIR}")
endif()

# include path 추가
include_directories(${NLOHMANN_JSON_INCLUDE_DIR})


# Eigen 경로 추가
include_directories(${CMAKE_SOURCE_DIR}/include /usr/include/eigen3)

# pybind11 include 경로 정리
execute_process(
    COMMAND python3 -m pybind11 --includes
    OUTPUT_VARIABLE PYBIND11_INCLUDES_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
separate_arguments(PYBIND11_INCLUDES_RAW)

# -I prefix 제거하고 경로만 추출
set(PYBIND11_INCLUDE_DIRS "")
foreach(flag ${PYBIND11_INCLUDES_RAW})
    if(flag MATCHES "^-I(.+)")
        list(APPEND PYBIND11_INCLUDE_DIRS "${CMAKE_MATCH_1}")
    endif()
endforeach()

# include 디렉토리 추가
include_directories(${PYBIND11_INCLUDE_DIRS})

# 공통 include 디렉토리
include_directories(
    ${Python3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PYBIND11_INCLUDE}
    ${BINDINGS_DIR}
    ${SDL2_INCLUDE_DIRS}
)

# 모든 소스 파일 한 번에 검색
file(GLOB_RECURSE SOURCES ${SRC_DIR}/*.cpp)
file(GLOB_RECURSE TEST_FILES ${TEST_DIR}/*.cpp)
file(GLOB_RECURSE PYBINDINGS_SRC ${BINDINGS_DIR}/*.cpp)

# 공통 라이브러리 빌드 (정적 라이브러리, -fPIC 적용)
add_library(quasi_static_lib STATIC ${SOURCES})
target_compile_options(quasi_static_lib PRIVATE -fPIC)
target_link_libraries(quasi_static_lib PUBLIC
    ${SDL2_LIBRARIES}
    ${SDL2_GFX_LIB}
    ${OpenCV_LIBS}
)


# 함수: 테스트 실행 파일 빌드
function(build_test name)
    add_executable(${name} ${TEST_DIR}/${name}.cpp)
    target_link_libraries(${name} PRIVATE quasi_static_lib)
    set_target_properties(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${BIN_DIR})
endfunction()

# Python 바인딩을 위한 공유 라이브러리 빌드
function(build_shared_lib name)
    add_library(${name} MODULE ${PYBINDINGS_SRC})
    target_link_libraries(${name} PRIVATE quasi_static_lib ${Python3_LIBRARIES})
    set(CMAKE_INSTALL_RPATH "\$ORIGIN/lib")
    set_target_properties(${name} PROPERTIES PREFIX "")
    set_target_properties(${name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${OUTPUT_DIR})
endfunction()

# 테스트 실행 파일 빌드
build_test(test_lcp_solver)
build_test(test_viewer)

# Python 바인딩을 위한 공유 라이브러리 빌드
build_shared_lib(quasi_static_push)

# 클린업 규칙 설정
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_BINARY_DIR}/${BIN_DIR};${CMAKE_BINARY_DIR}/${OUTPUT_DIR}")
