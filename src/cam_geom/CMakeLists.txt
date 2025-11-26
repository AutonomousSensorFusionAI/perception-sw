cmake_minimum_required(VERSION 3.10)

project(cam_geom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(app_debug main.cpp camera_geometry.cpp)
add_executable(app_calibration cal.cpp)

# 위에서 찾은 OpenCV 라이브러리를 우리 프로그램에 연결(링크)합니다.
target_link_libraries(app_debug PRIVATE ${OpenCV_LIBS})
target_link_libraries(app_calibration PRIVATE ${OpenCV_LIBS})