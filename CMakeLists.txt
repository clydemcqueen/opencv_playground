cmake_minimum_required(VERSION 3.9)
project(opencv3_playground)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(example_2_1 example_2_1.cpp)
target_link_libraries(example_2_1 ${OpenCV_LIBS})

add_executable(example_2_3 example_2_3.cpp)
target_link_libraries(example_2_3 ${OpenCV_LIBS})

add_executable(lkdemo lkdemo.cpp)
target_link_libraries(lkdemo ${OpenCV_LIBS})

add_executable(mono_odometry mono_odometry.cpp)
target_link_libraries(mono_odometry ${OpenCV_LIBS})

add_executable(example_2_7 example_2_7.cpp)
target_link_libraries(example_2_7 ${OpenCV_LIBS})

add_executable(kalman kalman.cpp)
target_link_libraries(kalman ${OpenCV_LIBS})

add_executable(ekf_drag ekf_drag.cpp)
target_link_libraries(ekf_drag ${OpenCV_LIBS})

add_executable(ekf_imu ekf_imu.cpp)
target_link_libraries(ekf_imu ${OpenCV_LIBS})

add_executable(ekf_drag_imu ekf_imu_drag.cpp)
target_link_libraries(ekf_drag_imu ${OpenCV_LIBS})