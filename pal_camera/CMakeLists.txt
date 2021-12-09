cmake_minimum_required(VERSION 3.5)
project(pal_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

set(DEPENDENCIES
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
#find_package(opencv REQUIRED)
#find_package(PkgConfig REQUIRED)

#pkg_search_module(PKG_OPENCV REQUIRED opencv)

set(AVAILABLE_ROS_VERSIONS "foxy")

foreach(version $AVAILABLE_ROS_VERSIONS})
        find_path(ROS_LIBRARY_PATH librclcpp.so /opt/ros/${version}/lib)
endforeach()

set(PAL_LIBS  libPAL.so libPAL_CAMERA.so libPAL_DEPTH.so libPAL_DE.so)
set(OPENCV_LIBS  libopencv_calib3d.so.3.4 libopencv_core.so.3.4 libopencv_features2d.so.3.4 libopencv_flann.so.3.4 libopencv_imgcodecs.so.3.4 libopencv_imgproc.so.3.4 libopencv_videoio.so.3.4 libopencv_ximgproc.so.3.4)

include_directories(
        /usr/local/include
        ./include
        #       ${OpenCV_INCLUDE_DIRS}
        )

link_directories(
        ./lib
        ./lib/dreamvu
        ./lib/opencv 
        /usr/local/lib
        ${ROS_LIBRARY_PATH}
        )

link_libraries(
        ${PAL_LIBS}
        ${OPENCV_LIBS}
        )


add_executable(capture src/pal_camera_capture.cpp src/pal_camera_node.cpp)
ament_target_dependencies(capture rclcpp std_msgs cv_bridge image_transport camera_info_manager ${DEPENDENCIES} )
target_link_libraries(capture ${PAL_LIBS})

target_link_libraries(capture
        ${PAL_LIBS}
        ${OPENCV_LIBS}
        )

install(
   TARGETS capture 
   RUNTIME DESTINATION lib/${PROJECT_NAME}                
#   LIBRARY DESTINATION lib/${PROJECT_NAME}                
)

foreach(dir launch)
        install(DIRECTORY ${dir}/
                DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

install(DIRECTORY lib
        DESTINATION .
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(pal_camera)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
