cmake_minimum_required(VERSION 3.10.0)
project(imgui_scan VERSION 0.1.0 LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)


find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)


set(THIS_PACKAGE_DEPENDENCIES
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    tf2_geometry_msgs
)
foreach(DEPENDENCY ${THIS_PACKAGE_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
endforeach()


# Thư mục include
include_directories(
    include
    ${OPENGL_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    thirdparty/imgui
    thirdparty/imgui/backends
)

# Liệt kê file nguồn ImGui
set(IMGUI_SOURCES
    thirdparty/imgui/imgui.cpp
    thirdparty/imgui/imgui_draw.cpp
    thirdparty/imgui/imgui_tables.cpp
    thirdparty/imgui/imgui_widgets.cpp
    thirdparty/imgui/imgui_demo.cpp
    
    thirdparty/imgui/backends/imgui_impl_glfw.cpp
    thirdparty/imgui/backends/imgui_impl_opengl3.cpp
)


add_library(imgui_lib SHARED 
    ${IMGUI_SOURCES}
    src/GuiWrapper.cpp
)
target_link_libraries(imgui_lib
     ${OPENGL_LIBRARIES}
     ${GLFW_LIBRARIES}
)


add_executable(imgui_scan src/main.cpp src/TFPublish.cpp)
target_link_libraries(imgui_scan imgui_lib)
ament_target_dependencies(imgui_scan ${THIS_PACKAGE_DEPENDENCIES})



ament_package()