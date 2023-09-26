cmake_minimum_required(VERSION 3.8)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
project(ros2_xsens_mtw_wrapper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)
# Compile as C++14
# add_compile_options(-std=c++14)

# find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)


# list(INSERT CMAKE_MODULE_PATH
#             0
#             "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vi_interfaces REQUIRED)


list(INSERT CMAKE_MODULE_PATH
            0
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
            

find_package(Xsens REQUIRED)


set(CMAKE_INSTALL_RPATH "/usr/local/xsens/lib/")
        

add_executable(${PROJECT_NAME}_node src/main.cpp        
               src/Wrapper.cpp
                src/MtwCallback.cpp
                src/Synchronizer.cpp
                src/WirelessMasterCallback.cpp
                src/utils.cpp

               )




# file(GLOB XSENS_LIBS_SO /usr/local/xsens/lib/*.so)

set(Xsens_INCLUDE_DIRS /usr/local/xsens/include/)

target_include_directories(${PROJECT_NAME}_node
        PUBLIC include
        ${Xsens_INCLUDE_DIRS}
        # ${Xsens_INCLUDE_DIRS}/../examples/mtsdk/xda_cpp/)
        )



file(GLOB XSENS_SELECTED_LIBS /usr/local/xsens/lib/libxsensdeviceapi.so /usr/local/xsens/lib/libxstypes.so)


target_link_libraries(${PROJECT_NAME}_node
        #  /usr/local/xsens/lib/libxsensdeviceapi.so
        #  /usr/local/xsens/lib/libxstypes.so
        ${XSENS_SELECTED_LIBS}

         )
      

ament_target_dependencies(${PROJECT_NAME}_node rclcpp geometry_msgs vi_interfaces tf2 tf2_ros )



install(TARGETS
  ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()