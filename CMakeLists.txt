cmake_minimum_required(VERSION 3.5)
project(hiros_xsens_mtw_wrapper)

# set(CMAKE_CXX_STANDARD 14)

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

# set(Xsens_DIR /usr/local/xsens/)

list(INSERT CMAKE_MODULE_PATH
            0
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
            

find_package(Xsens REQUIRED)

# System dependencies are found with CMake's conventions

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# set(msg_files
#   "msg/Euler.msg"
#   "msg/MIMU.msg"
#   "msg/MIMUArray.msg"
# )

# set(srv_files 
#   "srv/ResetOrientation.srv"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   ${srv_files}
#   DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs 
# )

# ament_export_dependencies(rosidl_default_runtime)


# ament_export_dependencies(geometry_msgs rosidl_default_generators rclcpp sensor_msgs std_msgs tf2 tf2_ros)


# ##############################################################################
# Build ##
# ##############################################################################
# Display additional files in qtcreator
# file(GLOB_RECURSE
#      FILES_LIST
#      "*.launch"
#      "*.msg"
#      "*.srv"
#      "*.xml")


# add_custom_target(additional_files_${PROJECT_NAME} SOURCES ${FILES_LIST})

# add_executable(${PROJECT_NAME}_node
#                src/MtwCallback.cpp
#                src/Synchronizer.cpp
#                src/WirelessMasterCallback.cpp
#                src/Wrapper.cpp
#                src/main.cpp
#                src/utils.cpp)


# add_executable(${PROJECT_NAME}_node src/main.cpp        
#                src/MtwCallback.cpp
#                src/Synchronizer.cpp
#                src/WirelessMasterCallback.cpp
#                src/Wrapper.cpp
        #        src/utils.cpp)
               
add_executable(${PROJECT_NAME}_node src/main.cpp        
               src/Wrapper.cpp
                src/MtwCallback.cpp
                src/Synchronizer.cpp
                src/WirelessMasterCallback.cpp
                src/utils.cpp

               )


# set_target_properties(${PROJECT_NAME}_node
#                       PROPERTIES OUTPUT_NAME
#                                  ${PROJECT_NAME}
#                                  PREFIX
#                                  "")

# Add cmake target dependencies of the executable same as for the library above
# add_dependencies(${PROJECT_NAME}_node
#                  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#                  ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#                       PUBLIC ${catkin_LIBRARIES}
#                       PRIVATE ${Xsens_LIBRARIES})

# set_target_properties(${PROJECT_NAME}_node
#                       PROPERTIES PUBLIC_HEADER include/xsens_mtw/Wrapper.h)

file(GLOB XSENS_LIBS_SO /usr/local/xsens/lib/*.so)

set(Xsens_INCLUDE_DIRS /usr/local/xsens/include/)

target_include_directories(${PROJECT_NAME}_node
        PUBLIC include
        ${Xsens_INCLUDE_DIRS}
        ${Xsens_INCLUDE_DIRS}/../examples/mtsdk/xda_cpp/)


# add_dependencies(${PROJECT_NAME}_node
#         ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )


# target_link_libraries(${PROJECT_NAME}_node
#          ${Xsens_LIBRARIES}
        #  )


file(GLOB XSENS_SELECTED_LIBS /usr/local/xsens/lib/libxsensdeviceapi.so.2022 /usr/local/xsens/lib/libxstypes.so.2022)

# target_link_libraries(${PROJECT_NAME}_node
#          ${XSENS_SELECTED_LIBS})


# target_link_libraries(${PROJECT_NAME}_node
#          ${XSENS_LIBS_SO})

set(CMAKE_INSTALL_RPATH "/usr/local/xsens/lib")  # Replace with the actual library directory


target_link_libraries(${PROJECT_NAME}_node
         /usr/local/xsens/lib/libxsensdeviceapi.so.2022
         /usr/local/xsens/lib/libxstypes.so.2022
         )
         
# message("MyVariable%%%%%%%%%%%%: ${XSENS_LIBS_SO}")


ament_target_dependencies(${PROJECT_NAME}_node rclcpp geometry_msgs vi_interfaces tf2 tf2_ros )


message( ${CMAKE_INSTALL_RPATH})



# set(CMAKE_INSTALL_RPATH "/usr/local/xsens/lib")
# target_link_libraries(${PROJECT_NAME}_node ${XSENS_LIBS_SO})


# ament_target_dependencies(${PROJECT_NAME}_node geometry_msgs rosidl_default_generators rclcpp sensor_msgs std_msgs tf2 tf2_ros)


# ##############################################################################
# Install ##
# ##############################################################################
# install(TARGETS ${PROJECT_NAME}_node
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(FILES include/xsens_mtw/Wrapper.h
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



install(TARGETS
  ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()