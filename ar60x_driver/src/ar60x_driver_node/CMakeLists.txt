#project(ar60x_driver_wtf)
add_executable(ar60x_driver_node ar60x_driver_node.cpp
        BaseController/BaseController.cpp
        PowerController/PowerController.cpp
        JointsController/JointsController.cpp
        SensorsController/SensorsController.cpp)

target_include_directories(ar60x_driver_node PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${AR60xHWDriver_INCLUDE_DIRECTORIES})
target_link_libraries(ar60x_driver_node
        ${catkin_LIBRARIES}
        AR60xHWDriver)