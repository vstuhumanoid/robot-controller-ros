project(AR60x_driver)
add_executable(AR60x_driver AR60x_driver.cpp
        BaseController/BaseController.cpp
        PowerController/PowerController.cpp
        JointsController/JointsController.cpp
        SensorsController/SensorsController.cpp)

target_include_directories(AR60x_driver PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${AR60xHWDriver_INCLUDE_DIRECTORIES})
target_link_libraries(AR60x_driver
        ${catkin_LIBRARIES}
        AR60xHWDriver)

add_executable(config_converter config_converter.cpp)
target_include_directories(config_converter PRIVATE ${AR60xHWDriver_INCLUDE_DIRECTORIES})
target_link_libraries(config_converter
        ${catkin_LIBRARIES}
        AR60xHWDriver)