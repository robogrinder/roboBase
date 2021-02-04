project(Aim_Test)

include_directories(../autoAim)

add_executable(
        Aim_test
        Aim_Test.cpp
        ../../Robogrinder_SDK/serial_port.cpp
        ../../Robogrinder_SDK/serial_port.h
        ../autoAim/ArmorFilter.h
        ../autoAim/ArmorFilter.cpp
        ../autoAim/autoAim.cpp
        ../autoAim/autoAim.h

)
target_link_libraries(Aim_test ${OpenCV_LIBS})

