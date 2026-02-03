cmake_minimum_required(VERSION 3.19)
project(newui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "D:/QT projects/opencv/build2/install/x64/vc17/lib")
find_package(OpenCV REQUIRED CONFIG)
find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Quick Gui Multimedia)
set(SDL3_INCLUDE_DIR "D:/QT projects/SDL3-devel-3.2.28-VC/SDL3-3.2.28/include")
set(SDL3_LIB_DIR     "D:/QT projects/SDL3-devel-3.2.28-VC/SDL3-3.2.28/lib/x64")
set(SDL3_LIB_NAME    SDL3)  # name of the library without lib prefix or extension

include_directories(${SDL3_INCLUDE_DIR})
link_directories(${SDL3_LIB_DIR})
qt_standard_project_setup()
qt_add_resources(QML_RESOURCES 
qml.qrc)
qt_add_executable(newui
    main.cpp
    videostreamer.h
    opencvimageprovider.h
    opencvimageprovider.cpp
    videostreamer.cpp
    ${QML_RESOURCES}
    receiver.h receiver.cpp
    link.h link.cpp
    VEDTP.h VEDTP.cpp
    joystickcontroller.h
    joystickcontroller.cpp
    datalogger.h datalogger.cpp
)

target_link_libraries(newui
    PRIVATE
        Qt6::Core
        Qt6::Widgets
        Qt6::Quick
        Qt6::Gui
        opencv_core
        opencv_imgproc
        opencv_highgui
        opencv_videoio
        ${SDL3_LIB_NAME}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE USE_SDL3)
