cmake_minimum_required(VERSION 3.19)
project(ORCA LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 👉 Point to your OpenCV install
set(OpenCV_DIR
    "C:/Users/vijay/Downloads/opencv-4.12.0/build2/install"
)

find_package(OpenCV REQUIRED CONFIG)
find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Quick Gui Multimedia Core5Compat)
#set(SDL3_INCLUDE_DIR "C:/Users/vijay/Downloads/SDL3-devel-3.2.28-VC/SDL3-3.2.28/include")
#set(SDL3_LIB_DIR     "C:/Users/vijay/Downloads/SDL3-devel-3.2.28-VC/SDL3-3.2.28/lib/x64")
#set(SDL3_LIB_NAME    SDL3)  # name of the library without lib prefix or extension
set(SDL3_ROOT "C:/Users/vijay/Downloads/SDL3-devel-3.2.28-VC/SDL3-3.2.28")
# Include headers
include_directories(${SDL3_ROOT}/include)

# Find SDL3 library (lib/x64)
set(SDL3_LIB "${SDL3_ROOT}/lib/x64/SDL3.lib")

# Make SDL3 an imported target
add_library(SDL3::SDL3 UNKNOWN IMPORTED)
set_target_properties(SDL3::SDL3 PROPERTIES
    IMPORTED_LOCATION "${SDL3_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${SDL3_ROOT}/include"
)

#include_directories(${SDL3_INCLUDE_DIR})
#link_directories(${SDL3_LIB_DIR})
qt_standard_project_setup()
qt_add_resources(QML_RESOURCES 
qml.qrc)
qt_add_executable(ORCA
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

target_link_libraries(ORCA
    PRIVATE
        Qt6::Core
        Qt6::Widgets
        Qt6::Quick
        Qt6::Gui
        Qt6::Core5Compat
        opencv_core
        opencv_imgproc
        opencv_highgui
        opencv_videoio
        SDL3::SDL3   # <-- link SDL3 here
)
#target_compile_definitions(${PROJECT_NAME} PRIVATE USE_SDL3)
# --- Copy SDL3.dll to build folder ---
add_custom_command(TARGET ORCA POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${SDL3_ROOT}/lib/x64/SDL3.dll"
    $<TARGET_FILE_DIR:ORCA>
)
if(WIN32)
    # 1. Define the OpenCV Binary Path
    # Since you already set OpenCV_DIR, we point to the 'bin' folder inside it
    set(OPENCV_BIN_DIR "${OpenCV_DIR}/x64/vc17/bin") # Adjust 'vc16' if using a different VS version

    # 2. Automated Copy: Post-Build
    # This copies the OpenCV DLLs (like opencv_world.dll) to your EXE folder
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${OPENCV_BIN_DIR}"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
        COMMENT "Copying OpenCV runtime binaries to build folder..."
    )
endif()



include(GNUInstallDirs)

install(TARGETS ORCA
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET ORCA
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
