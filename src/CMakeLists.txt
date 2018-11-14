cmake_minimum_required(VERSION 2.8.11)

project(PointsToImages)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)

set(OpenCV_DIR "/usr/local/Cellar/opencv/3.3.0_1/share/OpenCV")
find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# Fix a compilation bug under ubuntu 16.04 (Xenial)
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

find_package(VTK REQUIRED)
#include_directories(${VTK_INCLUDE_DIRS})
#include(${VTK_USE_FILE})


#set(HEADERS qtpclviewer.h glviewwindow.h qtglviewer.h plane3dtransforms.h planedetection.h)
set(project_SOURCES main.cpp qtpclviewer.cpp planedetection.cpp glviewwindow.cpp qtglviewer.cpp
    plane3dtransforms.cpp)


add_executable(${PROJECT_NAME} ${project_HEADERS} ${project_SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION "bin")

qt5_use_modules(${PROJECT_NAME} Widgets)
