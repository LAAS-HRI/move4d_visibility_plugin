
cmake_minimum_required(VERSION 2.8.12)

project(move4d-visibility-grid)

set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(move4d REQUIRED)
find_package(move4d-gui REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(SRCS
    src/VisibilityGrid.cpp
    src/VisibilityCell.cpp
    src/VisibilityModule.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRCS})
target_link_libraries(${PROJECT_NAME} move4d ${LIBS} move4d-gui-common)

#add_executable(test test.cpp)
#target_link_libraries(test move4d-softmotion-trajectory)

install (TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

