
cmake_minimum_required(VERSION 2.8.12)

project(move4d-visibility-grid)

set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

if(UNIX)
	if(EXISTS "/usr/local/lib/OGRE/cmake")

	  set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
	  set(OGRE_SAMPLES_INCLUDEPATH "/usr/local/share/OGRE/samples/Common/include/") # We could just *assume* that developers uses this basepath : /usr/local

	elseif(EXISTS "/usr/lib/OGRE/cmake")

	  set(CMAKE_MODULE_PATH "/usr/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
	  set(OGRE_SAMPLES_INCLUDEPATH "/usr/share/OGRE/samples/Common/include/") # Otherwise, this one

	elseif(EXISTS "/usr/share/OGRE/cmake")

	  set(CMAKE_MODULE_PATH "/usr/share/OGRE/cmake/modules;${CMAKE_MODULE_PATH}")
	  set(OGRE_SAMPLES_INCLUDEPATH "/usr/share/OGRE/samples/Common/include/") # Otherwise, this one

	else ()
	  message(SEND_ERROR "Failed to find module path.")
	endif(EXISTS "/usr/local/lib/OGRE/cmake")
endif(UNIX)

find_package(libmove3d REQUIRED)
find_package(OGRE 1.9 REQUIRED)
find_package(move4d REQUIRED)
find_package(move4d-gui REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(SRCS
    src/VisibilityGrid.cpp
    src/VisibilityCell.cpp
    src/VisibilityGridCreator.cpp
    src/VisibilityGridLoader.cpp
    src/create_entities.cpp
    src/VisibilityPlanner.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRCS})
target_link_libraries(${PROJECT_NAME} move4d ${LIBS} move3d move4d-gui-common ${OGRE_LIBRARIES})

add_executable(test src/test.cpp)
target_link_libraries(test ${PROJECT_NAME})

install (TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

