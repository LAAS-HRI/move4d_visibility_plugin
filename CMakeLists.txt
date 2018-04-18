
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
        message(SEND_WARNING "Failed to find module path.")
	endif(EXISTS "/usr/local/lib/OGRE/cmake")
endif(UNIX)

find_package(libmove3d REQUIRED)
find_package(OGRE 1.9)
find_package(move4d REQUIRED)
find_package(move4d-gui)
find_package(Boost REQUIRED COMPONENTS serialization)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include BEFORE)

file(GLOB_RECURSE HDRS
    "include/*.hpp"
)

set(SRCS
    src/VisibilityGrid.cpp
    src/VisibilityCell.cpp
    src/VisibilityGridLoader.cpp
    src/PointingPlanner.cpp
    src/PointingPlannerModule.cpp
    #src/VisibilityPlanner.cpp
    #src/LocationIndicatorPlanner.cpp
)

if(move4d-gui_FOUND AND OGRE_FOUND)
    message("building with ogre and move4d-gui")
    list(APPEND SRCS
    src/VisibilityGridCreator.cpp
    src/create_entities.cpp
    )
    list(APPEND LIBS move4d-gui-common ${OGRE_LIBRARIES} )
endif(move4d-gui_FOUND AND OGRE_FOUND)


add_library(${PROJECT_NAME} SHARED ${SRCS} ${HDRS})
target_link_libraries(${PROJECT_NAME} move4d ${LIBS} move3d boost_serialization)

add_executable(test src/test.cpp)
target_link_libraries(test ${PROJECT_NAME})

install (TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

