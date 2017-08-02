cmake_minimum_required(VERSION 3.5)
project( KOBAYAXI )

set_property( GLOBAL PROPERTY USE_FOLDERS ON )

if( APPLE )
	set( CMAKE_CXX_FLAGS "-ObjC++ --std=c++11" )
elseif(UNIX)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Set output directory wrt file type
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin" )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/arch" )

file( GLOB PROJECT_SOURCE_FILES *.cpp *.h )

add_subdirectory( "thirdparty/bgfx" )
add_executable( ${PROJECT_NAME} ${PROJECT_SOURCE_FILES} )
target_link_libraries( ${PROJECT_NAME} PRIVATE bgfx example-common )