
if( TARGET forsyth-too )
	return()
endif()

file( GLOB FORSYTH-TOO_SOURCES ${BGFX_DIR}/3rdparty/forsyth-too/*.cpp ${BGFX_DIR}/3rdparty/forsyth-too/*.h )

add_library( forsyth-too STATIC ${FORSYTH-TOO_SOURCES} )
target_include_directories( forsyth-too PUBLIC ${BGFX_DIR}/3rdparty )
set_target_properties( forsyth-too PROPERTIES FOLDER "bgfx/3rdparty" )
