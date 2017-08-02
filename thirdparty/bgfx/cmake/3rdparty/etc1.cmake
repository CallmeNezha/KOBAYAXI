
if( TARGET etc1 )
	return()
endif()

file( GLOB ETC1_SOURCES ${BIMG_DIR}/3rdparty/etc1/*.cpp ${BIMG_DIR}/3rdparty/etc1/*.h )

add_library( etc1 STATIC ${ETC1_SOURCES} )
target_include_directories( etc1 PUBLIC ${BIMG_DIR}/3rdparty )
set_target_properties( etc1 PROPERTIES FOLDER "bgfx/3rdparty" )
