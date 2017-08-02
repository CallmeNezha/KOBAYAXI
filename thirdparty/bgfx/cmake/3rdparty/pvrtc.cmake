
if( TARGET pvrtc )
	return()
endif()

file( GLOB SQUISH_SOURCES ${BIMG_DIR}/3rdparty/pvrtc/*.cpp ${BIMG_DIR}/3rdparty/pvrtc/*.h )

add_library( pvrtc STATIC ${SQUISH_SOURCES} )
target_include_directories( pvrtc PUBLIC ${BIMG_DIR}/3rdparty )
set_target_properties( pvrtc PROPERTIES FOLDER "bgfx/3rdparty" )
