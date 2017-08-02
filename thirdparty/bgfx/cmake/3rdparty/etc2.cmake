
if( TARGET etc2 )
	return()
endif()

file( GLOB ETC2_SOURCES ${BIMG_DIR}/3rdparty/etc2/*.cpp ${BIMG_DIR}/3rdparty/etc2/*.h )

add_library( etc2 STATIC ${ETC2_SOURCES} )
target_include_directories( etc2 PUBLIC ${BIMG_DIR}/3rdparty )
set_target_properties( etc2 PROPERTIES FOLDER "bgfx/3rdparty" )
target_link_libraries( etc2 PUBLIC bx )
