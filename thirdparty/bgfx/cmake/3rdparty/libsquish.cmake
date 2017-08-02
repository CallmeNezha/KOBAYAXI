
if( TARGET squish )
	return()
endif()

file( GLOB SQUISH_SOURCES ${BIMG_DIR}/3rdparty/libsquish/*.cpp ${BIMG_DIR}/3rdparty/libsquish/*.h ${BIMG_DIR}/3rdparty/libsquish/*.inl )

add_library( squish STATIC ${SQUISH_SOURCES} )
target_include_directories( squish PUBLIC ${BIMG_DIR}/3rdparty )
set_target_properties( squish PROPERTIES FOLDER "bgfx/3rdparty" )
