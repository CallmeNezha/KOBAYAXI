
if( TARGET iqa )
	return()
endif()

file( GLOB IQA_SOURCES ${BIMG_DIR}/3rdparty/iqa/source/*.c ${BIMG_DIR}/3rdparty/iqa/include/*.h )

add_library( iqa STATIC ${IQA_SOURCES} )
target_include_directories( iqa PUBLIC ${BIMG_DIR}/3rdparty/iqa/include )
set_target_properties( iqa PROPERTIES FOLDER "bgfx/3rdparty" )
