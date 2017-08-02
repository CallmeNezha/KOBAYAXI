
if( TARGET edtaa3 )
	return()
endif()

file( GLOB EDTAA3_SOURCES ${BIMG_DIR}/3rdparty/edtaa3/*.cpp ${BIMG_DIR}/3rdparty/edtaa3/*.h )

add_library( edtaa3 STATIC ${EDTAA3_SOURCES} )
target_include_directories( edtaa3 PUBLIC ${BIMG_DIR}/3rdparty )
set_target_properties( edtaa3 PROPERTIES FOLDER "bgfx/3rdparty" )
