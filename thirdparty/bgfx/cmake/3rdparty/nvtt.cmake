
if( TARGET nvtt )
	return()
endif()

file(
	GLOB
	NVTT_SOURCES
	${BIMG_DIR}/3rdparty/nvtt/bc6h/*.cpp
	${BIMG_DIR}/3rdparty/nvtt/bc6h/*.h
	${BIMG_DIR}/3rdparty/nvtt/bc7/*.cpp
	${BIMG_DIR}/3rdparty/nvtt/bc7/*.h
	${BIMG_DIR}/3rdparty/nvtt/nvcore/*.h
	${BIMG_DIR}/3rdparty/nvtt/nvcore/*.inl
	${BIMG_DIR}/3rdparty/nvtt/nvmath/*.cpp
	${BIMG_DIR}/3rdparty/nvtt/nvmath/*.h
	${BIMG_DIR}/3rdparty/nvtt/*.cpp
	${BIMG_DIR}/3rdparty/nvtt/*.h
)

add_library( nvtt STATIC ${NVTT_SOURCES} )
target_include_directories( nvtt PUBLIC ${BIMG_DIR}/3rdparty ${BIMG_DIR}/3rdparty/nvtt )
set_target_properties( nvtt PROPERTIES FOLDER "bgfx/3rdparty" )
target_link_libraries( nvtt PUBLIC bx )
