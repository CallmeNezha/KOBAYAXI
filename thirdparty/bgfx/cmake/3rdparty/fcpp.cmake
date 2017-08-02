
if( TARGET fcpp )
	return()
endif()

file( GLOB FCPP_SOURCES ${BGFX_DIR}/3rdparty/fcpp/*.c ${BGFX_DIR}/3rdparty/fcpp/*.h )

add_library( fcpp STATIC ${FCPP_SOURCES} )
target_include_directories( fcpp PUBLIC ${BGFX_DIR}/3rdparty/fcpp )
if( MSVC )
	set_target_properties( fcpp PROPERTIES COMPILE_FLAGS "/W0" )
endif()
set_target_properties( fcpp PROPERTIES FOLDER "bgfx/3rdparty" )
set_source_files_properties( ${BGFX_DIR}/3rdparty/fcpp/usecpp.c PROPERTIES HEADER_FILE_ONLY ON )
