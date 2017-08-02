
# Ensure the directory exists
if( NOT IS_DIRECTORY ${BX_DIR} )
	message( SEND_ERROR "Could not load bx, directory does not exist. ${BX_DIR}" )
	return()
endif()

# Grab the bx source files
file( GLOB BX_SOURCES ${BX_DIR}/src/*.cpp )
if(BX_AMALGAMATED)
	set(BX_NOBUILD ${BX_SOURCES})
	list(REMOVE_ITEM BX_NOBUILD ${BX_DIR}/src/amalgamated.cpp)
	foreach(BX_SRC ${BX_NOBUILD})
		set_source_files_properties( ${BX_SRC} PROPERTIES HEADER_FILE_ONLY ON )
	endforeach()
else()
	set_source_files_properties( ${BX_DIR}/src/amalgamated.cpp PROPERTIES HEADER_FILE_ONLY ON )
endif()

# Create the bx target
add_library( bx STATIC ${BX_SOURCES} )

# Link against psapi on Windows
if( WIN32 )
	target_link_libraries( bx PUBLIC psapi )
endif()

# Add include directory of bx
target_include_directories( bx PUBLIC ${BX_DIR}/include )

# Build system specific configurations
if( MSVC )
	target_include_directories( bx PUBLIC ${BX_DIR}/include/compat/msvc )
elseif( MINGW )
	target_include_directories( bx PUBLIC ${BX_DIR}/include/compat/mingw )
elseif( APPLE )
	target_include_directories( bx PUBLIC ${BX_DIR}/include/compat/osx )
endif()

# All configurations
target_compile_definitions( bx PUBLIC "__STDC_LIMIT_MACROS" )
target_compile_definitions( bx PUBLIC "__STDC_FORMAT_MACROS" )
target_compile_definitions( bx PUBLIC "__STDC_CONSTANT_MACROS" )

# Additional dependencies on Unix
if( UNIX AND NOT APPLE )
	# Threads
	find_package( Threads )
	target_link_libraries( bx ${CMAKE_THREAD_LIBS_INIT} dl )

	# Real time (for clock_gettime)
	target_link_libraries( bx rt )
endif()

# Put in a "bgfx" folder in Visual Studio
set_target_properties( bx PROPERTIES FOLDER "bgfx" )

# Export debug build as "bxd"
set_target_properties( bx PROPERTIES OUTPUT_NAME_DEBUG "bxd" )
