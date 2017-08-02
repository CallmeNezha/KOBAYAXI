
if( TARGET glslang )
	return()
endif()

file( GLOB GLSLANG_SOURCES
	${BGFX_DIR}/3rdparty/glslang/glslang/GenericCodeGen/*.cpp
	${BGFX_DIR}/3rdparty/glslang/glslang/MachineIndependent/*.cpp
	${BGFX_DIR}/3rdparty/glslang/glslang/MachineIndependent/preprocessor/*.cpp
	${BGFX_DIR}/3rdparty/glslang/hlsl/*.cpp
	${BGFX_DIR}/3rdparty/glslang/SPIRV/*.cpp
	${BGFX_DIR}/3rdparty/glslang/OGLCompilersDLL/*.cpp
)

if( WIN32 )
	list( APPEND GLSLANG_SOURCES ${BGFX_DIR}/3rdparty/glslang/glslang/OSDependent/Windows/ossource.cpp )
else()
	list( APPEND GLSLANG_SOURCES ${BGFX_DIR}/3rdparty/glslang/glslang/OSDependent/Unix/ossource.cpp )
endif()

add_library( glslang STATIC EXCLUDE_FROM_ALL ${GLSLANG_SOURCES} )
target_include_directories( glslang PUBLIC ${BGFX_DIR}/3rdparty/glslang ${BGFX_DIR}/3rdparty/glslang/glslang/Include ${BGFX_DIR}/3rdparty/glslang/glslang/Public )
set_target_properties( glslang PROPERTIES FOLDER "bgfx/3rdparty" )
