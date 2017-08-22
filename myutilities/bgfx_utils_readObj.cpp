/*
 * Copyright 2011-2017 Branimir Karadzic. All rights reserved.
 * License: https://github.com/bkaradzic/bgfx#license-bsd-2-clause
 */

#include "common.h"

#include <tinystl/allocator.h>
#include <tinystl/vector.h>
#include <tinystl/string.h>
#include <tinystl/unordered_map.h>
#include <tinystl/unordered_set.h>
namespace stl = tinystl;

#include <bgfx/bgfx.h>
#include <bx/commandline.h>
#include <bx/endian.h>
#include <bx/fpumath.h>
#include <bx/readerwriter.h>
#include <bx/string.h>
#include "entry/entry.h"
#include <forsyth-too/forsythtriangleorderoptimizer.h>
#include <ib-compress/indexbufferdecompression.h>

#include "bgfx_utils.h"

#include <bimg/decode.h>
#include <algorithm>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void* load(bx::FileReaderI* _reader, bx::AllocatorI* _allocator, const char* _filePath, uint32_t* _size)
{
	if (bx::open(_reader, _filePath) )
	{
		uint32_t size = (uint32_t)bx::getSize(_reader);
		void* data = BX_ALLOC(_allocator, size);
		bx::read(_reader, data, size);
		bx::close(_reader);
		if (NULL != _size)
		{
			*_size = size;
		}
		return data;
	}
	else
	{
		DBG("Failed to open: %s.", _filePath);
	}

	if (NULL != _size)
	{
		*_size = 0;
	}

	return NULL;
}

void* load(const char* _filePath, uint32_t* _size)
{
	return load(entry::getFileReader(), entry::getAllocator(), _filePath, _size);
}

void unload(void* _ptr)
{
	BX_FREE(entry::getAllocator(), _ptr);
}

static const bgfx::Memory* loadMem(bx::FileReaderI* _reader, const char* _filePath)
{
	if (bx::open(_reader, _filePath) )
	{
		uint32_t size = (uint32_t)bx::getSize(_reader);
		const bgfx::Memory* mem = bgfx::alloc(size+1);
		bx::read(_reader, mem->data, size);
		bx::close(_reader);
		mem->data[mem->size-1] = '\0';
		return mem;
	}

	DBG("Failed to load %s.", _filePath);
	return NULL;
}

static void* loadMem(bx::FileReaderI* _reader, bx::AllocatorI* _allocator, const char* _filePath, uint32_t* _size)
{
	if (bx::open(_reader, _filePath) )
	{
		uint32_t size = (uint32_t)bx::getSize(_reader);
		void* data = BX_ALLOC(_allocator, size);
		bx::read(_reader, data, size);
		bx::close(_reader);

		if (NULL != _size)
		{
			*_size = size;
		}
		return data;
	}

	DBG("Failed to load %s.", _filePath);
	return NULL;
}

static bgfx::ShaderHandle loadShader(bx::FileReaderI* _reader, const char* _name, const char* _basepath)
{
	char filePath[512];

	const char* shaderPath = "???";

	switch (bgfx::getRendererType() )
	{
	case bgfx::RendererType::Noop:
	case bgfx::RendererType::Direct3D9:  shaderPath = "shaders/dx9/";   break;
	case bgfx::RendererType::Direct3D11:
	case bgfx::RendererType::Direct3D12: shaderPath = "shaders/dx11/";  break;
	case bgfx::RendererType::Gnm:        shaderPath = "shaders/pssl/";  break;
	case bgfx::RendererType::Metal:      shaderPath = "shaders/metal/"; break;
	case bgfx::RendererType::OpenGL:     shaderPath = "shaders/glsl/";  break;
	case bgfx::RendererType::OpenGLES:   shaderPath = "shaders/essl/";  break;
	case bgfx::RendererType::Vulkan:     shaderPath = "shaders/spirv/"; break;

	case bgfx::RendererType::Count:
		BX_CHECK(false, "You should not be here!");
		break;
	}
    if(_basepath)
    {
        bx::strCopy(filePath, BX_COUNTOF(filePath), _basepath);
        bx::strCat(filePath, BX_COUNTOF(filePath), shaderPath);
    }
    else
        bx::strCopy(filePath, BX_COUNTOF(filePath), shaderPath);

	bx::strCat(filePath, BX_COUNTOF(filePath), _name);
	bx::strCat(filePath, BX_COUNTOF(filePath), ".bin");

	return bgfx::createShader(loadMem(_reader, filePath) );
}

bgfx::ShaderHandle loadShader(const char* _basepath, const char* _name)
{
	return loadShader(entry::getFileReader(), _basepath, _name);
}

bgfx::ProgramHandle loadProgram(bx::FileReaderI* _reader, const char* _vsName, const char* _fsName, const char* _basepath)
{
	bgfx::ShaderHandle vsh = loadShader(_reader, _vsName, _basepath);
	bgfx::ShaderHandle fsh = BGFX_INVALID_HANDLE;
	if (NULL != _fsName)
	{
		fsh = loadShader(_reader, _fsName, _basepath);
	}

	return bgfx::createProgram(vsh, fsh, true /* destroy shaders when program is destroyed */);
}

bgfx::ProgramHandle loadProgram(const char* _vsName, const char* _fsName, const char* _basepath)
{
	return loadProgram(entry::getFileReader(), _vsName, _fsName, _basepath);
}

static void imageReleaseCb(void* _ptr, void* _userData)
{
	BX_UNUSED(_ptr);
	bimg::ImageContainer* imageContainer = (bimg::ImageContainer*)_userData;
	bimg::imageFree(imageContainer);
}

bgfx::TextureHandle loadTexture(bx::FileReaderI* _reader, const char* _filePath, uint32_t _flags, uint8_t _skip, bgfx::TextureInfo* _info)
{
	BX_UNUSED(_skip);
	bgfx::TextureHandle handle = BGFX_INVALID_HANDLE;

	uint32_t size;
	void* data = load(_reader, entry::getAllocator(), _filePath, &size);
	if (NULL != data)
	{
		bimg::ImageContainer* imageContainer = bimg::imageParse(entry::getAllocator(), data, size);

		if (NULL != imageContainer)
		{
			const bgfx::Memory* mem = bgfx::makeRef(
					  imageContainer->m_data
					, imageContainer->m_size
					, imageReleaseCb
					, imageContainer
					);
			unload(data);

			if (imageContainer->m_cubeMap)
			{
				handle = bgfx::createTextureCube(
					  uint16_t(imageContainer->m_width)
					, 1 < imageContainer->m_numMips
					, imageContainer->m_numLayers
					, bgfx::TextureFormat::Enum(imageContainer->m_format)
					, _flags
					, mem
					);
			}
			else if (1 < imageContainer->m_depth)
			{
				handle = bgfx::createTexture3D(
					  uint16_t(imageContainer->m_width)
					, uint16_t(imageContainer->m_height)
					, uint16_t(imageContainer->m_depth)
					, 1 < imageContainer->m_numMips
					, bgfx::TextureFormat::Enum(imageContainer->m_format)
					, _flags
					, mem
					);
			}
			else
			{
				handle = bgfx::createTexture2D(
					  uint16_t(imageContainer->m_width)
					, uint16_t(imageContainer->m_height)
					, 1 < imageContainer->m_numMips
					, imageContainer->m_numLayers
					, bgfx::TextureFormat::Enum(imageContainer->m_format)
					, _flags
					, mem
					);
			}

			if (NULL != _info)
			{
				bgfx::calcTextureSize(
					  *_info
					, uint16_t(imageContainer->m_width)
					, uint16_t(imageContainer->m_height)
					, uint16_t(imageContainer->m_depth)
					, imageContainer->m_cubeMap
					, 1 < imageContainer->m_numMips
					, imageContainer->m_numLayers
					, bgfx::TextureFormat::Enum(imageContainer->m_format)
					);
			}
		}
	}

	return handle;
}

bgfx::TextureHandle loadTexture(const char* _name, uint32_t _flags, uint8_t _skip, bgfx::TextureInfo* _info)
{
	return loadTexture(entry::getFileReader(), _name, _flags, _skip, _info);
}

bimg::ImageContainer* imageLoad(const char* _filePath, bgfx::TextureFormat::Enum _dstFormat)
{
	uint32_t size = 0;
	void* data = loadMem(entry::getFileReader(), entry::getAllocator(), _filePath, &size);

	return bimg::imageParse(entry::getAllocator(), data, size, bimg::TextureFormat::Enum(_dstFormat) );
}

void calcTangents(void* _vertices, uint16_t _numVertices, bgfx::VertexDecl _decl, const uint16_t* _indices, uint32_t _numIndices)
{
	struct PosTexcoord
	{
		float m_x;
		float m_y;
		float m_z;
		float m_pad0;
		float m_u;
		float m_v;
		float m_pad1;
		float m_pad2;
	};

	float* tangents = new float[6*_numVertices];
	bx::memSet(tangents, 0, 6*_numVertices*sizeof(float) );

	PosTexcoord v0;
	PosTexcoord v1;
	PosTexcoord v2;

	for (uint32_t ii = 0, num = _numIndices/3; ii < num; ++ii)
	{
		const uint16_t* indices = &_indices[ii*3];
		uint32_t i0 = indices[0];
		uint32_t i1 = indices[1];
		uint32_t i2 = indices[2];

		bgfx::vertexUnpack(&v0.m_x, bgfx::Attrib::Position,  _decl, _vertices, i0);
		bgfx::vertexUnpack(&v0.m_u, bgfx::Attrib::TexCoord0, _decl, _vertices, i0);

		bgfx::vertexUnpack(&v1.m_x, bgfx::Attrib::Position,  _decl, _vertices, i1);
		bgfx::vertexUnpack(&v1.m_u, bgfx::Attrib::TexCoord0, _decl, _vertices, i1);

		bgfx::vertexUnpack(&v2.m_x, bgfx::Attrib::Position,  _decl, _vertices, i2);
		bgfx::vertexUnpack(&v2.m_u, bgfx::Attrib::TexCoord0, _decl, _vertices, i2);

		const float bax = v1.m_x - v0.m_x;
		const float bay = v1.m_y - v0.m_y;
		const float baz = v1.m_z - v0.m_z;
		const float bau = v1.m_u - v0.m_u;
		const float bav = v1.m_v - v0.m_v;

		const float cax = v2.m_x - v0.m_x;
		const float cay = v2.m_y - v0.m_y;
		const float caz = v2.m_z - v0.m_z;
		const float cau = v2.m_u - v0.m_u;
		const float cav = v2.m_v - v0.m_v;

		const float det = (bau * cav - bav * cau);
		const float invDet = 1.0f / det;

		const float tx = (bax * cav - cax * bav) * invDet;
		const float ty = (bay * cav - cay * bav) * invDet;
		const float tz = (baz * cav - caz * bav) * invDet;

		const float bx = (cax * bau - bax * cau) * invDet;
		const float by = (cay * bau - bay * cau) * invDet;
		const float bz = (caz * bau - baz * cau) * invDet;

		for (uint32_t jj = 0; jj < 3; ++jj)
		{
			float* tanu = &tangents[indices[jj]*6];
			float* tanv = &tanu[3];
			tanu[0] += tx;
			tanu[1] += ty;
			tanu[2] += tz;

			tanv[0] += bx;
			tanv[1] += by;
			tanv[2] += bz;
		}
	}

	for (uint32_t ii = 0; ii < _numVertices; ++ii)
	{
		const float* tanu = &tangents[ii*6];
		const float* tanv = &tangents[ii*6 + 3];

		float normal[4];
		bgfx::vertexUnpack(normal, bgfx::Attrib::Normal, _decl, _vertices, ii);
		float ndt = bx::vec3Dot(normal, tanu);

		float nxt[3];
		bx::vec3Cross(nxt, normal, tanu);

		float tmp[3];
		tmp[0] = tanu[0] - normal[0] * ndt;
		tmp[1] = tanu[1] - normal[1] * ndt;
		tmp[2] = tanu[2] - normal[2] * ndt;

		float tangent[4];
		bx::vec3Norm(tangent, tmp);

		tangent[3] = bx::vec3Dot(nxt, tanv) < 0.0f ? -1.0f : 1.0f;
		bgfx::vertexPack(tangent, true, bgfx::Attrib::Tangent, _decl, _vertices, ii);
	}

	delete [] tangents;
}

struct Aabb
{
	float m_min[3];
	float m_max[3];
};

struct Obb
{
	float m_mtx[16];
};

struct Sphere
{
	float m_center[3];
	float m_radius;
};

struct Primitive
{
	uint32_t m_startIndex;
	uint32_t m_numIndices;
	uint32_t m_startVertex;
	uint32_t m_numVertices;

	Sphere m_sphere;
	Aabb m_aabb;
	Obb m_obb;
};

typedef stl::vector<Primitive> PrimitiveArray;

struct Group
{
	Group()
	{
		reset();
	}

	void reset()
	{
		m_vbh.idx = bgfx::kInvalidHandle;
		m_ibh.idx = bgfx::kInvalidHandle;
		m_prims.clear();
	}

	bgfx::VertexBufferHandle m_vbh;
	bgfx::IndexBufferHandle m_ibh;
	Sphere m_sphere;
	Aabb m_aabb;
	Obb m_obb;
	PrimitiveArray m_prims;
};

namespace bgfx
{
	int32_t read(bx::ReaderI* _reader, bgfx::VertexDecl& _decl, bx::Error* _err = NULL);
}

struct Mesh
{
	void load(bx::ReaderSeekerI* _reader)
	{
#define BGFX_CHUNK_MAGIC_VB  BX_MAKEFOURCC('V', 'B', ' ', 0x1)
#define BGFX_CHUNK_MAGIC_IB  BX_MAKEFOURCC('I', 'B', ' ', 0x0)
#define BGFX_CHUNK_MAGIC_IBC BX_MAKEFOURCC('I', 'B', 'C', 0x0)
#define BGFX_CHUNK_MAGIC_PRI BX_MAKEFOURCC('P', 'R', 'I', 0x0)

		using namespace bx;
		using namespace bgfx;

		Group group;

		bx::AllocatorI* allocator = entry::getAllocator();

		uint32_t chunk;
		bx::Error err;
		while (4 == bx::read(_reader, chunk, &err)
		&&     err.isOk() )
		{
			switch (chunk)
			{
			case BGFX_CHUNK_MAGIC_VB:
				{
					read(_reader, group.m_sphere);
					read(_reader, group.m_aabb);
					read(_reader, group.m_obb);

					read(_reader, m_decl);

					uint16_t stride = m_decl.getStride();

					uint16_t numVertices;
					read(_reader, numVertices);
					const bgfx::Memory* mem = bgfx::alloc(numVertices*stride);
					read(_reader, mem->data, mem->size);

					group.m_vbh = bgfx::createVertexBuffer(mem, m_decl);
				}
				break;

			case BGFX_CHUNK_MAGIC_IB:
				{
					uint32_t numIndices;
					read(_reader, numIndices);
					const bgfx::Memory* mem = bgfx::alloc(numIndices*2);
					read(_reader, mem->data, mem->size);
					group.m_ibh = bgfx::createIndexBuffer(mem);
				}
				break;

			case BGFX_CHUNK_MAGIC_IBC:
				{
					uint32_t numIndices;
					bx::read(_reader, numIndices);

					const bgfx::Memory* mem = bgfx::alloc(numIndices*2);

					uint32_t compressedSize;
					bx::read(_reader, compressedSize);

					void* compressedIndices = BX_ALLOC(allocator, compressedSize);

					bx::read(_reader, compressedIndices, compressedSize);

					ReadBitstream rbs( (const uint8_t*)compressedIndices, compressedSize);
					DecompressIndexBuffer( (uint16_t*)mem->data, numIndices / 3, rbs);

					BX_FREE(allocator, compressedIndices);

					group.m_ibh = bgfx::createIndexBuffer(mem);
				}
				break;

			case BGFX_CHUNK_MAGIC_PRI:
				{
					uint16_t len;
					read(_reader, len);

					stl::string material;
					material.resize(len);
					read(_reader, const_cast<char*>(material.c_str() ), len);

					uint16_t num;
					read(_reader, num);

					for (uint32_t ii = 0; ii < num; ++ii)
					{
						read(_reader, len);

						stl::string name;
						name.resize(len);
						read(_reader, const_cast<char*>(name.c_str() ), len);

						Primitive prim;
						read(_reader, prim.m_startIndex);
						read(_reader, prim.m_numIndices);
						read(_reader, prim.m_startVertex);
						read(_reader, prim.m_numVertices);
						read(_reader, prim.m_sphere);
						read(_reader, prim.m_aabb);
						read(_reader, prim.m_obb);

						group.m_prims.push_back(prim);
					}

					m_groups.push_back(group);
					group.reset();
				}
				break;

			default:
				DBG("%08x at %d", chunk, bx::skip(_reader, 0) );
				break;
			}
		}
	}

	void unload()
	{
		for (GroupArray::const_iterator it = m_groups.begin(), itEnd = m_groups.end(); it != itEnd; ++it)
		{
			const Group& group = *it;
			bgfx::destroyVertexBuffer(group.m_vbh);

			if (bgfx::isValid(group.m_ibh) )
			{
				bgfx::destroyIndexBuffer(group.m_ibh);
			}
		}
		m_groups.clear();
	}

	void submit(uint8_t _id, bgfx::ProgramHandle _program, const float* _mtx, uint64_t _state) const
	{
		if (BGFX_STATE_MASK == _state)
		{
			_state = 0
				| BGFX_STATE_RGB_WRITE
				| BGFX_STATE_ALPHA_WRITE
				| BGFX_STATE_DEPTH_WRITE
				| BGFX_STATE_DEPTH_TEST_LESS
				| BGFX_STATE_CULL_CCW
				| BGFX_STATE_MSAA
				;
		}

		bgfx::setTransform(_mtx);
		bgfx::setState(_state);

		for (GroupArray::const_iterator it = m_groups.begin(), itEnd = m_groups.end(); it != itEnd; ++it)
		{
			const Group& group = *it;

			bgfx::setIndexBuffer(group.m_ibh);
			bgfx::setVertexBuffer(0, group.m_vbh);
			bgfx::submit(_id, _program, 0, it != itEnd-1);
		}
	}

	void submit(const MeshState*const* _state, uint8_t _numPasses, const float* _mtx, uint16_t _numMatrices) const
	{
		uint32_t cached = bgfx::setTransform(_mtx, _numMatrices);

		for (uint32_t pass = 0; pass < _numPasses; ++pass)
		{
			bgfx::setTransform(cached, _numMatrices);

			const MeshState& state = *_state[pass];
			bgfx::setState(state.m_state);

			for (uint8_t tex = 0; tex < state.m_numTextures; ++tex)
			{
				const MeshState::Texture& texture = state.m_textures[tex];
				bgfx::setTexture(texture.m_stage
						, texture.m_sampler
						, texture.m_texture
						, texture.m_flags
						);
			}

			for (GroupArray::const_iterator it = m_groups.begin(), itEnd = m_groups.end(); it != itEnd; ++it)
			{
				const Group& group = *it;

				bgfx::setIndexBuffer(group.m_ibh);
				bgfx::setVertexBuffer(0, group.m_vbh);
				bgfx::submit(state.m_viewId, state.m_program, 0, it != itEnd-1);
			}
		}
	}

	bgfx::VertexDecl m_decl;
	typedef stl::vector<Group> GroupArray;
	GroupArray m_groups;
};

Mesh* meshLoad(bx::ReaderSeekerI* _reader)
{
	Mesh* mesh = new Mesh;
	mesh->load(_reader);
	return mesh;
}

Mesh* meshLoad(const char* _filePath)
{
	bx::FileReaderI* reader = entry::getFileReader();
	if (bx::open(reader, _filePath) )
	{
		Mesh* mesh = meshLoad(reader);
		bx::close(reader);
		return mesh;
	}

	return NULL;
}

void meshUnload(Mesh* _mesh)
{
	_mesh->unload();
	delete _mesh;
}

MeshState* meshStateCreate()
{
	MeshState* state = (MeshState*)BX_ALLOC(entry::getAllocator(), sizeof(MeshState) );
	return state;
}

void meshStateDestroy(MeshState* _meshState)
{
	BX_FREE(entry::getAllocator(), _meshState);
}

void meshSubmit(const Mesh* _mesh, uint8_t _id, bgfx::ProgramHandle _program, const float* _mtx, uint64_t _state)
{
	_mesh->submit(_id, _program, _mtx, _state);
}

void meshSubmit(const Mesh* _mesh, const MeshState*const* _state, uint8_t _numPasses, const float* _mtx, uint16_t _numMatrices)
{
	_mesh->submit(_state, _numPasses, _mtx, _numMatrices);
}

Args::Args(int _argc, char** _argv)
	: m_type(bgfx::RendererType::Count)
	, m_pciId(BGFX_PCI_ID_NONE)
{
	bx::CommandLine cmdLine(_argc, (const char**)_argv);

	if (cmdLine.hasArg("gl") )
	{
		m_type = bgfx::RendererType::OpenGL;
	}
	else if (cmdLine.hasArg("vk") )
	{
		m_type = bgfx::RendererType::Vulkan;
	}
	else if (cmdLine.hasArg("noop") )
	{
		m_type = bgfx::RendererType::Noop;
	}
	else if (BX_ENABLED(BX_PLATFORM_WINDOWS) )
	{
		if (cmdLine.hasArg("d3d9") )
		{
			m_type = bgfx::RendererType::Direct3D9;
		}
		else if (cmdLine.hasArg("d3d11") )
		{
			m_type = bgfx::RendererType::Direct3D11;
		}
		else if (cmdLine.hasArg("d3d12") )
		{
			m_type = bgfx::RendererType::Direct3D12;
		}
	}
	else if (BX_ENABLED(BX_PLATFORM_OSX) )
	{
		if (cmdLine.hasArg("mtl") )
		{
			m_type = bgfx::RendererType::Metal;
		}
	}

	if (cmdLine.hasArg("amd") )
	{
		m_pciId = BGFX_PCI_ID_AMD;
	}
	else if (cmdLine.hasArg("nvidia") )
	{
		m_pciId = BGFX_PCI_ID_NVIDIA;
	}
	else if (cmdLine.hasArg("intel") )
	{
		m_pciId = BGFX_PCI_ID_INTEL;
	}
	else if (cmdLine.hasArg("sw") )
	{
		m_pciId = BGFX_PCI_ID_SOFTWARE_RASTERIZER;
	}
}


// cyn
void triangleReorder(uint16_t* _indices, uint32_t _numIndices, uint32_t _numVertices, uint16_t _cacheSize)
{
	uint16_t* newIndexList = new uint16_t[_numIndices];
	Forsyth::OptimizeFaces(_indices, _numIndices, _numVertices, newIndexList, _cacheSize);
	bx::memCopy(_indices, newIndexList, _numIndices * 2);
	delete[] newIndexList;
}


// cyn
Mesh* meshLoad_obj(const char* _filePath)
{
	struct Vector3
	{
		float x;
		float y;
		float z;
	};

	typedef std::vector<Vector3> Vector3Array;

	struct Index3
	{
		int32_t m_position;
		int32_t m_texcoord;
		int32_t m_normal;
		int32_t m_vertexIndex;
		int32_t m_vbc; // Barycentric ID. Holds eigher 0, 1 or 2.
	};

	typedef stl::unordered_map<uint64_t, Index3> Index3Map;

	struct Triangle
	{
		uint64_t m_index[3];
	};

	typedef std::vector<Triangle> TriangleArray;

	struct Group_modified
	{
		uint32_t m_startTriangle;
		uint32_t m_numTriangles;
		std::string m_name;
		std::string m_material;
	};

	typedef std::vector<Group_modified> GroupArray;

	struct GroupSortByMaterial
	{
		bool operator()(const Group_modified& _lhs, const Group_modified& _rhs)
		{
			return _lhs.m_material < _rhs.m_material;
		}
	};

	Mesh* mesh = new Mesh;

	FILE* file = fopen(_filePath, "r");
	if (NULL == file) {
		printf("Unable to open input file '%s'", _filePath);
		exit(bx::kExitFailure);
	}

	long int pos = ftell(file);
	fseek(file, 0L, SEEK_END);
	long int size_ = ftell(file);
	fseek(file, pos, SEEK_SET);

	uint32_t size = (uint32_t)(size_);
	char* data = new char[size + 1];
	size = (uint32_t)fread(data, 1, size, file);
	data[size] = '\0';
	fclose(file);

	Vector3Array positions;
	Vector3Array normals;
	Vector3Array texcoords;
	Index3Map indexMap;
	TriangleArray triangles;
	GroupArray groups;

	uint32_t num = 0;

	Group_modified group;
	group.m_startTriangle = 0;
	group.m_numTriangles = 0;

	char commandLine[2048];
	uint32_t len = sizeof(commandLine);
	int argc;
	char* argv[64];
	const char* next = data;
	do
	{
		next = bx::tokenizeCommandLine(next, commandLine, len, argc, argv, BX_COUNTOF(argv), '\n');
		if (0 < argc)
		{
			if (0 == bx::strCmp(argv[0], "f"))
			{
				Triangle triangle;
				bx::memSet(&triangle, 0, sizeof(Triangle));

				const int numNormals = (int)normals.size();
				const int numTexcoords = (int)texcoords.size();
				const int numPositions = (int)positions.size();
				for (uint32_t edge = 0, numEdges = argc - 1; edge < numEdges; ++edge)
				{
					Index3 index;
					index.m_texcoord = -1;
					index.m_normal = -1;
					index.m_vertexIndex = -1;
					index.m_vbc = 0;

					const char* vertex = argv[edge + 1];
					char* texcoord = const_cast<char*>(bx::strFind(vertex, '/'));
					if (NULL != texcoord)
					{
						*texcoord++ = '\0';

						char* normal = const_cast<char*>(bx::strFind(texcoord, '/'));
						if (NULL != normal)
						{
							*normal++ = '\0';
							const int nn = atoi(normal);
							index.m_normal = (nn < 0) ? nn + numNormals : nn - 1;
						}

						if (*texcoord != '\0')
						{
							const int tex = atoi(texcoord);
							index.m_texcoord = (tex < 0) ? tex + numTexcoords : tex - 1;
						}
					}

					const int pos = atoi(vertex);
					index.m_position = (pos < 0) ? pos + numPositions : pos - 1;

					uint64_t hash0 = index.m_position;
					uint64_t hash1 = uint64_t(index.m_texcoord) << 20;
					uint64_t hash2 = uint64_t(index.m_normal) << 40;
					uint64_t hash3 = uint64_t(index.m_vbc) << 60;
					uint64_t hash = hash0^hash1^hash2^hash3;

					stl::pair<Index3Map::iterator, bool> result = indexMap.insert(stl::make_pair(hash, index));
					if (!result.second)
					{
						Index3& oldIndex = result.first->second;
						BX_UNUSED(oldIndex);
						BX_CHECK(oldIndex.m_position == index.m_position
							&& oldIndex.m_texcoord == index.m_texcoord
							&& oldIndex.m_normal == index.m_normal
							, "Hash collision!"
						);
					}

					switch (edge)
					{
					case 0:
					case 1:
					case 2:
						triangle.m_index[edge] = hash;
						if (2 == edge)
						{
							triangles.push_back(triangle);
						}
						break;

					default:
						triangle.m_index[1] = triangle.m_index[2];
						triangle.m_index[2] = hash;
						triangles.push_back(triangle);
						break;
					}
				}
			}
			else if (0 == bx::strCmp(argv[0], "g"))
			{
				group.m_name = argv[1];
			}
			else if (*argv[0] == 'v')
			{
				group.m_numTriangles = (uint32_t)(triangles.size()) - group.m_startTriangle;
				if (0 < group.m_numTriangles)
				{
					groups.push_back(group);
					group.m_startTriangle = (uint32_t)(triangles.size());
					group.m_numTriangles = 0;
				}

				if (0 == bx::strCmp(argv[0], "vn"))
				{
					Vector3 normal;
					normal.x = (float)atof(argv[1]);
					normal.y = (float)atof(argv[2]);
					normal.z = (float)atof(argv[3]);

					normals.push_back(normal);
				}
				else if (0 == bx::strCmp(argv[0], "vp"))
				{
					static bool once = true;
					if (once)
					{
						once = false;
						printf("warning: 'parameter space vertices' are unsupported.\n");
					}
				}
				else if (0 == bx::strCmp(argv[0], "vt"))
				{
					Vector3 texcoord;
					texcoord.x = (float)atof(argv[1]);
					texcoord.y = 0.0f;
					texcoord.z = 0.0f;
					switch (argc)
					{
					case 4:
						texcoord.z = (float)atof(argv[3]);
					case 3:
						texcoord.y = (float)atof(argv[2]);
						break;

					default:
						break;
					}

					texcoords.push_back(texcoord);
				}
				else
				{
					float px = (float)atof(argv[1]);
					float py = (float)atof(argv[2]);
					float pz = (float)atof(argv[3]);
					float pw = 1.0f;
					if (argc > 4)
					{
						pw = (float)atof(argv[4]);
					}

					float invW = 1.0 / pw;
					px *= invW;
					py *= invW;
					pz *= invW;

					Vector3 pos;
					pos.x = px;
					pos.y = py;
					pos.z = pz;

					positions.push_back(pos);
				}
			}
			else if (0 == bx::strCmp(argv[0], "usemtl"))
			{
				std::string material(argv[1]);

				if (material != group.m_material)
				{
					group.m_numTriangles = (uint32_t)(triangles.size()) - group.m_startTriangle;
					if (0 < group.m_numTriangles)
					{
						groups.push_back(group);
						group.m_startTriangle = (uint32_t)(triangles.size());
						group.m_numTriangles = 0;
					}
				}

				group.m_material = material;
			}
		}

		++num;
	} while ('\0' != *next);

	group.m_numTriangles = (uint32_t)(triangles.size()) - group.m_startTriangle;
	if (0 < group.m_numTriangles)
	{
		groups.push_back(group);
		group.m_startTriangle = (uint32_t)(triangles.size());
		group.m_numTriangles = 0;
	}

	delete[] data;
	
	std::sort(groups.begin(), groups.end(), GroupSortByMaterial());

	bool hasColor = false;
	bool hasNormal;
	bool hasTexcoord;
	{
		Index3Map::const_iterator it = indexMap.begin();
		hasNormal = -1 != it->second.m_normal;
		hasTexcoord = -1 != it->second.m_texcoord;

		if (!hasTexcoord
			&&  texcoords.size() == positions.size())
		{
			hasTexcoord = true;

			for (Index3Map::iterator jt = indexMap.begin(), jtEnd = indexMap.end(); jt != jtEnd; ++jt)
			{
				jt->second.m_texcoord = jt->second.m_position;
			}
		}

		if (!hasNormal
			&&  normals.size() == positions.size())
		{
			hasNormal = true;

			for (Index3Map::iterator jt = indexMap.begin(), jtEnd = indexMap.end(); jt != jtEnd; ++jt)
			{
				jt->second.m_normal = jt->second.m_position;
			}
		}
	}

	mesh->m_decl.begin();
	mesh->m_decl.add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float);

	if (hasTexcoord)
	{
		mesh->m_decl.add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float);
	}
	
	if (hasNormal)
	{
		mesh->m_decl.add(bgfx::Attrib::Normal, 3, bgfx::AttribType::Float);
	}

	mesh->m_decl.end();

	uint32_t stride = mesh->m_decl.getStride();
	uint8_t* vertexData = new uint8_t[triangles.size() * 3 * stride];
	uint16_t* indexData = new uint16_t[triangles.size() * 3];
	int32_t numVertices = 0;
	int32_t numIndices = 0;
	int32_t numPrimitives = 0;

	uint8_t* vertices = vertexData;
	uint16_t* indices = indexData;

	std::string material = groups.begin()->m_material;

	Group mesh_group;
	Primitive mesh_prim;
	mesh_prim.m_startVertex = 0;
	mesh_prim.m_startIndex = 0;

	uint32_t positionOffset = mesh->m_decl.getOffset(bgfx::Attrib::Position);

	uint32_t ii = 0;
	for (GroupArray::const_iterator groupIt = groups.begin(); groupIt != groups.end(); ++groupIt, ++ii)
	{
		for (uint32_t tri = groupIt->m_startTriangle, end = tri + groupIt->m_numTriangles; tri < end; ++tri)
		{
			if (material != groupIt->m_material
				|| 65533 < numVertices)
			{
				mesh_prim.m_numVertices = numVertices - mesh_prim.m_startVertex;
				mesh_prim.m_numIndices = numIndices - mesh_prim.m_startIndex;
				if (0 < mesh_prim.m_numVertices)
				{
					mesh_group.m_prims.push_back(mesh_prim);
				}

				for (PrimitiveArray::const_iterator primIt = mesh_group.m_prims.begin(); primIt != mesh_group.m_prims.end(); ++primIt)
				{
					const Primitive& prim1 = *primIt;
					triangleReorder(indexData + prim1.m_startIndex, prim1.m_numIndices, numVertices, 32);
				}
				const bgfx::Memory* mem_vertex = bgfx::alloc(numVertices*stride);
				const bgfx::Memory* mem_index = bgfx::alloc(numIndices * 2);

				for (int32_t i = 0; i < numVertices*(int32_t)stride; i++)
				{
					*((mem_vertex->data) + i) = *(vertexData + positionOffset + i);
				}
				
				for (int32_t i = 0; i < numIndices * 2; i++)
				{
					*((mem_index->data) + i) = *((uint8_t*)indexData + i);
				}

				mesh_group.m_vbh = bgfx::createVertexBuffer(mem_vertex, mesh->m_decl);
				mesh_group.m_ibh = bgfx::createIndexBuffer(mem_index);
				mesh->m_groups.push_back(mesh_group);
				mesh_group.reset();

				for (Index3Map::iterator indexIt = indexMap.begin(); indexIt != indexMap.end(); ++indexIt)
				{
					indexIt->second.m_vertexIndex = -1;
				}

				vertices = vertexData;
				indices = indexData;
				numVertices = 0;
				numIndices = 0;
				mesh_prim.m_startVertex = 0;
				mesh_prim.m_startIndex = 0;
				++numPrimitives;

				material = groupIt->m_material;
			}

			Triangle& triangle = triangles[tri];
			for (uint32_t edge = 0; edge < 3; ++edge)
			{
				uint64_t hash = triangle.m_index[edge];
				Index3& index = indexMap[hash];
				if (index.m_vertexIndex == -1)
				{
					index.m_vertexIndex = numVertices++;

					float* position = (float*)(vertices + positionOffset);
					bx::memCopy(position, &positions[index.m_position], 3 * sizeof(float));

					if (hasTexcoord)
					{
						float uv[2];
						bx::memCopy(uv, &texcoords[index.m_texcoord], 2 * sizeof(float));
						bgfx::vertexPack(uv, true, bgfx::Attrib::TexCoord0, mesh->m_decl, vertices);
					}

					if (hasNormal)
					{
						float normal[4];
						bx::vec3Norm(normal, (float*)&normals[index.m_normal]);
						bgfx::vertexPack(normal, true, bgfx::Attrib::Normal, mesh->m_decl, vertices);
					}

					vertices += stride;
				}

				*indices++ = (uint16_t)index.m_vertexIndex;
				++numIndices;
			}
		}

		mesh_prim.m_numVertices = numVertices - mesh_prim.m_startVertex;
		if (0 < mesh_prim.m_numVertices)
		{
			mesh_prim.m_numIndices = numIndices - mesh_prim.m_startIndex;
			mesh_group.m_prims.push_back(mesh_prim);
			mesh_prim.m_startVertex = numVertices;
			mesh_prim.m_startIndex = numIndices;
		}

		BX_TRACE("%3d: s %5d, n %5d, %s\n"
			, ii
			, groupIt->m_startTriangle
			, groupIt->m_numTriangles
			, groupIt->m_material.c_str()
		);
	}

	if (mesh_group.m_prims.size() > 0)
	{
		for (PrimitiveArray::const_iterator primIt = mesh_group.m_prims.begin(); primIt != mesh_group.m_prims.end(); ++primIt)
		{
			const Primitive& prim1 = *primIt;
			triangleReorder(indexData + prim1.m_startIndex, prim1.m_numIndices, numVertices, 32);
		}
		const bgfx::Memory* mem_vertex = bgfx::alloc(numVertices*stride);
		const bgfx::Memory* mem_index = bgfx::alloc(numIndices * 2);

		for (int32_t i = 0; i < numVertices*(int32_t)stride; i++)
		{
			*((mem_vertex->data) + i) = *(vertexData + positionOffset + i);
		}

		for (int32_t i = 0; i < numIndices * 2; i++)
		{
			*((mem_index->data) + i) = *((uint8_t*)indexData + i);
		}

		mesh_group.m_vbh = bgfx::createVertexBuffer(mem_vertex, mesh->m_decl);
		mesh_group.m_ibh = bgfx::createIndexBuffer(mem_index);
		mesh->m_groups.push_back(mesh_group);
	}
	
	delete[] indexData;
	delete[] vertexData;

	return mesh;
}
