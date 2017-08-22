#include "view3d_debug_draw.h"

namespace VIEW3D
{
    void drawObb(
        float _sx, float _sy, float _sz
        , float _ax, float _ay, float _az
        , float _tx, float _ty, float _tz
        , uint32_t _color, bool _wireframe)
    {
        ddPush();
            Obb obb;
            ddSetWireframe(_wireframe);
            ddSetColor(_color);
            bx::mtxSRT(obb.m_mtx, _sx, _sy, _sz, _ax, _ay, _az, _tx, _ty, _tz);
            ddDraw(obb);
        ddPop();
    }

    void drawObb(const float* _mtx, float _sx, float _sy, float _sz, uint32_t _color, bool _wireframe)
    {
        ddPush();
            Obb obb;
            bx::memCopy(obb.m_mtx, _mtx, 16 * sizeof(float));
            ddSetWireframe(_wireframe);
            ddSetColor(_color);
            obb.m_mtx[0] = _mtx[0] * -_sx; obb.m_mtx[1] = _mtx[1] * -_sx; obb.m_mtx[2] = _mtx[2] * -_sx;
            obb.m_mtx[4] = _mtx[4] * -_sy; obb.m_mtx[5] = _mtx[5] * -_sy; obb.m_mtx[6] = _mtx[6] * -_sy;
            obb.m_mtx[8] = _mtx[8] * -_sz; obb.m_mtx[9] = _mtx[9] * -_sz; obb.m_mtx[10] = _mtx[10] * -_sz;
            ddDraw(obb);
        ddPop();
    }

    void drawHMDBox(const float* _mtx, uint32_t _color, bool _wireframe)
    {
        VIEW3D::drawObb(_mtx, 0.17f, 0.09f, 0.07f, _color, _wireframe);
    }

    void drawHandle(const float* _mtx, uint32_t _color, bool _wireframe)
    {
        VIEW3D::drawObb(_mtx, 0.05f, 0.05f, 0.05f, _color, _wireframe);
    }

    void drawCamera(const float* _mtx, uint32_t _color)
    {
        ddPush();
            Sphere sphere = { { _mtx[12], _mtx[13], _mtx[14] }, 0.02f };
            ddSetColor(_color);
            ddSetLod(0);
            ddDraw(sphere);
        ddPop();
    }

    void drawAxis(const float* _mtx, float _length)
    {
        ddPush();
            ddSetTransform(_mtx);
            ddDrawAxis(0.f, 0.f, 0.f, _length);
        ddPop();
    }

}