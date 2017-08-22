#include "multiview.h"

///
BasicViewController::BasicViewController()
    : m_fbh(BGFX_INVALID_HANDLE)
{
}

///
BasicViewController::~BasicViewController()
{
    destroyWindow();
}

///
uint16_t BasicViewController::createWindow(int32_t _x, int32_t _y, uint32_t _width, uint32_t _height)
{
    entry::WindowHandle handle = entry::createWindow(_x, _y, _width, _height);
    if (entry::isValid(handle))
    {
        char str[256];
        bx::snprintf(str, BX_COUNTOF(str), "Window - handle %d", handle.idx);
        entry::setWindowTitle(handle, str);
        m_win.m_handle = handle;
        return m_win.m_handle.idx;
    }
    return bgfx::kInvalidHandle;
}

///
void BasicViewController::destroyWindow()
{
    if (bgfx::isValid(m_fbh))
    {
        bgfx::destroyFrameBuffer(m_fbh);
        m_fbh.idx = bgfx::kInvalidHandle;

        // Flush destruction of swap chain before destroying window!
        bgfx::frame();
        bgfx::frame();
    }
    if (entry::isValid(m_win.m_handle))
    {
        entry::destroyWindow(m_win.m_handle);
        m_win.m_handle.idx = bgfx::kInvalidHandle;
    }
}

/// Dumb function
void BasicViewController::update(float _time)
{
}

///
uint16_t BasicViewController::getId()
{
    return m_win.m_handle.idx;
}

///
void BasicViewController::processWindowEvent(const entry::WindowState& state)
{
    if (m_win.m_nwh != state.m_nwh
        || m_win.m_width != state.m_width
        || m_win.m_height != state.m_height)
    {
        // When window changes size or native window handle changed
        // frame buffer must be recreated.
        if (bgfx::isValid(m_fbh))
        {
            bgfx::destroyFrameBuffer(m_fbh);
            m_fbh.idx = bgfx::kInvalidHandle;
        }

        m_win.m_nwh = state.m_nwh;
        m_win.m_width = state.m_width;
        m_win.m_height = state.m_height;

        if (NULL != m_win.m_nwh)
        {
            m_fbh = bgfx::createFrameBuffer(m_win.m_nwh, uint16_t(m_win.m_width), uint16_t(m_win.m_height));
        }
        else
        {
            m_win.m_handle.idx = bgfx::kInvalidHandle;
        }
    }
}