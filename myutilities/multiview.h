#ifndef MULTIVIEW_H_HEADER_GUARD
#define MULTIVIEW_H_HEADER_GUARD

#include "bgfx.h"

///
class BasicViewController
{
public:
    BasicViewController();
    virtual ~BasicViewController();

    /// Forbit any handle transfer.
    BasicViewController(BasicViewController&) = delete;
    BasicViewController(BasicViewController&&) = delete;

    /// Callback for override
    virtual void update(float _time);

    /// Should not be overwrite
    uint16_t  getId();
    void      processWindowEvent(const entry::WindowState& state);
    uint16_t  createWindow(int32_t _x, int32_t _y, uint32_t _width, uint32_t _height);
    void      destroyWindow();

protected:
    bgfx::FrameBufferHandle m_fbh;
    entry::WindowState      m_win;
};


#endif // !MULTIVIEW_H_HEADER_GUARD