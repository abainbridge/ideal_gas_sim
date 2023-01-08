#include "world.h"

// Project headers
#include "particles.h"
#include "walls.h"

// Deadfrog headers
#include "df_bitmap.h"
#include "df_bmp.h"
#include "df_time.h"
#include "df_window.h"

// Standard headers
#include <memory.h>
#include <stdlib.h>


World g_world;


World::World() {
    m_particles = new Particles;

    m_viewOffsetX = 0.0f;
    m_viewOffsetY = 0.0f;
    m_viewScale = 1.0f;

    m_advanceTime = 1.0f / 400.0f;
}


void World::Advance() {
    if (g_window->input.lmb || g_window->input.mmb || g_window->input.rmb) {
        m_viewOffsetX += g_window->input.mouseVelX;
        m_viewOffsetY += g_window->input.mouseVelY;
    }

    if ((g_window->input.mouseVelZ > 0 && m_viewScale < 20.0f) || 
        (g_window->input.mouseVelZ < 0 && m_viewScale > 0.5f))
    {
        double scaleDelta = 1.0f + (g_window->input.mouseVelZ * 0.001f);
        m_viewOffsetX -= WORLD_SIZE_X / 2.0f;
        m_viewOffsetY -= WORLD_SIZE_Y / 2.0f;
        m_viewOffsetX *= scaleDelta;
        m_viewOffsetY *= scaleDelta;
        m_viewOffsetX += WORLD_SIZE_X / 2.0f;
        m_viewOffsetY += WORLD_SIZE_Y / 2.0f;
        m_viewScale *= scaleDelta;
    }

    if (g_window->input.keys[KEY_UP])
        m_advanceTime *= 1.01f;
    if (g_window->input.keys[KEY_DOWN])
        m_advanceTime /= 1.01f;
    m_advanceTime = ClampDouble(m_advanceTime, 5.0e-5, 5.0e-3);

    if (!g_window->input.keys[KEY_SPACE])
        m_particles->Advance();
    else
        SleepMillisec(800);

    m_particles->Advance();
}


void World::Render(DfBitmap *bmp) {
    m_particles->Render(bmp);
}


void World::WorldToScreen(float *x, float *y) {
    *x = (*x * m_viewScale) + m_viewOffsetX;
    *y = (*y * m_viewScale) + m_viewOffsetY;
}
