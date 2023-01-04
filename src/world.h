#pragma once


typedef struct _DfBitmap DfBitmap;
class Particles;
class Walls;


static const unsigned WORLD_SIZE_X = 800;
static const unsigned WORLD_SIZE_Y = 600;


class World
{
public:
    Particles *m_particles;
    Walls *m_walls;

    float m_viewOffsetX;
    float m_viewOffsetY;
    float m_viewScale;
    float m_advanceTime;

    World();

    void Advance();
    void Render(DfBitmap *bmp);

    void WorldToScreen(float *x, float *y);
};


extern World g_world;
