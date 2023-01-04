#pragma once

#include "world.h"  // For WORLD_SIZE_X and _Y


typedef struct _DfBitmap DfBitmap;


// Possible optimizations:
// * Indices instead of pointers
// * Split the extra m_particles array so that there is a smaller one per row of the grid.
// * Switched to fixed point maths


struct Particle
{
    float x, y;
    float vx, vy;
};


static float const PARTICLE_RADIUS = 0.354f;
static float const INVALID_PARTICLE_X = -1000.0f;


class Particles
{
public:
    static unsigned const GRID_RES_X = 300;
    static unsigned const GRID_RES_Y = (GRID_RES_X * WORLD_SIZE_Y) / WORLD_SIZE_X;

    struct PList
    {
        Particle p;
        PList *next;

        bool IsEmpty() { return p.x == INVALID_PARTICLE_X; }
    };

private:
    bool m_showHistogram;
    void HandleAnyCollisions(PList *cell, PList *otherCell);

public:
    static const unsigned NUM_PARTICLES = 80000;
    static const unsigned SPEED_HISTOGRAM_NUM_BINS = 20;

    PList m_grid[GRID_RES_X * GRID_RES_Y];  // A 2D array of PLists. When a cell is empty, p.x == INVALID_PARTICLE_X and next == NULL.
    PList m_particles[NUM_PARTICLES];    // Extra particles not stored directly in the grid. Unlike when in the grid, when a PList is unused (ie is on the free list), then p.x != INVALID_PARTICLE_X. 
    PList *m_firstFree;

    unsigned m_speedHistogram[SPEED_HISTOGRAM_NUM_BINS];

    Particles();

    void Advance();
    void Render(DfBitmap *bmp);

    PList *GetPListFromIndices(unsigned x, unsigned y);
    PList *GetPListFromCoords(float x, float y);

    unsigned CountParticlesInCell(unsigned x, unsigned y);
    unsigned Count();
    void AddParticle(Particle *p);
};
