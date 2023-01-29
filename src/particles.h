#pragma once

#include <smmintrin.h>
#include "world.h"  // For WORLD_SIZE_X and _Y


typedef struct _DfBitmap DfBitmap;


// Possible optimizations:
// * Split the extra m_particles array so that there is a smaller one per row of the grid.
// * Switch to fixed point maths so the number of bytes needed to represent a particle can be reduced.
// * Reduce x and y to 8-bit values by storing the position as an offset from the top-left of the cell they are in.
// * Move vx and vy into a different array because they aren't needed in 99.7% of collision checks.
// * Use Morton indexing for the grid
// * Split the grid into chunks that fit in L1 cache (each ~3000 particles). Process collisions within the
//   chunk in one pass, then between chunks in another (only need to consider cells on the boundary of the
//   chunk, which should only be about 0.1 of them).
// * Store cells in the grid as an array of indices to particles and a count of the number of particles.
//   then make special case versions of HandleAnyCollisions() for each combination of possible numbers of particles.


struct Particle {
    union {
        struct {
            float x, y;
            float vx, vy;
        };
        __m128 sse;
    };
};


static float const PARTICLE_RADIUS = 0.354f;
static float const INVALID_PARTICLE_X = -1000.0f;


class Particles {
public:
    static unsigned const GRID_RES_X = 512;
    static unsigned const GRID_RES_Y = (GRID_RES_X * WORLD_SIZE_Y) / WORLD_SIZE_X;

    struct PList {
        Particle p;
        unsigned nextIdx;

        bool IsEmpty() { return p.x == INVALID_PARTICLE_X; }
    };

private:
    bool m_showHistogram;
    void HandleCollision(Particle *p1, Particle *p2, float distSqrd);
    void HandleAnyCollisions(PList *cell, PList *otherCell);
    void HandleAnyCollisionsSelf(PList *cell);

public:
    static const unsigned NUM_PARTICLES = 80000;
    static const unsigned SPEED_HISTOGRAM_NUM_BINS = 20;

    PList m_grid[GRID_RES_X * GRID_RES_Y];  // A 2D array of PLists. When a cell is empty, p.x == INVALID_PARTICLE_X and next == NULL.
    PList m_particles[NUM_PARTICLES];    // Extra particles not stored directly in the grid. Unlike when in the grid, when a PList is unused (ie is on the free list), then p.x != INVALID_PARTICLE_X. 
    unsigned m_firstFreeIdx;

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
