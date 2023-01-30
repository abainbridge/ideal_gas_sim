// Simulates an ideal gas composed of thousands of particles.
// The physics for this simulation came from:
// http://vobarian.com/collisions/2dcollisions2.pdf

// Own header
#include "particles.h"

// Project headers
#include "maths.h"
#include "walls.h"
#include "world.h"

// Deadfrog headers
#include "df_bitmap.h"
#include "df_common.h"
#include "df_window.h"

// Standard headers
#include <math.h>
#include <memory.h>


static float const MAX_INITIAL_SPEED = 120.0f;
static float const RADIUS2 = PARTICLE_RADIUS * 2.0f;


Particles::Particles() {
    m_showHistogram = false;

    // Initialize the grid as empty.
    for (unsigned y = 0; y < GRID_RES_Y; y++) {
        for (unsigned x = 0; x < GRID_RES_X; x++) {
            PList &plist = m_grid[y * GRID_RES_X + x];
            plist.p.x = INVALID_PARTICLE_X;
            plist.nextIdx = -1;
        }
    }

    // Initialize the particle array as empty (it uses a free list)
    for (unsigned i = 0; i < NUM_PARTICLES - 1; i++)
        m_particles[i].nextIdx = i + 1;
    m_particles[NUM_PARTICLES - 1].nextIdx = -1;
    m_firstFreeIdx = 0;

    // Place the particles
    for (unsigned i = 0; i < NUM_PARTICLES; i++) {
        Particle p;
        do {
            p.x = frand(WORLD_SIZE_X * 0.999f);
            p.y = frand(WORLD_SIZE_Y * 0.999f);
        } while (0);// g_world.m_walls->IsWallPixel(p.x, p.y));

        p.vx = MAX_INITIAL_SPEED;
        p.vy = MAX_INITIAL_SPEED;
        p.vx = frand(MAX_INITIAL_SPEED * 2.0f) - MAX_INITIAL_SPEED;
        p.vy = frand(MAX_INITIAL_SPEED * 2.0f) - MAX_INITIAL_SPEED;
        p.vx += MAX_INITIAL_SPEED * 0.2f;
        p.vy += MAX_INITIAL_SPEED * 0.1f;
        PList *plist = GetPListFromCoords(p.x, p.y);
        AddParticle(&p, plist);
    }
}


void Particles::HandleCollision(Particle *p1, Particle *p2, float distSqrd) {
    // Collision normal is the vector between the two particle centers.
    // The only change in velocity of either particle is in the 
    // direction of the collision normal.

    // Calculate collision unit normal and tangent.
    float deltaX = p2->x - p1->x;
    float deltaY = p2->y - p1->y;
    float dist = sqrtf(distSqrd);
    float invDist = 1.0f / dist;
    float normX = deltaX * invDist;
    float normY = deltaY * invDist;
    float tangX = normY;
    float tangY = -normX;

    // Project p1 and p2 velocities onto collision unit normal
    float p1VelNorm = DotProduct(p1->vx, p1->vy, normX, normY);
    float p2VelNorm = DotProduct(p2->vx, p2->vy, normX, normY);

    // Project p1 and p2 velocities onto collision tangent
    float p1VelTang = DotProduct(p1->vx, p1->vy, tangX, tangY);
    float p2VelTang = DotProduct(p2->vx, p2->vy, tangX, tangY);

    // Since all particles have the same mass, the collision
    // simply swaps the particles velocities along the normal.
    // This a no-op. We just swap the variables we use in the
    // calculations below.

    // Convert the scalar normal and tangential velocities back
    // into vectors.
    float p1VelNormX = p2VelNorm * normX;
    float p1VelNormY = p2VelNorm * normY;
    float p1VelTangX = p1VelTang * tangX;
    float p1VelTangY = p1VelTang * tangY;

    float p2VelNormX = p1VelNorm * normX;
    float p2VelNormY = p1VelNorm * normY;
    float p2VelTangX = p2VelTang * tangX;
    float p2VelTangY = p2VelTang * tangY;

    // The final velocities of the particles are now just the
    // sums of the normal and tangential velocity vectors
    p1->vx = p1VelNormX + p1VelTangX;
    p1->vy = p1VelNormY + p1VelTangY;
    p2->vx = p2VelNormX + p2VelTangX;
    p2->vy = p2VelNormY + p2VelTangY;

    // Now move the two particles apart by a few percent of their 
    // embeddedness to prevent the pathological interactions that
    // can occur causing the two particles keep re-colliding every
    // frame.
    float embeddedness = RADIUS2 - dist;
    float pushAmount = embeddedness * 0.5f;
    p1->x -= normX * pushAmount;
    p1->y -= normY * pushAmount;
    p2->x += normX * pushAmount;
    p2->y += normY * pushAmount;
}


static float getDistSqrd(Particle const *a, Particle const *b) {
#if 1
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    return dx * dx + dy * dy;
#else
    __m128 subResult = _mm_sub_ps(a->sse, b->sse);
    __m128 dotProdResult = _mm_dp_ps(subResult, subResult, 0x33);
    return _mm_cvtss_f32(dotProdResult);
#endif
}


void Particles::HandleAnyCollisions(PList *plist, PList *otherPlist) {
    PList *otherPlistOrig = otherPlist;

    while (1) {
        Particle *p1 = &plist->p;
        otherPlist = otherPlistOrig;

        while (1) {
            Particle *p2 = &otherPlist->p;
            float distSqrd = getDistSqrd(p1, p2);
            if (distSqrd < RADIUS2 * RADIUS2) {
                // There has been a collision.
                HandleCollision(p1, p2, distSqrd);
            }

            if (otherPlist->nextIdx == -1)
                break;
            otherPlist = &m_particles[otherPlist->nextIdx];
        }

        if (plist->nextIdx == -1)
            break;
        plist = &m_particles[plist->nextIdx];
    }
}


void Particles::HandleAnyCollisionsSelf(PList *plist) {
    if (plist->nextIdx == -1) return;

    Particle *p1 = &plist->p;
    plist = &m_particles[plist->nextIdx];

    while (1) {
        Particle *p2 = &plist->p;
        float distSqrd = getDistSqrd(p1, p2);
        if (distSqrd < RADIUS2 * RADIUS2) {
            // There has been a collision.
            HandleCollision(p1, p2, distSqrd);
        }

        if (plist->nextIdx == -1)
            break;
        plist = &m_particles[plist->nextIdx];
    }
}


void Particles::Advance() {
    float advanceTime = 0.001f;// g_world.m_advanceTime;

    if (g_window->input.keyDowns[KEY_H])
        m_showHistogram = true;

    if (m_showHistogram)
        memset(m_speedHistogram, 0, sizeof(unsigned) * SPEED_HISTOGRAM_NUM_BINS);

    for (unsigned y = 0; y < GRID_RES_Y; y++) {
        for (unsigned x = 0; x < GRID_RES_X; x++) {
            PList *plistGrid = m_grid + y * GRID_RES_X + x;
            if (plistGrid->IsEmpty())
                continue;

            PList *plist = plistGrid;
            PList *prevPlist = NULL;
            while (1) {
                Particle *p = &plist->p;

                // Increment position and keep particle inside the bounds of the world.
                p->x += p->vx * advanceTime;
                if ((p->x < 0.0f && p->vx < 0.0f) || (p->x > WORLD_SIZE_X && p->vx > 0.0f))
                    p->vx = -p->vx;
                p->y += p->vy * advanceTime;
                if ((p->y < 0.0f && p->vy < 0.0f) || (p->y > WORLD_SIZE_Y && p->vy > 0.0f))
                    p->vy = -p->vy;

                // Update speed histogram
                if (m_showHistogram) {
                    float speed = sqrtf(p->vx * p->vx + p->vy * p->vy);
                    unsigned bin_index = SPEED_HISTOGRAM_NUM_BINS * speed / (3.0f * MAX_INITIAL_SPEED);
                    if (bin_index >= SPEED_HISTOGRAM_NUM_BINS)
                        bin_index = SPEED_HISTOGRAM_NUM_BINS - 1;
                    m_speedHistogram[bin_index]++;
                }

                // Move this particle into another cell, if needed.
                PList *newPlist = GetPListFromCoords(p->x, p->y);
                if (newPlist && plistGrid != newPlist) {
                    AddParticle(p, newPlist);

                    if (plist == plistGrid) {
                        if (plist->nextIdx == -1) {
                            plist->p.x = INVALID_PARTICLE_X;
                            break;
                        }
                        else {
                            unsigned nextIdx = plist->nextIdx;
                            PList *toFree = &m_particles[plist->nextIdx];
                            *plist = m_particles[plist->nextIdx];
                            toFree->nextIdx = m_firstFreeIdx;
                            m_firstFreeIdx = nextIdx;
                        }
                    }
                    else {
                        unsigned toFreeIdx = prevPlist->nextIdx;
                        unsigned nextIdx = plist->nextIdx;
                        prevPlist->nextIdx = plist->nextIdx;
                        plist->nextIdx = m_firstFreeIdx;
                        m_firstFreeIdx = toFreeIdx;
                        if (nextIdx == -1)
                            break;
                        plist = &m_particles[nextIdx];
                    }
                }
                else {
                    prevPlist = plist;
                    if (plist->nextIdx == -1)
                        break;
                    plist = &m_particles[plist->nextIdx];
                }
            } // end while

            // Do collisions.
            {
                PList *otherPlist = NULL;

                otherPlist = GetPListFromIndices(x - 1, y - 1);
                if (otherPlist && !otherPlist->IsEmpty()) HandleAnyCollisions(plistGrid, otherPlist);

                otherPlist = GetPListFromIndices(x, y - 1);
                if (otherPlist && !otherPlist->IsEmpty()) HandleAnyCollisions(plistGrid, otherPlist);

                otherPlist = GetPListFromIndices(x + 1, y - 1);
                if (otherPlist && !otherPlist->IsEmpty()) HandleAnyCollisions(plistGrid, otherPlist);

                otherPlist = GetPListFromIndices(x - 1, y);
                if (otherPlist && !otherPlist->IsEmpty()) HandleAnyCollisions(plistGrid, otherPlist);

                HandleAnyCollisionsSelf(plistGrid);  // Special one - check cell against itself.
            }
        }
    }
}


void Particles::Render(DfBitmap *bmp) {
    static const DfColour col = g_colourWhite;

    for (unsigned y = 0; y < GRID_RES_Y; y++) {
        for (unsigned x = 0; x < GRID_RES_X; x++) {
            PList *plist = m_grid + y * GRID_RES_X + x;
            if (plist->IsEmpty())
                continue;

            while (1) {
                float px = plist->p.x;
                float py = plist->p.y;
                g_world.WorldToScreen(&px, &py);
                if (g_world.m_viewScale < 2.5f)
                    PutPix(bmp, px, py, col);
                else
                    CircleOutline(bmp, px, py, PARTICLE_RADIUS * g_world.m_viewScale, col);
                if (plist->nextIdx == -1)
                    break;
                plist = &m_particles[plist->nextIdx];
            }
        }
    }

    if (m_showHistogram) {
        for (unsigned i = 0; i < SPEED_HISTOGRAM_NUM_BINS; i++) {
            const unsigned barWidth = 4;
            const float scale = 1000.0f / (float)NUM_PARTICLES;
            unsigned h = m_speedHistogram[i] * scale;
            RectFill(bmp, i * barWidth, bmp->height - h, barWidth, h, Colour(255, 99, 99));
        }
    }
}


Particles::PList *Particles::GetPListFromIndices(unsigned x, unsigned y) {
    if (x >= GRID_RES_X || y >= GRID_RES_Y)
        return NULL;
    return m_grid + y * GRID_RES_X + x;
}


Particles::PList *Particles::GetPListFromCoords(float x, float y) {
    static const float xFactor = (float)GRID_RES_X / (float)WORLD_SIZE_X;
    static const float yFactor = (float)GRID_RES_Y / (float)WORLD_SIZE_Y;
    unsigned gridX = x * xFactor;
    unsigned gridY = y * yFactor;
    return GetPListFromIndices(gridX, gridY);
}


unsigned Particles::CountParticlesInCell(unsigned x, unsigned y) {
    PList *plist = m_grid + y * GRID_RES_X + x;
    if (plist->IsEmpty())
        return 0;

    unsigned count = 1;
    while (plist->nextIdx != -1) {
        count++;
        plist = &m_particles[plist->nextIdx];
    }

    return count;
}


unsigned Particles::Count() {
    memset(m_countsPerCell, 0, sizeof(m_countsPerCell));
    unsigned totalNum = 0;
    for (unsigned y = 0; y < GRID_RES_Y; y++) {
        for (unsigned x = 0; x < GRID_RES_X; x++) {
            unsigned thisCount = CountParticlesInCell(x, y);
            m_countsPerCell[thisCount]++;
            totalNum += thisCount;
        }
    }

    return totalNum;
}


void Particles::AddParticle(Particle *p, PList *plist) {
    if (plist->IsEmpty()) {
        plist->p = *p;
        return;
    }

    // Get a PList from the free list
    DebugAssert(m_firstFreeIdx != -1);
    unsigned newIdx = m_firstFreeIdx;
    PList *newPlist = &m_particles[newIdx];
    m_firstFreeIdx = m_particles[m_firstFreeIdx].nextIdx;

    newPlist->p = *p;

    // Insert the new PList at the start of the list for this cell
    newPlist->nextIdx = plist->nextIdx;
    plist->nextIdx = newIdx;
}
