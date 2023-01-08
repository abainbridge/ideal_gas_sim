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
            plist.next = NULL;
        }
    }

    // Initialize the particle array as empty (it uses a free list)
    for (unsigned i = 0; i < NUM_PARTICLES - 1; i++)
        m_particles[i].next = m_particles + i + 1;
    m_particles[NUM_PARTICLES - 1].next = NULL;
    m_firstFree = m_particles;

    // Place the particles
    for (unsigned i = 0; i < NUM_PARTICLES; i++) {
        Particle p;
        do {
            p.x = frand(WORLD_SIZE_X * 0.999f);
            p.y = frand(WORLD_SIZE_Y * 0.999f);
        } while (0);// g_world.m_walls->IsWallPixel(p.x, p.y));

        p.vx = MAX_INITIAL_SPEED;
        p.vy = MAX_INITIAL_SPEED;
//         p.vx = frand(MAX_INITIAL_SPEED * 2.0f) - MAX_INITIAL_SPEED;
//         p.vy = frand(MAX_INITIAL_SPEED * 2.0f) - MAX_INITIAL_SPEED;
        p.vx += MAX_INITIAL_SPEED * 0.2f;
        p.vy += MAX_INITIAL_SPEED * 0.1f;
        AddParticle(&p);
    }
}


void Particles::HandleAnyCollisions(PList *plist, PList *otherPlist) {
    bool compareAgainstSelf = plist == otherPlist;
    PList *otherPlistOrig = otherPlist;

    do {
        Particle *p1 = &plist->p;

        if (compareAgainstSelf)
            otherPlist = plist->next; // Special case when checking the cell against itself - we have to skip half to avoid double counting the collisions.
        else
            otherPlist = otherPlistOrig;

        while (otherPlist) {
            Particle *p2 = &otherPlist->p;
            float deltaX = p2->x - p1->x;
            float deltaY = p2->y - p1->y;
            float distSqrd = deltaX * deltaX + deltaY * deltaY;
            if (distSqrd < RADIUS2 * RADIUS2) {
                // There has been a collision.

                // Collision normal is the vector between the two particle centers.
                // The only change in velocity of either particle is in the 
                // direction of the collision normal.

                // Calculate collision unit normal and tangent.
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

            otherPlist = otherPlist->next;
        }

        plist = plist->next;
    } while (plist);
}



void Particles::Advance() {
    float advanceTime = 0.001f;// g_world.m_advanceTime;

    if (g_window->input.keyDowns[KEY_H])
        m_showHistogram = true;

    if (m_showHistogram)
        memset(m_speedHistogram, 0, sizeof(unsigned) * SPEED_HISTOGRAM_NUM_BINS);

    for (unsigned y = 0; y < GRID_RES_Y; y++) {
        for (unsigned x = 0; x < GRID_RES_X; x++) {
            PList *plist = m_grid + y * GRID_RES_X + x;
            if (plist->IsEmpty())
                continue;

            PList *plistGrid = plist;
            do {
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

                plist = plist->next;
            } while (plist);

            // Do the collisions
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

                HandleAnyCollisions(plistGrid, plistGrid);  // Special one - check cell against itself.

                otherPlist = GetPListFromIndices(x - 1, y - 1);
                if (otherPlist && !otherPlist->IsEmpty()) HandleAnyCollisions(plistGrid, otherPlist);
            }
        }
    }

    for (unsigned y = 0; y < GRID_RES_Y; y++) {
        for (unsigned x = 0; x < GRID_RES_X; x++) {
            PList *plistGrid = m_grid + y * GRID_RES_X + x;
            if (plistGrid->IsEmpty())
                continue;

            PList *plist = plistGrid;
            PList *prevPlist = NULL;
            do {
                // Move this particle into another cell, if needed.
                Particle *p = &plist->p;
                PList *newPlist = GetPListFromCoords(p->x, p->y);
                if (newPlist && plistGrid != newPlist) {
                    AddParticle(p);

                    if (plist == plistGrid) {
                        if (plist->next == NULL) {
                            plist->p.x = INVALID_PARTICLE_X;
                            plist = NULL;
                        }
                        else {
                            PList *toFree = plist->next;
                            *plist = *(plist->next);
                            toFree->next = m_firstFree;
                            m_firstFree = toFree;
                        }
                    }
                    else {
                        prevPlist->next = plist->next;
                        PList *toFree = plist;
                        plist = plist->next;
                        toFree->next = m_firstFree;
                        m_firstFree = toFree;
                    }
                }
                else {
                    prevPlist = plist;
                    plist = plist->next;
                }
            } while (plist);
        }
    }
}


void Particles::Render(DfBitmap *bmp) {
    static const DfColour col = Colour(255, 255, 255, 255);

    for (unsigned y = 0; y < GRID_RES_Y; y++) {
        for (unsigned x = 0; x < GRID_RES_X; x++) {
            PList *plist = m_grid + y * GRID_RES_X + x;
            if (plist->IsEmpty())
                continue;

            do {
                float px = plist->p.x;
                float py = plist->p.y;
                g_world.WorldToScreen(&px, &py);
                if (g_world.m_viewScale < 2.5f)
                    PutPix(bmp, px, py, col);
                else
                    CircleOutline(bmp, px, py, PARTICLE_RADIUS * g_world.m_viewScale, col);
                plist = plist->next;
            } while (plist);
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
    unsigned count = 0;
    PList *plist = m_grid + y * GRID_RES_X + x;
    if (plist->IsEmpty())
        return 0;

    do {
        count++;
        plist = plist->next;
    } while (plist);

    return count;
}


unsigned Particles::Count() {
    unsigned count = 0;
    for (unsigned y = 0; y < GRID_RES_Y; y++)
        for (unsigned x = 0; x < GRID_RES_X; x++)
            count += CountParticlesInCell(x, y);

    return count;
}


void Particles::AddParticle(Particle *p) {
    PList *plist = GetPListFromCoords(p->x, p->y);
    DebugAssert(plist);

    if (plist->IsEmpty()) {
        plist->p = *p;
        return;
    }

    // Get a PList from the free list
    DebugAssert(m_firstFree);
    PList *newPlist = m_firstFree;
    m_firstFree = m_firstFree->next;

    newPlist->p = *p;

    // Insert the new PList at the start of the list for this cell
    newPlist->next = plist->next;
    plist->next = newPlist;
}
