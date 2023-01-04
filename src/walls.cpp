// Own header
#include "walls.h"

// Project headers
#include "maths.h"
#include "particles.h"
#include "world.h"

// Deadfrog headers
#include "df_bitmap.h"

#include <math.h>


static float const WALL_SPHERE_RADIUS = 1.5f;


Walls::Walls()
{
    m_wallBitmap = NULL;
}


bool Walls::IsWallPixel(unsigned x, unsigned y)
{
    unsigned bitWithinByte = x & 7;
    unsigned char bitMask = 1 << bitWithinByte;
    x /= 8;
    unsigned char wallByte = m_wallBitmap[y * m_wallBitmapWidth / 8 + x];
    return !!(wallByte & bitMask);
}


void Walls::SetWallPixel(unsigned x, unsigned y)
{
    unsigned bitWithinByte = x & 7;
    unsigned char bitMask = 1 << bitWithinByte;
    x /= 8;
    unsigned char *wallByte = &m_wallBitmap[y * m_wallBitmapWidth / 8 + x];
    *wallByte |= bitMask;
}


bool Walls::GetWallNormalFromPixel(DfBitmap *bmp, unsigned x, unsigned y, float *resultX, float *resultY)
{
    // Assume the specified pixel is a wall pixel

    static const float R2 = 0.7071068f;  // 1.0 / sqrt(2.0)
                                         // Lookup tables for the normals for each of the compass directions, in the order (N, NE, E, etc)
    float normalsX[8] = { 0.0f, R2, 1.0f, R2, 0.0f, -R2, -1.0f, -R2 };
    float normalsY[8] = { 1.0f, R2, 0.0f, -R2, -1.0f, -R2, 0.0f, R2 };

    int compassDirX[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
    int compassDirY[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };

    *resultX = 0.0f;
    *resultY = 0.0f;

    unsigned numNoneType = 0;
    for (unsigned compassDirIndex = 0; compassDirIndex < 8; compassDirIndex++)
    {
        int px = x + compassDirX[compassDirIndex];
        int py = y + compassDirY[compassDirIndex];
        if (!IsWallPixel(px, py))
        {
            *resultX += normalsX[compassDirIndex];
            *resultY += normalsY[compassDirIndex];
            numNoneType++;
        }
    }

    // If the specified pixel was not connected to any neighbouring wall 
    // pixels, then the normal is undefined. We just pick one anyway.
    if (numNoneType == 0 || numNoneType == 8)
        return false;

    // Re-normalize result
    float len = sqrtf(*resultX * *resultX + *resultY * *resultY);
    *resultX /= len;
    *resultY /= len;

    return true;
}


void Walls::Load(DfBitmap *bmp)
{
    m_wallBitmapWidth = bmp->width;
    m_wallBitmapHeight = bmp->height;
    unsigned bitmapSize = bmp->width * bmp->height / 8;
    m_wallBitmap = new unsigned char [bitmapSize];
    memset(m_wallBitmap, 0, bitmapSize);

    for (unsigned y = 0; y < bmp->height; y++)
    {
        for (unsigned x = 0; x < bmp->width; x++)
        {
            if (GetPix(bmp, x, y).g > 128)
                SetWallPixel(x, y);
        }
    }

    for (unsigned y = 0; y < bmp->height; y++)
    {
        for (unsigned x = 0; x < bmp->width; x++)
        {
            if (IsWallPixel(x, y))
            {
                WallSphere ws;
                ws.x = x;
                ws.y = y;
                if (GetWallNormalFromPixel(bmp, x, y, &ws.normalX, &ws.normalY))
                    m_wallSpheres.push_back(ws);

            }
        }
    }
}


void Walls::Advance()
{
    for (unsigned i = 0; i < m_wallSpheres.size(); i++)
    {
        WallSphere const &ws = m_wallSpheres[i];
        Particles::PList *gc = g_world.m_particles->GetPListFromCoords(ws.x, ws.y);

        if (gc->IsEmpty())
            continue;

        do
        {
            Particle *p = &gc->p;

            float deltaX = ws.x - p->x;
            float deltaY = ws.y - p->y;
            float dist = sqrtf(deltaX * deltaX + deltaY * deltaY);
            if (dist < (WALL_SPHERE_RADIUS + PARTICLE_RADIUS))
            {
                // Ignore collisions where particle is already moving away 
                // from wall (ie within 90 degrees of the wall's normal).
                float dp = DotProduct(p->vx, p->vy, ws.normalX, ws.normalY);
                if (dp < 0.0f)
                {
                    // Reflect particle velocity about the surface normal.
                    p->vx -= 2.0f * (dp * ws.normalX);
                    p->vy -= 2.0f * (dp * ws.normalY);
                }
            }

            gc = gc->next;
        } while (gc);
    }
}


void Walls::Render(DfBitmap *bmp)
{
    DfColour col = Colour(50, 255, 25);
    for (unsigned i = 0; i < m_wallSpheres.size(); i++)
    {
        WallSphere const &ws = m_wallSpheres[i];

        float cx = ws.x;
        float cy = ws.y;
        g_world.WorldToScreen(&cx, &cy);

        if (g_world.m_viewScale < 1.5f)
            PutPix(bmp, cx, cy, col);
        else
            CircleOutline(bmp, cx, cy, WALL_SPHERE_RADIUS * g_world.m_viewScale, col);

        float l1x = ws.x;
        float l1y = ws.y;
        float l2x = ws.x + ws.normalX * 3.0f;
        float l2y = ws.y + ws.normalY * 3.0f;
        g_world.WorldToScreen(&l1x, &l1y);
        g_world.WorldToScreen(&l2x, &l2y);
//        DrawLine(bmp, l1x, l1y, l2x, l2y, Colour(255, 50, 50));
    }
}
