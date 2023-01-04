#pragma once

#include <vector>

typedef struct _DfBitmap DfBitmap;


// A slightly weird geometrical concept this: a sphere with a normal and no radius!
// The idea is that all the WallSphere's have the same radius, so there's no
// point in storing it for each one. The position and radius are used to determine
// collisions. If a collision occurs, the normal is used to tell the particle which
// way to bounce. A continuous string of overlapping WallSpheres will be created
// around every wall in the model. As a result, when a particle collides with a
// WallSphere we are trying to make it bounce off the wall surface that is 
// represented by the string of WallSpheres. The nice thing here is that a collision
// only needs to consider one WallSphere and one particle.
struct WallSphere
{
    float x, y;
    float normalX, normalY;
};


class Walls
{
private:
    // This data structure is a simple 1 bit per pixel bitmap of locations occupied by wall. It
    // is used in particle creation to make sure the particle is not created 
    // inside a wall. It is not ideal for collision detection of moving particles
    // since no surface normal is defined, and therefore you can't easily tell
    // what to do with a particle that has collided.
    unsigned char *m_wallBitmap;
    unsigned m_wallBitmapWidth;
    unsigned m_wallBitmapHeight;
    
    bool GetWallNormalFromPixel(DfBitmap *bmp, unsigned x, unsigned y, float *resultX, float *resultY);

public:
    // This data structure just models the surface of the walls and is used
    // in collision detection against moving particles. If a particle somehow
    // got through the surface layer of the wall, it would not collide with
    // the interior of the wall.
    std::vector<WallSphere> m_wallSpheres;

    Walls();
    void Load(DfBitmap *bmp);

    bool IsWallPixel(unsigned x, unsigned y);
    void SetWallPixel(unsigned x, unsigned y);

    void Advance();
    void Render(DfBitmap *bmp);
};
