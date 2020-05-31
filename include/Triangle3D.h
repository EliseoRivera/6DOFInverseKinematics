#ifndef TRIANGLE3D_H
#define TRIANGLE3D_H
#include"vector3d.h"

class Triangle3D
{
public:
Triangle3D();
~Triangle3D();
vector3d vertices[3];
vector3d N;
vector3d normal();
void definirRz(float dtheta);
void dibujar();
void rotar(float dtheta);
};

#endif // TRIANGLE3D_H
