#ifndef MODELO3D_H
#define MODELO3D_H
#include"Triangle3D.h"
#include"Matrix.h"
#include<fstream>
#include <iostream>

#include <gl/gl.h>
#include<GL/GLU.h>
#define PI 3.14159265
#include <string>
//Copyright (C) <2017>  <Eliseo Rivera Silva> curso.tareas@gmail.com

using namespace std;
class modelo3D
{
    public:
int ntriangles;
Triangle3D* triangulos;
modelo3D(int ntriangulos);
Matrix Rx,Ry,Rz;  //local
modelo3D();


void dibujar();
void aplicarRx(float theta);
void aplicarRy(float theta);
void aplicarRz(float theta);
ifstream archivo;
void leer(string nombre);
virtual ~modelo3D();
vector3d O;
Matrix R;
vector3d LocalMassCenter()const;
vector3d GlobalCenterMass();

void Drawarrow3D( vector3d A,  vector3d B, vector3d color, double R);
vector3d color;

};

#endif // MODELO3D_H
