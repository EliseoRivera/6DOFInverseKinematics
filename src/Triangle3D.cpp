#include "Triangle3D.h"
//Copyright (C) <2017>  <Eliseo Rivera Silva> curso.tareas@gmail.com

Triangle3D::Triangle3D()
{
    //ctor
}

Triangle3D::~Triangle3D()
{
    //dtor
}
vector3d Triangle3D::normal(){
vector3d d1,d2,n;
d1=vertices[1]-vertices[0];
d2=vertices[2]-vertices[0];
n=d1*d2;  ///devuelve el producto vectorial
n.normalize();
N=n;  ///se guarda el valor de la normal en el campo N
return n;
}
