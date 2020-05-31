#ifndef CLASEOPENGL_H
#define CLASEOPENGL_H
#include <gl/gl.h>
#include<GL/GLU.h>
#include "Robot.h"
//Copyright (C) <2017>  <Eliseo Rivera Silva> curso.tareas@gmail.com

///Copyright (C) <2017>  <Eliseo Rivera> curso.tareas@gmail.com
#include"modelo3D.h"
#define PI 3.14159265
class ClaseOpenGL
{
    public:
        ClaseOpenGL();
        virtual ~ClaseOpenGL();
        void inicializar();
        void preparar(float dt);
        void renderizar();
        void Resize(int width, int height);
        float theta;
        void  Drawarrow3D( vector3d A,  vector3d B, vector3d color, double cota1,double R=0.01);

        float  cameraX,  cameraZ, cameraY ,   mouseX,mouseY, camerafactor,angle;
        float Rcamera, phiCamera, thetaCamera;
        float radians;

        vector3d rojo={1,0.0,0.0};
        vector3d verde={0.0,1,0.0};
        vector3d azul={0.0,0.0,1};
        void dibujarBodyFrame(const modelo3D &modelo);

        Robot robot;
};

#endif // CLASEOPENGL_H
