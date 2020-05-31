#ifndef ROBOT_H
#define ROBOT_H
#include "modelo3D.h"
#include<vector>
#include <cstdlib>
///Copyright (C) <2017>  <Eliseo Rivera> curso.tareas@gmail.com
class Robot
{
    public:
        Robot();
        ~Robot();

        modelo3D *base;
           modelo3D *b1;
              modelo3D *b2;
                 modelo3D *b3;
                    modelo3D *b4;
                       modelo3D *b5;
                          modelo3D *b6;
                               modelo3D *gripe;

void inicializar();
void renderizar();
void configurarTH();
float t;
///
void Parametrica();
bool InverseKinematics();
double xg,yg,zg;
//
void AplicarTHx(float theta, vector3d d);
void AplicarTHy(float theta, vector3d d);
void AplicarTHz(float theta, vector3d d);
Matrix THx,THy,THz,TH;

std::vector<Matrix> THList;
std::vector<vector3d> Origenes;
std::vector<modelo3D*> modelos;


double theta1, theta2, theta3,theta4, theta5, theta6,theta7;
double z1, z2, z3,z4, z5,z6,z7;
double q1, q2, q3,q4, q5,q6;
double x0,y0,z0;
double x2;
void DibujarCurva();
/// cinematica inversa parte 2//////////////////
Matrix iL, jL, kL;
Matrix k3,k4,k5,k6,k60;
Matrix i3,i4,i5,i6, i60;
Matrix TPL, TPG,TPVL,TPVG,TPG0;
Matrix O4, O40, O5,O6;

void initVectores();
void initInverseKinematics2();
bool InverseKinematics2();
Matrix A, TH06;
///
void Estado();

private :
void DefinirTHx(float theta, vector3d d);
void DefinirTHy(float theta, vector3d d);
void DefinirTHz(float theta, vector3d d);
void  Drawarrow3D( vector3d A,  vector3d B, vector3d color, double cota1,double R) ;
std::vector<vector3d> curva;
double signo(const Matrix &A){
if (A.aij[2][0]>=0) return 1;
else return -1;
}


double menor3 (double a, double b, double c, double d){
    double m;
m=a;

if(fabs(d-b)<fabs(d-m)) m=b;
if (fabs(d-c)<fabs(d-m)) m=c;

return m;
}

double menor4 (double a, double b, double c, double e, double d){
    double m=menor3(a,b,c,d);
    m=menor2(m,e,d);

return m;
}

double menor2 (double a, double b, double d){
    double m;
m=a;

if(fabs(d-b)<fabs(d-m)) m=b;


return m;
}

bool sol1(double a, double b, double d, double &s1,double &s2,double &s3,double &s4){
/*
float phi,c,t,c1;
phi=atan2(a,b);
c=sqrt(a*a+b*b);
c1=sqrt(c*c-d*d);
cout<<"a = " <<a<<" , b= "<< b<<" , d= "<< d<<" , c= "<<c<<" d/c= "<<d/c<<endl;
if (c==0) return false;
if (fabs(d/c)>1) return false;

s1=atan2(d,c1)-phi;
s2=atan2(d,-c1)-phi;
s3=s1;
s4=s2;
*/

float R=sqrt(a*a+b*b);
float dis=a*a+b*b-d*d;
if (dis<0) return false;
if (b==-d) return false;

s1=2*atan((a+sqrt(dis))/(b+d));
s2=2*atan((a-sqrt(dis))/(b+d));

s3=2*(atan((a+sqrt(dis))/(b+d)+PI));
s4=2*(atan((a+sqrt(dis))/(b+d)-PI))   ;

return true;
}

bool sol2(double p, double q, double &s1,double &s2, double  &s3,double &s4){


if (q==0) return false;
if (fabs(p/q)>1) return false;

s1=asin(p/q);
s2=PI-asin(p/q);

s3=-2*PI+asin(p/q);
s4=+2*PI+PI-asin(p/q);


/*

float c1;

c1=sqrt(q*q-p*p);
if (q==0) return false;
if (fabs(p/q)>1) return false;


s1=atan2(p,c1);
s2=atan2(p,-c1);
s3=s1;
s4=s2;
*/
return true;
}

};

#endif // ROBOT_H
