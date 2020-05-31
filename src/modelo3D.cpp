#include "modelo3D.h"
#include <gl/gl.h>
#include <stdint.h>

//Copyright (C) <2017>  <Eliseo Rivera Silva> curso.tareas@gmail.com


modelo3D::modelo3D()
{
    ntriangles=0;

    R.identity(3);  //accumulated rotations
    Rz.identity(3);
    Rx.identity(3);
    Ry.identity(3);
    ///***************
    O={0,0,0};
}

modelo3D::~modelo3D()
{
 delete[] triangulos;

}
modelo3D::modelo3D(int ntriangulos){
triangulos=new Triangle3D[ntriangulos];
ntriangles=ntriangulos;
}
void modelo3D::leer(string nombre){

char head[80] = "";
///************
	archivo.open(nombre.c_str(), std::ios::in | std::ios::binary);
	if(archivo){
	archivo.read(head, 80);

	int32_t size ;
	archivo.read(reinterpret_cast<char *> (&size), sizeof(int32_t));
	ntriangles=size;
	triangulos=new Triangle3D[size];
	Triangle3D triangle;
	vector3d P0, P1, P2;
	vector3d normal;
	float p0[3], p1[3], p2[3], n[3];
	char attribute[2] = "0";
for (int i=0;i<size;i++){

		archivo.read(reinterpret_cast<char *> (&n[0]), 4);
		archivo.read(reinterpret_cast<char *> (&n[1]), 4);
		archivo.read(reinterpret_cast<char *> (&n[2]), 4);
    //  cout<<n[0]<<" , "<<n[1]<<" , "<<n[2]<<endl;
       triangulos[i].N={n[0],n[1],n[2]};
		archivo.read(reinterpret_cast<char *> (&p0[0]), 4);
		archivo.read(reinterpret_cast<char *> (&p0[1]), 4);
		archivo.read(reinterpret_cast<char *> (&p0[2]), 4);
	//	   cout<<p0[0]<<" , "<<p0[1]<<" , "<<p0[2]<<endl;
          triangulos[i].vertices[0]={p0[0],p0[1],p0[2]};


		archivo.read(reinterpret_cast<char *> (&p1[0]), 4);
		archivo.read(reinterpret_cast<char *> (&p1[1]), 4);
		archivo.read(reinterpret_cast<char *> (&p1[2]), 4);
	//	   cout<<p1[0]<<" , "<<p1[1]<<" , "<<p1[2]<<endl;
          triangulos[i].vertices[1]={p1[0],p1[1],p1[2]};

		archivo.read(reinterpret_cast<char *> (&p2[0]), 4);
		archivo.read(reinterpret_cast<char *> (&p2[1]), 4);
		archivo.read(reinterpret_cast<char *> (&p2[2]), 4);
	//	   cout<<p2[0]<<" , "<<p2[1]<<" , "<<p2[2]<<endl;

        triangulos[i].vertices[2]={p2[0],p2[1],p2[2]};
	    archivo.read(attribute, 2);
	}


	archivo.close();
}
else{
        ntriangles=0;
        cout<<"el archivo no se encuentra"<<endl;
}
}
void modelo3D::aplicarRx(float dtheta){


Rx.aij[0][0]=1;
Rx.aij[0][1]=0;
Rx.aij[0][2]=0;

Rx.aij[1][0]=0;
Rx.aij[1][1]=cos(dtheta);
Rx.aij[1][2]=-sin(dtheta);

Rx.aij[2][0]=0;
Rx.aij[2][1]=sin(dtheta);
Rx.aij[2][2]=cos(dtheta);
R=R*Rx;
}
void modelo3D::aplicarRy(float dtheta){


Ry.aij[0][0]=cos(dtheta);
Ry.aij[0][1]=0;
Ry.aij[0][2]=sin(dtheta);

Ry.aij[1][0]=0;
Ry.aij[1][1]=1;
Ry.aij[1][2]=0;

Ry.aij[2][0]=-sin(dtheta);
Ry.aij[2][1]=0;
Ry.aij[2][2]=cos(dtheta);

R=R*Ry;

}
void modelo3D::aplicarRz(float dtheta){

Rz.aij[0][0]=cos(dtheta);
Rz.aij[0][1]=-sin(dtheta);
Rz.aij[0][2]=0;

Rz.aij[1][0]=sin(dtheta);
Rz.aij[1][1]=cos(dtheta);
Rz.aij[1][2]=0;

Rz.aij[2][0]=0;
Rz.aij[2][1]=0;
Rz.aij[2][2]=1;
R=R*Rz;
}


void modelo3D::dibujar(){
    glBegin(GL_TRIANGLES);
    glFrontFace(GL_FRONT_AND_BACK);
vector3d UX,UY,UZ;

    for (int i=0;i<ntriangles;i++){


vector3d v1=R*triangulos[i].vertices[0];   //locales son locales
vector3d v2=R*triangulos[i].vertices[1];
vector3d v3=R*triangulos[i].vertices[2];
vector3d V1,V2, V3;


V1=O+v1; V2=O+v2; V3=O+v3;

vector3d d1,d2,n;
d1=V2-V1;
d2=V3-V1;
n=d1*d2;  ///devuelve el producto vectorial
n.normalize();

glNormal3f(n.x,n.y,n.z);
        glVertex3f(V1.x,V1.y,V1.z);
        glVertex3f(V2.x,V2.y,V2.z);
        glVertex3f(V3.x,V3.y,V3.z);
    }

glEnd();

         Drawarrow3D(O,O+4*R*vector3d(1,0,0),{1,0,0.2},0.05);
         Drawarrow3D(O,O+4*R*vector3d(0,1,0),{.1,1,0.2},0.05);
         Drawarrow3D(O,O+4*R*vector3d(0,0,1),{0.1,0.2,1},0.05);

       //

};











vector3d modelo3D::LocalMassCenter()const
{
 vector3d center;
 for(int i=0;i<ntriangles;i++){
        vector3d v1,v2,v3;
        v1=triangulos[i].vertices[0];
        v2=triangulos[i].vertices[1];
        v3=triangulos[i].vertices[2];
center=center+(1.0/3.0)*(v1+v2+v3);

    }
    center=(1.0/ntriangles)*center;
    cout<<" Local center mass is :"<<endl;
    center.mostrar();
    return center;

}
vector3d modelo3D::GlobalCenterMass(){
vector3d L=LocalMassCenter();



        vector3d C=O+R*L;
    cout<<" Global center mass is: "<<endl;
        C.mostrar();
        return C;

}
void modelo3D::Drawarrow3D( vector3d A,  vector3d B, vector3d color, double R)
{

double color1,color2,color3,a,b,c,d,e;

color1=color.x;//abs(color1/255);
color2=color.y;//abs(color2/255);
color3=color.z;//abs(color3/255);

glColor4f( color1,color2, color3,0.8);

vector3d n=B-A,np,vertex[10],normallight;
n.normalize();
if(n.z!=0)np={1,1,(-1/n.z)*(n.x+n.y)};
else if(n.y!=0)np={1,(-1/n.y)*(n.x+n.z),1};
else np={(-1/n.x)*(n.y+n.z),1,1};

np.normalize();
vertex[0]=R*np;
vertex[2]=R*(n*np).normalize();
vertex[1]=R*((0.5)*(vertex[2]-vertex[0])+vertex[0]).normalize();
vertex[4]=R*(n*vertex[2]).normalize();
vertex[3]=R*((0.5)*(vertex[4]-vertex[2])+vertex[2]).normalize();
vertex[6]=R*(n*vertex[4]).normalize();
vertex[5]=R*((0.5)*(vertex[6]-vertex[4])+vertex[4]).normalize();
vertex[7]=R*((0.5)*(vertex[0]-vertex[6])+vertex[6]).normalize();
vertex[8]=vertex[0];
vertex[9]=vertex[1];
int nx=8;
double d_thetha,fraccion=0.1,radioflecha=R+.7*R;
d_thetha=2.0f*PI/nx;


  ///tubos
 glBegin( GL_TRIANGLE_STRIP );

         for(int i=0;i<9;i++)
               {

normallight=n*(vertex[i-1]-vertex[i+1]);
normallight.normalize();
glNormal3f(normallight.x, normallight.y, normallight.z);
                 glVertex3f(vertex[i].x+A.x,vertex[i].y+A.y,vertex[i].z+A.z);

glVertex3f(vertex[i].x+B.x-fraccion*(B.x-A.x),vertex[i].y+B.y-fraccion*(B.y-A.y),vertex[i].z+B.z-fraccion*(B.z-A.z));

    // top face

                }

glEnd();



//flecha
vertex[0]=radioflecha*np;
vertex[2]=radioflecha*(n*np).normalize();
vertex[1]=radioflecha*((0.5)*(vertex[2]-vertex[0])+vertex[0]).normalize();
vertex[4]=radioflecha*(n*vertex[2]).normalize();
vertex[3]=radioflecha*((0.5)*(vertex[4]-vertex[2])+vertex[2]).normalize();
vertex[6]=radioflecha*(n*vertex[4]).normalize();
vertex[5]=radioflecha*((0.5)*(vertex[6]-vertex[4])+vertex[4]).normalize();
vertex[7]=radioflecha*((0.5)*(vertex[0]-vertex[6])+vertex[6]).normalize();
vertex[8]=vertex[0];
vertex[9]=vertex[1];
vector3d Ap(B-fraccion*(B-A));



 glBegin( GL_TRIANGLE_STRIP );  //flecha

         for(int i=0;i<9;i++)
               {

normallight=n*(vertex[i-1]-vertex[i+1]);
normallight.normalize();
glNormal3f(normallight.x, normallight.y, normallight.z);
                 glVertex3f(vertex[i].x+Ap.x,vertex[i].y+Ap.y,vertex[i].z+Ap.z);


glNormal3f(n.x, n.y, n.z);
glVertex3f(Ap.x+fraccion*(B-A).x,Ap.y+fraccion*(B-A).y,Ap.z+fraccion*(B-A).z);

    // top face

                }

glEnd();


}
