#include "vector3d.h"
#include <iostream>
using namespace std;
vector3d::vector3d(double _x, double _y, double _z): x(_x), y(_y), z(_z) {};
vector3d::vector3d(){ x=0;  y=0; z=0;};
vector3d::vector3d (const vector3d &p){x=p.x;y=p.y;z=p.z;};
vector3d vector3d::operator+(const vector3d &p)const
{ return vector3d(x + p.x, y + p.y, z + p.z); }
vector3d vector3d::operator*(const vector3d &p)const
{ return vector3d(y*p.z-z*p.y, z*p.x-x*p.z ,x*p.y-y*p.x); }
vector3d vector3d::operator-(const vector3d &p) const
{ return vector3d(x - p.x, y - p.y, z - p.z); }
vector3d operator*(const double &s,const vector3d &p){
    return vector3d(s * p.x, s * p.y, s * p.z);
}
int vector3d::operator==(vector3d &p)
{ return ((x == p.x) && (y == p.y) && (z == p.z));}
int vector3d::operator!=(vector3d &p)
{ return !(*this == p); }
double& vector3d::operator[](int i)
{ return ((i == 0) ? x : ((i == 1) ? y : z)); }
double vector3d:: dotProduct(const vector3d &p)
{ return (x*p.x + y*p.y + z*p.z); }

double vector3d::magnitude() const
{ return sqrt(x*x + y*y + z*z);
};
vector3d&   vector3d::normalize(){
double a=magnitude();
if (a!=0){
     x=x/a;  y=y/a; z=z/a;
};
return *this;
 }
  vector3d&  vector3d::operator =( const vector3d &B){
    x=B.x;  y=B.y; z=B.z;
 return *this;
 }
vector3d::~vector3d(){}
  void vector3d::mostrar(){
          cout<<" x = "<<x<<endl;
          cout<<" y = "<<y<<endl;
          cout<<" z = "<<z<<endl;


      }
