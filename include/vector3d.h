#ifndef vector3d_H
#define vector3d_H
#include <math.h>

class vector3d
{
    public:
    double x;
    double y;
    double z;
    vector3d(double _x, double _y, double _z);
    vector3d();
    ~vector3d();
    vector3d (const vector3d &p);
    vector3d operator+(const vector3d &p)const;
    vector3d operator*(const vector3d &p)const;
    vector3d operator-(const vector3d &p)const;
    friend vector3d operator*(const double &s, const vector3d &p);
    int operator==(vector3d &p);
    int operator!=(vector3d &p);
    double& operator[](int i);
    double dotProduct(const vector3d &p);
    double magnitude() const;
    vector3d&   normalize();
    vector3d&  operator =( const vector3d &B);

    void mostrar();
};
#endif // vector3d_H
